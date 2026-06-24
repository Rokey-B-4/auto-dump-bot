"""
robot_bridge.py
================
백그라운드 스레드에서 ROS2 노드를 spin하고, 토픽이 들어올 때마다
asyncio.Queue에 데이터를 넣는 역할만 전담

기존 코드와의 차이점:
  - 기존: _status_callback에서 asyncio.run_coroutine_threadsafe로
    WebSocket.send_json()을 직접 호출 (ROS가 WebSocket을 알고 있었음)
  - 변경: _status_callback은 call_soon_threadsafe로 메인 이벤트 루프의
    asyncio.Queue에 데이터를 넣기만 함. WebSocket은 전혀 모름.

  - 기존: self.executor.spin() (블로킹, 종료 신호를 줄 수 없음)
  - 변경: spin_once(timeout_sec=...)를 반복하며 stop_event를 체크
    → shutdown()을 안전하게 구현 가능

active_connections 리스트와 add_connection/remove_connection은
connection_manager.py로 이동했으므로 이 파일에서는 제거했음 

이 코드는 "ROS 2의 스레드 환경"에서 들어오는 실시간 데이터를 "FastAPI의 비동기(AsyncIO) 루프"로 
안전하게 던져주는 스레드 안전(Thread-safe) 브리지 역할을 완벽하게 수행하고 있음
"""

import logging
import threading
import time
from asyncio import AbstractEventLoop, Queue, QueueFull

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String  # 실제 로봇 상태 토픽 메시지 타입에 맞게 조정 가능

logger = logging.getLogger(__name__)

ROS_NODE_NAME = "fastapi_robot_bridge"
ROS_TOPIC_NAME = "/robot_status"
SPIN_TIMEOUT_SEC = 0.1

class RobotBridgeManager:
    """ROS2 Node + Executor + Thread 생명주기를 관리하는 매니저."""
    # 리지용 노드 이름과 구독할 토픽명("/robot_status"), 그리고 CPU를 과점유하지 않고 안전한 셧다운을 하기 위한 루프 주기(0.1초)를 선언

    def __init__(self) -> None:
        self.node: Node | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.ros_thread: threading.Thread | None = None
        self.loop: AbstractEventLoop | None = None
        self.queue: Queue | None = None
        self._stop_event = threading.Event()
        # ★ 종료 제어의 핵심. 스레드 간에 "이제 그만 멈춰!"라는 신호를 안전하게 주고받기 위한 플래그(Flag) 객체

    # ------------------------------------------------------------------
    def start_bridge(self, fastapi_loop: AbstractEventLoop, output_queue: Queue) -> None:
        """
        FastAPI lifespan에서 호출되어 백그라운드 스레드로 ROS2를 구동함

        Args:
            fastapi_loop: uvicorn이 돌고 있는 메인 이벤트 루프.
                          ROS 콜백 스레드에서 이 루프로 작업을 위임하기 위해 필요.
            output_queue: ROS에서 수신한 데이터가 쌓이는 큐.
                          이 큐를 누가 소비하는지(WebSocket인지)는 여기서 알 필요 없음.

        main.py의 lifespan 시점에 호출되어, 메인 비동기 루프(fastapi_loop)와 데이터 바구니(output_queue)를 인자로 받아옴 
        """
        if not rclpy.ok():
            rclpy.init(args=None) # ROS 2 통신 컨텍스트를 초기화

        self.node = Node(ROS_NODE_NAME)
        # Node(ROS_NODE_NAME): 백엔드가 ROS 네트워크에서 인식될 노드를 생성하고, 메인 루프와 큐를 클래스 내부에 저장
        self.loop = fastapi_loop
        self.queue = output_queue

        # 1. 로봇 모션/상태 토픽 구독(Subscriber) 설정
        self.subscription = self.node.create_subscription(
            # Subscriber 등록: "/robot_status" 토픽으로 메시지(String)가 들어올 때마다 _status_callback 함수가 실행되도록 구독을 시작
            String,
            ROS_TOPIC_NAME,
            self._status_callback,
            10,
        )

        # 2. 기존과 동일하게 타이머도 유지 (헬스체크/디버깅 용도)
        self.timer = self.node.create_timer(1.0, self.timer_callback)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        # Executor 등록: 노드의 이벤트를 처리해 줄 실행기(Executor)에 방금 만든 노드를 등록

        print("Nodes:", self.executor.get_nodes())

        self._stop_event.clear()
        self.ros_thread = threading.Thread(
            target=self._spin_loop, name="ros2-spin-thread", daemon=True
        )
        self.ros_thread.start()
        # 독립 스레드 분리: FastAPI 메인 스레드가 ROS 2 통신 때문에 멈추면(Blocking) 안 되므로, 
        # 별도의 백그라운드 스레드(ros2-spin-thread)를 파서 ROS 루프(_spin_loop)를 실행시킴 
        # daemon=True로 지정하여 메인 프로세스가 종료되면 함께 강제 종료되도록 안전장치를 둠 

    def shutdown(self) -> None:
        """spin 루프를 안전하게 멈추고 ROS 리소스를 정리합니다."""
        print("RobotBridgeManager shutdown 시작...", flush=True)

        self._stop_event.set() # 1) 스레드 루프 종료 신호

        if self.ros_thread is not None:
            self.ros_thread.join(timeout=5.0) # 2) 스레드가 죽을 때까지 최대 5초 대기
            if self.ros_thread.is_alive():
                logger.warning("spin 스레드가 timeout 내에 종료되지 않았습니다.")

        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
        if self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        # 노드를 파괴하고 ROS 2 컨테이너 시스템을 깨끗하게 정리(rclpy.shutdown())

        print("RobotBridgeManager shutdown 완료.", flush=True)

    # ------------------------------------------------------------------
    def _spin_loop(self) -> None:
        """
        별도 스레드에서 실행. executor.spin() 대신 spin_once를 반복하여
        _stop_event를 주기적으로 체크할 수 있게 함

        기존의 executor.spin()은 무한 대기에 빠져 외부에서 멈출 방법이 없었음
        변경된 구조에서는 spin_once(timeout_sec=0.1)를 사용하여 0.1초 동안만 이벤트를 체크하고, 
        즉시 빠져나와 _stop_event가 켜졌는지 확인함 덕분에 서버 종료 요청에 즉각적으로 반응하는 유연한 서브루프가 완성되었음!
        """
        assert self.executor is not None
        print("ROS2 spin 루프 시작.", flush=True)

        while not self._stop_event.is_set():
            try:
                self.executor.spin_once(timeout_sec=SPIN_TIMEOUT_SEC)
            except rclpy.executors.ExternalShutdownException:
                # Ctrl+C(SIGINT) 등으로 rclpy.shutdown()이 외부에서 호출된 경우.
                # 이건 정상적인 종료 신호이므로 ERROR로 찍지 않고 즉시 루프를 빠져나감 
                print("ExternalShutdownException 감지, spin 루프 종료.", flush=True)
                self._stop_event.set()
                break
            except Exception:  # noqa: BLE001
                logger.error("spin_once 루프에서 예외 발생", exc_info=True)
                time.sleep(0.5)

        print("ROS2 spin 루프 종료.", flush=True)

    def timer_callback(self) -> None:
        print("Timer works!", flush=True)

    def _status_callback(self, msg: String) -> None:
        """
        ROS 콜백 스레드에서 실행됨.

        절대 이 안에서 WebSocket이나 connection_manager를 직접 호출하지 않고,
        call_soon_threadsafe로 메인 이벤트 루프의 Queue에만 데이터를 넘김

        문제의 원인이었던 부분 완벽 해결: 이 함수는 FastAPI가 아닌 ROS 2 스레드 위에서 실행됨
        여기서 AsyncIO 영역인 웹소켓을 직접 건드리면 스레드가 꼬여서 터졌던 것
        """
        print(f"ROS2 Topic 수신]: {msg.data}", flush=True)

        payload = {
            "status": msg.data,
            "timestamp": time.time(),
        }

        if self.loop is not None and self.queue is not None:
            self.loop.call_soon_threadsafe(self._enqueue, payload)
            # ROS 스레드는 오직 데이터 팩(payload)만 예쁘게 포장한 뒤, 
            # 메인 비동기 루프(self.loop)에게 "이봐 메인 루프, 너 여유로울 때 이 _enqueue 함수 좀 실행해 줘!" 하고 안전하게 예약을 함 

    def _enqueue(self, payload: dict) -> None:
        """메인 이벤트 루프 안에서 실행됨 (call_soon_threadsafe에 의해 스케줄됨)."""
        try:
            self.queue.put_nowait(payload)
        except QueueFull:
            # 큐가 가득 찼으면 가장 오래된 데이터를 버리고 최신 데이터를 우선함
            try:
                self.queue.get_nowait() # 큐가 꽉 찼으면 가장 오래된 데이터 하나를 빼서 버림
            except Exception:  # noqa: BLE001
                pass
            try:
                self.queue.put_nowait(payload) # 그리고 최신 데이터를 욱여넣음, 비동기 큐에 데이터를 대기 없이 즉시 집어넣음 
            except QueueFull:
                logger.warning("Queue가 여전히 가득 차 있어 메시지를 버립니다.")

            # 링 버퍼(Ring Buffer) 모방 안전장치: 
            # 만약 웹소켓 전송이 너무 느려져서 큐가 가득 차면(QueueFull), 
            # 가장 오래된 밀린 데이터 하나를 과감히 버리고(get_nowait) 그 자리에 최신 로봇 상태 좌표를 넣음
            # 로봇 제어 및 모니터링에서는 지나간 과거 데이터보다 현재 시점의 최신 상태가 훨씬 중요하기 때문에, 시스템 먹통을 막는 매우 훌륭하고 실전적인 예외 처리 기법

# 어디서나 싱글톤처럼 접근 가능하도록 전역 인스턴스 생성 (기존과 동일한 패턴 유지)
bridge_manager = RobotBridgeManager()