# 백엔드 핵심 서버 코드
# [Architecture] ROS 2(Thread)와 FastAPI(AsyncIO)의 실행 모델 분리를 위한 Queue 기반 Producer-Consumer 구조.
# RobotBridge(ROS)는 오직 Queue 적재만 담당하며, 웹소켓(ConnectionManager) 및 HTTP API와 완전 분리되어 결합도를 낮춤.
# 이를 통해 스레드 간 경계를 명확히 하고 비동기 I/O 병목을 해결하여 실시간 데이터 브리지의 안정성을 확보함.

# 코드는 FastAPI의 비동기 이벤트 루프와 ROS 2의 스레드라는 서로 다른 실행 모델을 asyncio.Queue 기반의 Producer-Consumer 패턴으로 완벽하게 중재하고 있음.

import asyncio
import logging
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from database import init_db
from robot_bridge import bridge_manager
from connection_manager import ConnectionManager
from routers import robot_router
from datetime import datetime
import time


logging.basicConfig(level="INFO", format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)

## 초기 선언 및 글로벌 객체 생성

# ROS Callback → Queue → Background Task → ConnectionManager.broadcast() 구조의
# 중심이 되는 두 객체. main.py 모듈 레벨에 두어 lifespan/엔드포인트에서 공유
broadcast_queue: asyncio.Queue = asyncio.Queue(maxsize=1000)
# ROS 2가 던져준 데이터를 임시로 담아두는 비동기 큐. 최대 크기(maxsize=1000)를 지정해 메모리 폭주를 막음 
connection_manager = ConnectionManager()
# 앞서 분석한 웹소켓 관리자 인스턴스

_consumer_task: asyncio.Task | None = None
# 백그라운드에서 평생 돌며 큐를 감시할 타스크 객체를 저장할 변수


async def _queue_consumer_loop() -> None:
    """
    Queue에서 ROS 상태 데이터를 꺼내 프론트엔드 HMI 포맷으로 정제한 뒤
    ConnectionManager.broadcast()로 넘기는 유일한 Background Task.
    (ROS Callback → Queue → [데이터 정제] → 여기 → WebSocket)
    """
    print("Queue consumer 루프 시작.", flush=True)

    # 최신 상태를 누적 유지하는 캐시 (함수 호출 1회 동안 while 루프에서 계속 유지됨)
    latest_status = {
        "process_state": "대기 중 (사용자 이용 전)",
        "task_id": None,
        "gripper_state": "UNKNOWN",
        "joints": {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0, "J5": 0.0, "J6": 0.0},
    }

    try:
        while True:
            # 1) ROS 2 브리지 스레드가 적재한 원본 raw 데이터 추출 (dict 형태 가정)
            raw_ros_data = await broadcast_queue.get()
            try:
                msg_type = raw_ros_data.get("type")

                # 2) 들어온 메시지 타입에 따라 최신 상태 캐시만 갱신 (나머지는 유지)
                if msg_type == "PROCESS_STATE":
                    latest_status["process_state"] = raw_ros_data.get("payload", latest_status["process_state"])
                elif msg_type == "GRIPPER_STATUS":
                    latest_status["gripper_state"] = "GRASPED" if raw_ros_data.get("grasped") else "OPEN"
                elif msg_type == "SAFETY_EVENT":
                    latest_status["last_safety_event"] = {
                        "error_code": raw_ros_data.get("error_code"),
                        "error_msg": raw_ros_data.get("error_msg"),
                        "timestamp": raw_ros_data.get("timestamp", time.time()),
                    }
                # MOTION_STATUS, joints 갱신용 토픽이 추가되면 여기에 elif로 계속 확장

                # 3) 현재 GUI(process_queue)가 인식하는 형태로 PROCESS_STATE 브로드캐스트
                #    -> data.get("type") == "PROCESS_STATE", data.get("payload")가 문자열이어야 함
                if msg_type == "PROCESS_STATE":
                    await connection_manager.broadcast({
                        "type": "PROCESS_STATE",
                        "payload": latest_status["process_state"],
                        "timestamp": time.time(),
                    })
                elif msg_type == "SAFETY_EVENT":
                    await connection_manager.broadcast({
                        "type": "SAFETY_EVENT",
                        "error_code": raw_ros_data.get("error_code"),
                        "error_msg": raw_ros_data.get("error_msg"),
                        "timestamp": raw_ros_data.get("timestamp", time.time()),
                    })

                # 4) 누적 캐시 전체("ROBOT_STATUS")도 함께 브로드캐스트
                #    -> 지금 GUI는 무시하지만, 추후 관절각/그리퍼 표시 추가 시 바로 쓸 수 있음
                formatted_payload = {
                    "type": "ROBOT_STATUS",
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "payload": dict(latest_status),
                }
                await connection_manager.broadcast(formatted_payload)

            except Exception:  # noqa: BLE001
                logger.error("데이터 정제 및 웹소켓 broadcast 중 에러 발생", exc_info=True)
    except asyncio.CancelledError:
        # 서버가 종료될 때 이 루프를 안전하게 취소(cancel)하여 자원을 깔끔하게 해제
        print("Queue consumer 루프 취소됨, 정상 종료.", flush=True)
        raise
# 데이터 파이프라인의 생명주기(Lifespan)와 결합도로 robot_router가 아닌 main에서 GET /ws/robot/status
# 웹소켓 엔드포인트 함수(websocket_endpoint)는 클라이언트가 들어오면 connection_manager.connect(websocket)를 호출해야 하므로, 이 인스턴스에 반드시 접근할 수 있어야함

## 애플리케이션 생명주기 관리 (lifespan)
# FastAPI 서버가 켜질 때(Startup)와 꺼질 때(Shutdown) 작동하는 컨트롤러

@asynccontextmanager
async def lifespan(app: FastAPI):
    global _consumer_task
    try:
        print("lifespan start")

        init_db() # 1) DB 초기화

        current_loop = asyncio.get_running_loop()
        # robot_bridge는 이제 connection_manager를 모르고, broadcast_queue만 받음
        # 2) ROS 2 브리지 스레드 가동
        bridge_manager.start_bridge(current_loop, broadcast_queue)
        # 현재 FastAPI가 돌고 있는 비동기 이벤트 루프(current_loop)와 데이터 바구니(broadcast_queue)를 ROS 2 브리지에 넘겨주며 스레드를 실행함.
        # 이제 robot_bridge는 웹소켓이 뭔지 몰라도 이 큐에 데이터를 밀어 넣을 수 있게 됨 

        # 3) 백그라운드 큐 소비자 가동
        _consumer_task = asyncio.create_task(_queue_consumer_loop())
        # : 앞서 만든 소비자 루프(_queue_consumer_loop)를 백그라운드에서 비동기로 독립시켜 상시 구동

        print("lifespan startup finished")

        yield
        # 이 지점에서 서버가 정상 가동되며 사용자의 요청을 받기 시작

    except Exception as e:
        print("LIFESPAN EXCEPTION:", repr(e), flush=True)
        raise

    finally:
        # # 서버가 꺼질 때 안전하게 청소(Graceful Shutdown)
        print("lifespan shutdown", flush=True)

        # 1) consumer task를 먼저 취소
        if _consumer_task is not None:
            _consumer_task.cancel() # 소비자 정지
            try:
                await _consumer_task
            except asyncio.CancelledError:
                pass

        # 2) ROS spin 스레드 정리 + rclpy.shutdown
        bridge_manager.shutdown() # ROS 2 스레드 및 rclpy 안전 종료
        # 서버가 꺼질 때 백그라운드 태스크를 먼저 취소하고, ROS 2 스레드까지 안전하게 셧다운(bridge_manager.shutdown())하여 좀비 프로세스가 남는 것을 원천 차단

app = FastAPI(
    title="AutoDumpBot Backend API",
    description="음식물 수거 로봇 자동화 시스템을 위한 백엔드 제어 및 이력 관리 API",
    version="1.0.0",
    lifespan=lifespan,
)
# CORS 미들웨어 설정 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 개발 단계에서는 전체 허용, 추후 프론트 주소만 지정 가능
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(robot_router.router)

@app.get("/")
def read_root():
    return {"message": "AutoDumpBot Backend Server is Running!"}

## 웹소켓 엔드포인트 (/ws/robot/status)
# 기존: bridge_manager.add_connection / remove_connection을 직접 호출
# 변경: connection_manager에 등록만 함. robot_bridge는 이 함수의 존재를 모름.
# 프론트엔드가 웹소켓을 연결하면 connection_manager 주머니에 넣고 관리

# 이 엔드포인트는 오직 클라이언트의 접속과 해제만 관리
# 로봇이 데이터를 보내든 말든 상관하지 않으며, 데이터 전송은 오직 백그라운드 소비자(_queue_consumer_loop)가 담당하므로 코드가 매우 단단해짐
@app.websocket("/ws/robot/status")
async def websocket_endpoint(websocket: WebSocket):
    await connection_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await connection_manager.disconnect(websocket)