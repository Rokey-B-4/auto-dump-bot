"""
connection_manager.py
======================
WebSocket 연결의 생명주기(connect / disconnect / broadcast)만 책임짐

기존 robot_bridge.py의 RobotBridgeManager가 직접 들고 있던
active_connections 리스트와 send_json 호출을, 별도 모듈로 분리했음

이 파일은 rclpy를 import하지 않음! ROS가 뭔지 전혀 몰라도 됨.

이 파일은 백엔드와 프론트엔드(웹 화면 또는 Tkinter 대시보드) 간의 "실시간 통신 전용 게이트웨이"
이 모듈의 유일한 목적은 현재 연결된 웹소켓 클라이언트들을 관리하고, 
데이터가 들어왔을 때 그들에게 안전하게 뿌려주는(broadcast)
"""

import asyncio
# 웹소켓 통신은 비동기(async/await)로 동작하므로, 비동기 동기화 객체(Lock)를 쓰기 위해 가져옴
import logging
from fastapi import WebSocket
# FastAPI가 제공하는 웹소켓 연결 객체 타입을 지정하기 위해 가져옴

logger = logging.getLogger(__name__)
# 커넥션 실패나 에러 상황을 기록하기 위해 파일 단위 로거를 생성

class ConnectionManager:
    def __init__(self) -> None:
        self._active_connections: set[WebSocket] = set()
        # 현재 연결된 웹소켓 클라이언트들을 담는 곳
        # 중복을 허용하지 않고, 추가/제거가 빠른 파이썬의 set(집합) 자료형 선택
        self._lock = asyncio.Lock()
        # ★ 핵심 안전장치. 여러 클라이언트가 동시에 접속하거나 끊길 때, _active_connections라는 하나의 주머니를 동시에 수정하면 데이터가 깨질 수 있음
        # 이를 막기 위해 한 번에 한 녀석만 주머니에 손을 댈 수 있도록 줄을 세워주는 락(Lock)을 생성

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        # 웹소켓 프로토콜 연결 요청을 정식으로 승인(Handshake)
        async with self._lock:
            # 락을 획득. 이 블록 안의 코드가 실행되는 동안 다른 비동기 태스크는 이 주머니를 건드릴 수 없음
            self._active_connections.add(websocket)
            # 안전하게 활성 연결 목록에 추가
        print(f"웹소켓 커넥션 추가됨 (총 {len(self._active_connections)}개)", flush=True)
        # 출력 버퍼를 즉시 비워 도커 로그 등에서 실시간으로 프린트가 찍히도록 함 

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._active_connections.discard(websocket)
            # 클라이언트가 창을 닫거나 연결이 끊기면 목록에서 지움
            # remove() 대신 discard()를 쓰면, 이미 목록에 없는 대상을 지우려고 해도 에러(KeyError)를 내지 않고 안전하게 넘어감
        print(f"웹소켓 커넥션 제거됨 (남은 {len(self._active_connections)}개)", flush=True)

    async def broadcast(self, message: dict) -> None:
        """모든 활성 연결에 JSON 메시지를 전송. 끊긴 연결은 자동 정리."""
        async with self._lock:
            targets = list(self._active_connections)
            # 락을 잠깐 걸어서 현재 접속된 클라이언트들의 스냅샷(복사본)을 얼른 만들고 락을 바로 품
            # 왜냐하면 데이터를 전송(send_json)하는 동안 락을 계속 쥐고 있으면, 그 사이에 새로 접속하려는 클라이언트들이 하염없이 대기해야 하기 때문

        if not targets:
            return

        dead_connections: list[WebSocket] = []
        for connection in targets:
            try:
                await connection.send_json(message)
                # 복사해 둔 클라이언트 리스트를 돌며 딕셔너리 데이터를 JSON 형태로 전송
            except Exception as exc:  # noqa: BLE001
                # 네트워크 단절 등으로 전송에 실패하면, 프로그램이 뻗지 않도록 예외 처리를 하고 dead_connections(유령 커넥션) 명단에 올림
                logger.warning("WebSocket send 실패, 정리 대상으로 등록: %s", exc)
                dead_connections.append(connection)

        if dead_connections:
            # 좀비 커넥션 청소부: 전송 과정에서 에러가 났던 유령 커넥션들을 모아서 원본 주머니(_active_connections)에서 깔끔하게 지워버림
            # 이 덕분에 끊긴 클라이언트가 메모리에 계속 남아있는 누수 현상을 완벽히 방지
            async with self._lock:
                for dead in dead_connections:
                    self._active_connections.discard(dead)

    @property
    # @property: 메서드를 함수처럼 manager.connection_count()로 호출하지 않고, 
    # 변수처럼 manager.connection_count로 편하게 조회할 수 있게 해주는 파이썬 데코레이터
    def connection_count(self) -> int:
        return len(self._active_connections)
    
"""
웹소켓의 '접속 -> 전송 -> 실패 시 청소' 메커니즘이 이 클래스 하나에 완벽히 캡슐화되어 있음
다른 파일(예: Background Consumer 등)에서는 이 복잡한 과정이나 Lock 유무를 신경 쓸 필요 없이, 
그냥 manager.broadcast(로봇데이터) 한 줄만 호출하면 끝
"""