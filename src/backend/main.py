# 백엔드 핵심 서버 코드
# [Architecture] ROS 2(Thread)와 FastAPI(AsyncIO)의 실행 모델 분리를 위한 Queue 기반 Producer-Consumer 구조.
# RobotBridge(ROS)는 오직 Queue 적재만 담당하며, 웹소켓(ConnectionManager) 및 HTTP API와 완전 분리되어 결합도를 낮춤.
# 이를 통해 스레드 간 경계를 명확히 하고 비동기 I/O 병목을 해결하여 실시간 데이터 브리지의 안정성을 확보함.

# 코드는 FastAPI의 비동기 이벤트 루프와 ROS 2의 스레드라는 서로 다른 실행 모델을 asyncio.Queue 기반의 Producer-Consumer 패턴으로 완벽하게 중재하고 있음.

import asyncio
import logging
from contextlib import asynccontextmanager
from datetime import datetime

import uuid

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect

from database import init_db, get_db_connection
from models import TaskStartRequest, ErrorLogRequest
from robot_bridge import bridge_manager
from connection_manager import ConnectionManager

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
    Queue에서 ROS 상태 데이터를 꺼내 ConnectionManager.broadcast()로 넘기는
    유일한 Background Task. (ROS Callback → Queue → 여기 → WebSocket)
    """
    print("Queue consumer 루프 시작.", flush=True)
    try:
        while True:
            payload = await broadcast_queue.get()
            # ★ 비동기 아키텍처의 핵심. 
            # 큐에 데이터가 들어올 때까지 이 라인에서 대기(await)함. 데이터가 들어오면 깨어나서 즉시 다음 줄로 넘어감. 
            try:
                await connection_manager.broadcast(payload)
                # 꺼내온 로봇 상태 데이터를 연결된 모든 웹소켓 클라이언트에게 실시간으로 쏨 
            except Exception:  # noqa: BLE001
                logger.error("broadcast 중 에러 발생", exc_info=True)
    except asyncio.CancelledError:
        # 버가 종료될 때 이 루프를 안전하게 터트려(cancel) 자원을 깔끔하게 해제
        print("Queue consumer 루프 취소됨, 정상 종료.", flush=True)
        raise

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


# [핵심 API 1] 배출 작업 시작 및 ID 생성 (/api/dump/start)
@app.post("/api/dump/start", summary="배출 작업 시작 및 ID 생성 (REQ-01)")
# Tkinter 대시보드(또는 프론트엔드)에서 '배출 시작'을 누르면 호출되는 HTTP POST 엔드포인트
# 작업 ID 생성: 오늘 날짜(YYYYMMDD)와 고유한 해시값(uuid) 앞 8자리를 조합하여 TASK-20260624-a1b2c3d4 같은 고유 ID를 발행
async def start_dump(payload: TaskStartRequest):
    """
    Tkinter에서 사용자가 시작을 누르면 호출되는 API.
    작업 ID(task_id)를 발행하고 상태를 'INIT'으로 DB에 기록
    """
    date_str = datetime.now().strftime("%Y%m%d")
    unique_id = str(uuid.uuid4())[:8]
    task_id = f"TASK-{date_str}-{unique_id}"

    conn = get_db_connection()
    cursor = conn.cursor()

    try:
        cursor.execute("SELECT * FROM tb_dump_modes WHERE mode_id = ?", (payload.mode_id,))
        mode = cursor.fetchone()
        if not mode:
            raise HTTPException(status_code=400, detail="존재하지 않는 배출 모드 번호입니다.")

        cursor.execute("""
            INSERT INTO tb_dump_history (task_id, mode_id, status)
            VALUES (?, ?, 'INIT')
        """, (task_id, payload.mode_id, ))
        # DB 기록: tb_dump_modes에서 사용자가 선택한 모드가 진짜 있는지 검증한 뒤, tb_dump_history 테이블에 상태를 INIT으로 최초 저장

        conn.commit()

        # ROS2 제어 노드에 "배출 시작" 명령 하달 (상행 파이프라인)
        bridge_manager.publish_command(task_id=task_id, mode_id=payload.mode_id)

        return {
            "result": "SUCCESS",
            "task_id": task_id,
            "mode_name": mode["mode_name"],
            "tilt_angle": mode["tilt_angle"],
            "shake_count": mode["shake_count"],
            "status": "INIT"
        }

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()


# [핵심 API 2] 작업 도중 에러 로그 기록 (/api/error/log)
@app.post("/api/error/log", summary="작업 도중 에러 로그 기록 (REQ-08, REQ-09)")
# 로봇이 움직이다가 파지 불량, 충돌, 비상 정지 등 예외 상황이 터졌을 때 에러 정보를 DB에 남기는 엔드포인트
async def log_error(payload: ErrorLogRequest):
    """
    로봇 구동 중 파지 불량이나 충돌 등 예외 발생 시 에러를 DB에 기록하는 API.
    """
    conn = get_db_connection()
    cursor = conn.cursor()

    try:
        cursor.execute("""
            INSERT INTO tb_error_log (task_id, error_code, error_msg)
            VALUES (?, ?, ?)
        """, (payload.task_id, payload.error_code, payload.error_msg))

        cursor.execute("""
            UPDATE tb_dump_history
            SET status = 'ERROR', end_time = (datetime('now', 'localtime'))
            WHERE task_id = ?
        """, (payload.task_id,))

        # tb_error_log에 에러 코드와 메시지를 인서트하는 동시에, 현재 진행 중이던 작업(tb_dump_history)의 상태를 ERROR로 바꾸고 종료 시간(end_time)을 현재 시간으로 업데이트
        # 실패 시 conn.rollback()으로 안전하게 롤백

        conn.commit()
        return {"result": "SUCCESS", "message": f"에러코드 {payload.error_code}가 성공적으로 기록되었습니다."}

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()

# [핵심 API 3] 배출 작업 전체 이력 조회 (/api/dump/history)
@app.get("/api/dump/history", summary="배출 작업 전체 이력 조회 (최신순)")
async def get_dump_history():
    """
    tb_dump_history 테이블의 전체 작업 이력을 시작 시간 기준 최신순으로 조회.
    Tkinter 대시보드의 이력 테이블/리스트뷰에 표시할 데이터를 제공.
    """
    conn = get_db_connection()
    cursor = conn.cursor()

    try:
        cursor.execute("""
            SELECT
                h.task_id,
                h.mode_id,
                m.mode_name,
                h.start_time,
                h.end_time,
                h.status
            FROM tb_dump_history h
            LEFT JOIN tb_dump_modes m ON h.mode_id = m.mode_id
            ORDER BY h.start_time DESC
        """)
        rows = cursor.fetchall()
        # sqlite3.Row 객체는 JSON으로 바로 직렬화 안 되므로 dict로 변환
        history = [dict(row) for row in rows]

        return {"result": "SUCCESS", "count": len(history), "data": history}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()


# [핵심 API 4] 최근 에러 로그 조회 (/api/error/logs)
@app.get("/api/error/logs", summary="최근 에러 로그 조회")
async def get_error_logs(limit: int = 50):
    """
    tb_error_log 테이블의 최근 에러 기록을 최신순으로 조회.
    limit 쿼리 파라미터로 조회 개수를 제한 (기본 50건).
    """
    conn = get_db_connection()
    cursor = conn.cursor()

    try:
        cursor.execute("""
            SELECT
                error_id,
                task_id,
                error_code,
                error_msg,
                error_time
            FROM tb_error_log
            ORDER BY error_time DESC
            LIMIT ?
        """, (limit,))
        rows = cursor.fetchall()
        logs = [dict(row) for row in rows]

        return {"result": "SUCCESS", "count": len(logs), "data": logs}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()