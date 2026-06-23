from fastapi import FastAPI, HTTPException
from datetime import datetime
import uuid

# 우리가 방금 만든 모듈 임포트
from database import init_db, get_db_connection
from models import TaskStartRequest, ErrorLogRequest

app = FastAPI(
    title="AutoDumpBot Backend API",
    description="음식물 수거 로봇 자동화 시스템을 위한 백엔드 제어 및 이력 관리 API",
    version="1.0.0"
)

# 서버 가동 시 DB 초기화 실행
@app.on_event("startup")
def startup_event():
    init_db()

@app.get("/")
def read_root():
    return {"message": "AutoDumpBot Backend Server is Running!"}

@app.post("/api/dump/start", summary="배출 작업 시작 및 ID 생성 (REQ-01)")
async def start_dump(payload: TaskStartRequest):
    """
    Tkinter에서 사용자가 시작을 누르면 호출되는 API.
    작업 ID(task_id)를 발행하고 상태를 'INIT'으로 DB에 기록합니다.
    """
    # 1. 고유한 작업 ID 생성 (예: TASK-20260622-uuid앞자리)
    date_str = datetime.now().strftime("%Y%m%d")
    unique_id = str(uuid.uuid4())[:8]
    task_id = f"TASK-{date_str}-{unique_id}"
    
    conn = get_db_connection()
    cursor = conn.cursor()
    
    try:
        # 2. 존재하는 모드인지 검증
        cursor.execute("SELECT * FROM tb_dump_modes WHERE mode_id = ?", (payload.mode_id,))
        mode = cursor.fetchone()
        if not mode:
            raise HTTPException(status_code=400, detail="존재하지 않는 배출 모드 번호입니다.")
        
        # 3. 배출 이력 테이블에 첫 상태 'INIT'으로 삽입
        cursor.execute("""
            INSERT INTO tb_dump_history (task_id, mode_id, status)
            VALUES (?, ?, 'INIT')
        """, (task_id, payload.mode_id, ))
        
        conn.commit()
        
        # 💡 [나중에 뚫을 곳]: 여기에 ROS2 노드로 구동 신호를 전송하는 코드가 들어갈 자리입니다.
        
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

@app.post("/api/error/log", summary="작업 도중 에러 로그 기록 (REQ-08, REQ-09)")
async def log_error(payload: ErrorLogRequest):
    """
    로봇 구동 중 파지 불량이나 충돌 등 예외 발생 시 에러를 DB에 기록하는 API.
    """
    conn = get_db_connection()
    cursor = conn.cursor()
    
    try:
        # 1. 에러 테이블에 인서트
        cursor.execute("""
            INSERT INTO tb_error_log (task_id, error_code, error_msg)
            VALUES (?, ?, ?)
        """, (payload.task_id, payload.error_code, payload.error_msg))
        
        # 2. 원본 작업의 상태도 'ERROR'로 변경
        cursor.execute("""
            UPDATE tb_dump_history 
            SET status = 'ERROR', end_time = (datetime('now', 'localtime'))
            WHERE task_id = ?
        """, (payload.task_id,))
        
        conn.commit()
        return {"result": "SUCCESS", "message": f"에러코드 {payload.error_code}가 성공적으로 기록되었습니다."}
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()