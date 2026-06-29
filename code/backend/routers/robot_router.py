import uuid
import logging
from datetime import datetime
from fastapi import APIRouter, HTTPException

from database import get_db_connection
from models import (
    TaskStartRequest,
    ErrorLogRequest,
    EmergencyStopRequest,
    ResetRequest,
    MoveJointRequest,
)
from robot_bridge import bridge_manager

logger = logging.getLogger(__name__)

# 주소의 확장성을 위해 prefix를 "/api"로 지정합니다.
router = APIRouter(prefix="/api")

# [핵심 API 1] 배출 작업 시작 및 ID 생성 (/dump/start)
@router.post("/dump/start", summary="배출 작업 시작 및 ID 생성 (REQ-01)")
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
        bridge_manager.publish_command(
            command_type="START", 
            task_id=task_id, 
            mode_id=payload.mode_id
        )
        
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


# [핵심 API 2] 작업 도중 에러 로그 기록 (/error/log)
@router.post("/error/log", summary="작업 도중 에러 로그 기록 (REQ-08, REQ-09)")
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

        inserted_log_id = cursor.lastrowid

        cursor.execute("""
            UPDATE tb_dump_history
            SET status = 'ERROR', end_time = (datetime('now', 'localtime'))
            WHERE task_id = ?
        """, (payload.task_id,))

        # tb_error_log에 에러 코드와 메시지를 인서트하는 동시에, 현재 진행 중이던 작업(tb_dump_history)의 상태를 ERROR로 바꾸고 종료 시간(end_time)을 현재 시간으로 업데이트
        # 실패 시 conn.rollback()으로 안전하게 롤백

        conn.commit()
        return {
            "status": "LOGGED",
            "log_id": inserted_log_id
        }
    
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()

# [핵심 API 3] 배출 작업 전체 이력 조회 (/dump/history)
@router.get("/dump/history", summary="배출 작업 전체 이력 조회 (최신순)")
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


# [핵심 API 4] 최근 에러 로그 조회 (/error/logs)
@router.get("/error/logs", summary="최근 에러 로그 조회")
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
        
# [핵심 API 5] 비상 정지 (/robot/emergency-stop)
@router.post("/robot/emergency-stop", summary="비상 정지 명령 (REQ-EMG)")
# 관리자 GUI의 비상 정지 버튼 클릭 시 호출되는 엔드포인트
async def emergency_stop(payload: EmergencyStopRequest):
    """
    task_id가 있으면 해당 작업의 DB 상태를 EMERGENCY_STOP으로 갱신하고,
    ROS 단에는 즉시 정지 명령을 발행한다.
    """
    logger.info("emergency_stop 호출됨. task_id=%s", payload.task_id)
    print(f"[API] /api/robot/emergency-stop 호출: task_id={payload.task_id}", flush=True)

    conn = None
    try:
        # task_id가 있는 경우에만 DB 상태 갱신 (없으면 전체 비상정지로 간주, DB 변경 없음)
        if payload.task_id:
            conn = get_db_connection()
            cursor = conn.cursor()
            cursor.execute("""
                UPDATE tb_dump_history
                SET status = 'EMERGENCY_STOP', end_time = (datetime('now', 'localtime'))
                WHERE task_id = ?
            """, (payload.task_id,))
            conn.commit()

        # ROS2 제어 노드에 즉시 정지 명령 발행
        bridge_manager.publish_command(
            command_type="EMERGENCY_STOP",
            task_id=payload.task_id,
        )

        return {
            "result": "SUCCESS",
            "message": "비상 정지 명령이 로봇에 전달되었습니다.",
            "task_id": payload.task_id,
        }

    except Exception as e:
        if conn:
            conn.rollback()
        logger.error("emergency_stop 처리 중 에러 발생", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if conn:
            conn.close()


# [핵심 API 6] 시스템 리셋 및 인터록 해제 (/robot/reset)
@router.post("/robot/reset", summary="시스템 리셋 및 인터록 해제")
# 에러 조치 완료 후 로봇을 홈 위치로 복귀시키고 인터록을 해제할 때 호출
async def reset_robot(payload: ResetRequest):
    """
    현재는 DB 변경 없이 ROS 단에만 리셋 명령을 발행한다.
    """
    logger.info("reset_robot 호출됨.")
    print("[API] /api/robot/reset 호출", flush=True)

    try:
        bridge_manager.publish_command(command_type="RESET")

        return {
            "result": "SUCCESS",
            "message": "시스템 리셋 명령이 로봇에 전달되었습니다.",
        }

    except Exception as e:
        logger.error("reset_robot 처리 중 에러 발생", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/robot/retry-pick", summary="수거통 재파지 후 중단 공정 재개")
async def retry_bin_pick(payload: TaskStartRequest):
    """최초 파지 실패 위치에서 수거통을 다시 잡고 기존 공정을 이어가도록 요청한다."""
    try:
        bridge_manager.publish_command(command_type="RETRY_PICK", mode_id=payload.mode_id)
        return {"result": "SUCCESS", "message": "수거통 재파지 명령이 전달되었습니다."}
    except Exception as e:
        logger.error("retry_bin_pick 처리 중 에러 발생", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

# [핵심 API 7] 수동 관절각 제어 (/robot/move-joint)
@router.post("/robot/move-joint", summary="6축 관절각 수동 제어 및 그리퍼 우회 제어 (MoveJ)")
# 관리자 GUI 슬라이더 조작 후 '관절각 일괄 전송(MoveJ)'을 누르면 호출
async def move_joint(payload: MoveJointRequest):
    """
    프론트가 보낸 6축 관절각 데이터를 ROS의 MoveJ 명령으로 발행
    [우회 연동] J6 값이 1.0(OPEN) 또는 2.0(CLOSE)인 경우 그리퍼 명령으로 해석하여 ROS에 하달함.
    """
    joint_data = payload.model_dump()

    # 프론트엔드 우회 규격 파싱
    j6_val = joint_data.get("J6", 0.0)
    base_angle = joint_data.get("J1", 0.0)

    logger.info("move_joint 호출됨. joint_data=%s", joint_data)
    print(f"[API] /api/robot/move-joint 호출: {joint_data}", flush=True)

    try:
        if j6_val == 1.0:
            logger.info("[우회 매핑] 그리퍼 OPEN 명령 감지 (Base Angle: %s)", base_angle)
            print(f"-> [그리퍼 제어] OPEN / 베이스 각도: {base_angle}", flush=True)
            
            bridge_manager.publish_command(
                command_type="HARDWARE_CONTROL",
                payload={"action": "OPEN", "base_angle": base_angle}
            )
            
        elif j6_val == 2.0:
            logger.info("[우회 매핑] 그리퍼 CLOSE 명령 감지 (Base Angle: %s)", base_angle)
            print(f"-> [그리퍼 제어] CLOSE / 베이스 각도: {base_angle}", flush=True)
            
            bridge_manager.publish_command(
                command_type="HARDWARE_CONTROL",
                payload={"action": "CLOSE", "base_angle": base_angle}
            )
            
        else:
            # J6가 1.0이나 2.0이 아니면 일반적인 슬라이더 조작(순수 6축 이동)으로 처리
            bridge_manager.publish_command(
                command_type="MOVE_JOINT",
                payload=joint_data,
            )

        return {
            "result": "SUCCESS",
            "message": "명령이 로봇(ROS2) 브릿지에 성공적으로 전달되었습니다.",
            "joint_data": joint_data,
        }

    except Exception as e:
        logger.error("move_joint 처리 중 에러 발생", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))
