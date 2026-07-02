"""
음식물 수거통 자동 배출/세척 로봇 제어 노드 메인 진입점.

긴 구현은 역할별로 분리했다.
- motion_runtime.py: ROS/DSR 초기화, 상태 발행, 그리퍼, 안전 감시, safe move 헬퍼
- motion_workflow.py: START 공정 순서, RETRY_PICK, RESET 복구 시퀀스
"""

import json
import threading

import DR_init
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from . import motion_runtime as rt
from . import motion_workflow as wf
from .motion_runtime import (
    DUMP_MODE_NORMAL,
    DUMP_MODE_STRONG,
    ROBOT_ID,
    ROBOT_MODEL,
    ErrorCode,
    ProcessState,
    RecoveryStage,
    StatusBus,
)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def handle_emergency_stop(source: str = "/robot/command"):
    """EMERGENCY_STOP 공통 처리. 여러 콜백에서 동시에 들어와도 정지 요청은 직렬화한다."""
    with rt._emergency_command_lock:
        rt.g_node.get_logger().warning(f"비상정지 명령 수신({source}) - 즉시 정지 수행")
        try:
            rt.trigger_safety_stop(
                ErrorCode.ERR_SYSTEM,
                "사용자 비상정지 요청",
                ProcessState.ERROR,
            )
        except Exception as exc:
            rt.g_node.get_logger().error(f"비상정지 처리 중 예외 발생: {exc}")


def handle_emergency_stop_priority(msg: String):
    """같은 /robot/command 토픽에서 EMERGENCY_STOP만 별도 콜백으로 먼저 처리한다."""
    try:
        command = json.loads(msg.data)
    except (json.JSONDecodeError, TypeError):
        return

    cmd_type = command.get("command_type") or command.get("command")
    if cmd_type == "EMERGENCY_STOP":
        handle_emergency_stop("/robot/command priority")


def handle_robot_command(msg: String):
    """FastAPI가 발행한 작업 명령(START, MOVE_JOINT, HARDWARE_CONTROL)을 받아 처리한다."""
    try:
        command = json.loads(msg.data)
    except (json.JSONDecodeError, TypeError) as exc:
        rt.g_node.get_logger().error(f"/robot/command JSON 파싱 실패: {exc}")
        return

    cmd_type = command.get("command_type") or command.get("command")

    if cmd_type == "START":
        task_id = command.get("task_id")
        mode_id = command.get("mode_id")

        if not isinstance(task_id, str) or not task_id.strip():
            rt.g_node.get_logger().error("/robot/command에 유효한 task_id가 없습니다.")
            return
        if isinstance(mode_id, bool) or not isinstance(mode_id, int):
            rt.g_node.get_logger().error(f"task_id={task_id}: mode_id는 정수여야 합니다: {mode_id!r}")
            return
        if mode_id not in (DUMP_MODE_NORMAL, DUMP_MODE_STRONG):
            rt.g_node.get_logger().error(f"task_id={task_id}: 지원하지 않는 mode_id={mode_id}")
            return
        if not wf._process_lock.acquire(blocking=False):
            rt.g_node.get_logger().warning(f"이미 공정이 진행 중입니다. task_id={task_id} 명령 무시.")
            return

        rt._emergency_stop_requested.clear()
        rt.g_node.get_logger().info(f"공정 시작 명령 수신: task_id={task_id}, mode_id={mode_id}")

        try:
            process_thread = threading.Thread(
                target=wf._run_process_worker,
                args=(task_id, mode_id),
                daemon=True,
                name=f"process-{task_id}",
            )
            process_thread.start()
        except Exception:
            wf._process_lock.release()
            raise
        return

    if cmd_type == "RETRY_PICK":
        mode_id = command.get("mode_id")
        if isinstance(mode_id, bool) or mode_id not in (DUMP_MODE_NORMAL, DUMP_MODE_STRONG):
            rt.g_node.get_logger().error(f"RETRY_PICK mode_id 오류: {mode_id!r}")
            return
        if rt.get_recovery_stage() != RecoveryStage.BIN_PICK_FAILED:
            rt.g_node.get_logger().warning(f"RETRY_PICK 무시 - 현재 단계={rt.get_recovery_stage().value}")
            return
        if not wf._process_lock.acquire(blocking=False):
            rt.g_node.get_logger().warning("이미 공정이 진행 중입니다. RETRY_PICK 무시.")
            return
        rt._emergency_stop_requested.clear()
        try:
            threading.Thread(target=wf._run_retry_pick_worker, args=(mode_id,), daemon=True, name="retry-bin-pick").start()
        except Exception:
            wf._process_lock.release()
            raise
        return

    if cmd_type == "HARDWARE_CONTROL":
        payload = command.get("payload", {})
        action = payload.get("action")
        base_angle = payload.get("base_angle", 0.0)

        rt.g_node.get_logger().info(f"[수동 하드웨어 제어 수신] action={action}, base_angle={base_angle}")

        if action == "OPEN":
            rt.g_node.get_logger().info("-> 로봇 그리퍼 OPEN 구동 수행")
            rt.gripper_open()
        elif action == "CLOSE":
            rt.g_node.get_logger().info("-> 로봇 그리퍼 CLOSE 구동 수행")
            rt.gripper_close()
        else:
            rt.g_node.get_logger().warning(f"알 수 없는 하드웨어 액션: {action}")
        return

    if cmd_type == "MOVE_JOINT":
        payload = command.get("payload", {})
        rt.g_node.get_logger().info(f"[수동 관절 이동 수신] joint_data={payload}")
        try:
            joint_angles = [
                float(payload.get("J1", 0.0)),
                float(payload.get("J2", 0.0)),
                float(payload.get("J3", 90.0)),
                float(payload.get("J4", 0.0)),
                float(payload.get("J5", 90.0)),
                float(payload.get("J6", 0.0)),
            ]
            rt.g_node.get_logger().info(f"-> 관절각 일괄 이동(MoveJ) 수행: {joint_angles}")
            rt.safe_movej(joint_angles)
            rt.g_node.get_logger().info("-> MoveJ 완료")
        except Exception as exc:
            rt.g_node.get_logger().error(f"MOVE_JOINT 구동 중 예외 발생: {exc}")
        return

    if cmd_type == "EMERGENCY_STOP":
        handle_emergency_stop("/robot/command")
        return

    if cmd_type == "RESET":
        if wf._process_lock.locked():
            rt.g_node.get_logger().warning("공정 진행 중 - RESET 명령 무시됨")
            return

        rt.g_node.get_logger().info("RESET 명령 수신 - 마지막 체크포인트 기준 복구 수행")
        try:
            if rt.block_reset_if_controller_safety_stop():
                return
            wf.recover_after_emergency_stop()
            rt.g_node.get_logger().info("RESET 완료 - IDLE 상태로 복귀")
        except Exception as exc:
            rt.g_node.get_logger().error(f"RESET 처리 중 예외 발생: {exc}")
            rt.status.set_state(ProcessState.ERROR, str(exc))
        return

    rt.g_node.get_logger().warning(f"지원하지 않는 command_type 무시: {cmd_type!r}")


def main(args=None):
    """ROS 노드, DSR API, 구독 콜백, executor를 초기화하고 명령 수신을 시작한다."""
    rclpy.init(args=args)

    rt.dsr_node = rclpy.create_node("food_waste_dump_robot_dsr", namespace=ROBOT_ID)
    DR_init.__dsr__node = rt.dsr_node

    rt.stop_node = rclpy.create_node("food_waste_dump_robot_stop", namespace=ROBOT_ID)

    node = rclpy.create_node("food_waste_dump_robot", namespace=ROBOT_ID)
    rt.g_node = node

    node.declare_parameter("dump_mode", DUMP_MODE_NORMAL)
    node.declare_parameter("autostart", False)

    rt.status = StatusBus(node)
    rt.init_robot_api()
    rt.init_gripper_api()
    wf.sync_runtime_refs()

    process_callback_group = MutuallyExclusiveCallbackGroup()
    emergency_callback_group = ReentrantCallbackGroup()

    node.create_subscription(
        String, "/robot/command", handle_emergency_stop_priority, 10,
        callback_group=emergency_callback_group,
    )
    node.create_subscription(
        String, "/robot/command", handle_robot_command, 10,
        callback_group=process_callback_group,
    )

    rt.status.set_state(ProcessState.IDLE, "작업 대기")

    if node.get_parameter("autostart").value:
        mode = int(node.get_parameter("dump_mode").value)
        wf.run_process(mode)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        try:
            rt.gripper_open()
        except Exception:
            pass
        node.destroy_node()
        rt.dsr_node.destroy_node()
        rt.stop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
