"""
음식물 수거통 자동 배출/세척 로봇 제어 노드
- gear_insert.py 구조를 기반으로 작성
- B-4 요구사항 명세서 REQ-01 ~ REQ-09 반영

실행 예시:
  ros2 run <pkg> food_waste_dump_robot --ros-args -p operation_mode:=virtual -p dump_mode:=1 -p autostart:=true

주의:
  아래 좌표는 반드시 실제 지그/수거통/세척 위치에 맞게 교시 후 수정해야 한다.
"""

import math
import time
from enum import Enum

import rclpy
import DR_init
from std_msgs.msg import String, Bool, Int32
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveStop
from onrobot_rg_msgs.srv import SetCommand

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==============================================================================
# [운전 파라미터]
# ==============================================================================
VELOCITYX, ACCX = 30, 30
VELOCITYJ, ACCJ = 30, 30
SLOW_VELX, SLOW_ACCX = 15, 20

# 충돌/이탈 감지 기준
COLLISION_FORCE_N = 35.0      # F/T 센서 외력 임계값[N]
GRIPPER_INPUT_IDX = 1         # 실제 파지 확인용 Tool DI 번호. 현장 배선에 맞게 수정
VALVE_DO_IDX = 1              # 수도 밸브 제어용 Digital Output 번호. 현장 배선에 맞게 수정

# 배출 모드: 1=일반 배출, 2=강하게 털기
DUMP_MODE_NORMAL = 1
DUMP_MODE_STRONG = 2

# DRL/DSR 모듈은 DR_init.__dsr__node 등록 후 import해야 함
_ds = None
posx = None
posj = None
g_node = None
gripper_client = None


class ProcessState(str, Enum):
    IDLE = "IDLE"
    INIT = "INIT"
    READY = "READY"
    MOVING = "MOVING"
    DUMPING = "DUMPING"
    WASHING = "WASHING"
    COMPLETE = "COMPLETE"
    PAUSED = "PAUSED"
    COLLISION = "COLLISION"
    ERROR = "ERROR"


class ErrorCode(str, Enum):
    ERR_PICK = "ERR_PICK"
    ERR_DROP = "ERR_DROP"
    ERR_COLLISION = "ERR_COLLISION"
    ERR_SYSTEM = "ERR_SYSTEM"


# ==============================================================================
# [좌표 정의]
# ==============================================================================
def coordinates():
    """현장 교시 후 반드시 수정할 좌표 묶음."""
    return {
        # 초기 대기 위치
        "home": posj(9.46, 1.33, 103.26, -0.75, 74.64, 2.23),

        # 수거통 픽업 위치
        "bin_approach": posx(423.65, -3.58, 330.00, 97.76, -178.69, 95.48),
        "bin_pick":     posx(424.74, -4.82, 266.78, 80.35, -178.76, 78.04),

        # 배출 위치. dump_base에서 자세축을 변경해 통을 기울인다.
        "dump_approach": posx(277.21, 147.71, 330.00, 103.54, -178.79, 101.07),
        "dump_base":     posx(277.77, 147.23, 300.00, 89.86, -178.71, 87.42),

        # 세척 위치
        "wash_approach": posx(360.00, 220.00, 330.00, 90.00, -180.00, 90.00),
        "wash_base":     posx(360.00, 220.00, 285.00, 90.00, -180.00, 90.00),
    }


# ==============================================================================
# [초기화]
# ==============================================================================
def init_robot_api():
    global _ds, posx, posj

    import DSR_ROBOT2 as dsr_module
    from DR_common2 import posx as posx_class, posj as posj_class

    _ds = dsr_module
    posx = posx_class
    posj = posj_class

    required_services = [
        _ds._ros2_set_current_tool,
        _ds._ros2_set_current_tcp,
        _ds._ros2_set_singularity_handling,
        _ds._ros2_movej,
        _ds._ros2_movel,
        _ds._ros2_check_motion,
        _ds._ros2_get_tool_force,
        _ds._ros2_get_current_posx,
        _ds._ros2_get_current_posj,
        _ds._ros2_get_tool_digital_input,
        _ds._ros2_set_digital_output,
    ]

    g_node.get_logger().info("Waiting for DSR controller services...")
    for client in required_services:
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"DSR service is not available: {client.srv_name}")
    g_node.get_logger().info("DSR controller services are ready")

    operation_mode = g_node.declare_parameter(
        "operation_mode", "virtual"
    ).get_parameter_value().string_value

    if operation_mode == "real":
        if _ds.set_tool("Tool Weight_2FG") != 0:
            raise RuntimeError("Tool Weight_2FG is not registered on the real robot")
        if _ds.set_tcp("2FG_TCP") != 0:
            raise RuntimeError("2FG_TCP is not registered on the real robot")
    elif operation_mode == "virtual":
        g_node.get_logger().info("Virtual mode: skip real Tool/TCP registration")
    else:
        raise RuntimeError("operation_mode must be 'virtual' or 'real'")

    if _ds.set_singularity_handling(_ds.DR_AVOID) != 0:
        raise RuntimeError("Failed to set singularity handling")


def init_gripper_api():
    global gripper_client
    gripper_client = g_node.create_client(SetCommand, "/onrobot/sendCommand")
    if not gripper_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("RG2 service is not available: /onrobot/sendCommand")
    g_node.get_logger().info("RG2 service is ready")


# ==============================================================================
# [ROS 상태 출력]
# ==============================================================================
class StatusBus:
    def __init__(self, node):
        self.node = node
        self.state_pub = node.create_publisher(String, "/robot/process_state", 10)
        self.motion_pub = node.create_publisher(String, "/robot/motion_status", 10)
        self.safety_pub = node.create_publisher(String, "/robot/safety_event", 10)
        self.gripper_pub = node.create_publisher(Bool, "/gripper/status", 10)
        self.mode_pub = node.create_publisher(Int32, "/hmi/mode_cmd", 10)
        self.state = ProcessState.IDLE

    def set_state(self, state: ProcessState, msg: str = ""):
        self.state = state
        text = state.value if not msg else f"{state.value}:{msg}"
        self.state_pub.publish(String(data=text))
        self.motion_pub.publish(String(data=text))
        self.node.get_logger().info(text)

    def publish_safety(self, code: ErrorCode, msg: str):
        text = f"{code.value}:{msg}"
        self.safety_pub.publish(String(data=text))
        self.node.get_logger().error(text)

    def publish_gripper(self, grasped: bool):
        self.gripper_pub.publish(Bool(data=grasped))

    def publish_mode(self, mode: int):
        self.mode_pub.publish(Int32(data=mode))


status = None


# ==============================================================================
# [DR에 없거나 불안정한 함수 보완]
# ==============================================================================
def stop(mode=None):
    """DRL stop()과 동일한 목적의 ROS2 MoveStop 래퍼."""
    stop_mode = _ds.DR_QSTOP if mode is None else mode
    client = g_node.create_client(
        MoveStop,
        f"/{ROBOT_ID}/dsr_controller2/motion/move_stop",
    )
    if not client.wait_for_service(timeout_sec=1.0):
        return -1

    req = MoveStop.Request()
    req.stop_mode = int(stop_mode)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(g_node, future, timeout_sec=2.0)
    result = future.result() if future.done() else None
    return 0 if result and result.success else -1


def set_external_force_reset(mode=0, offset=None):
    """DSR_ROBOT2에 함수가 있을 때만 호출하는 호환 래퍼."""
    if hasattr(_ds, "set_external_force_reset"):
        if offset is None:
            return _ds.set_external_force_reset(mode)
        return _ds.set_external_force_reset(mode, offset)
    g_node.get_logger().warn("set_external_force_reset() is not available in this DSR_ROBOT2.py; skipped")
    return 0


# ==============================================================================
# [그리퍼 / 센서 / 밸브]
# ==============================================================================
def send_gripper_command(command: str):
    req = SetCommand.Request()
    req.command = command
    future = gripper_client.call_async(req)
    rclpy.spin_until_future_complete(g_node, future, timeout_sec=10.0)

    if not future.done():
        raise RuntimeError(f"RG2 command timed out: {command}")
    result = future.result()
    if result is None or not result.success:
        message = result.message if result else "no response"
        raise RuntimeError(f"RG2 command failed ({command}): {message}")


def gripper_open():
    send_gripper_command("o")
    _ds.wait(0.3)
    status.publish_gripper(False)


def gripper_close():
    send_gripper_command("c")
    _ds.wait(0.5)


def is_grasped() -> bool:
    """통 파지 확인. virtual 모드에서는 파지 성공으로 간주."""
    operation_mode = g_node.get_parameter("operation_mode").value
    if operation_mode == "virtual":
        status.publish_gripper(True)
        return True

    try:
        val = _ds.get_tool_digital_input(GRIPPER_INPUT_IDX)
        grasped = bool(val)
        status.publish_gripper(grasped)
        return grasped
    except Exception as exc:
        status.publish_safety(ErrorCode.ERR_PICK, f"gripper sensor read failed: {exc}")
        return False


def valve_open():
    _ds.set_digital_output(VALVE_DO_IDX, _ds.ON)


def valve_close():
    _ds.set_digital_output(VALVE_DO_IDX, _ds.OFF)


# ==============================================================================
# [안전 감시]
# ==============================================================================
def current_force_norm() -> float:
    force = _ds.get_tool_force(_ds.DR_BASE)
    return math.sqrt(force[0] ** 2 + force[1] ** 2 + force[2] ** 2)


def raise_safety_stop(code: ErrorCode, msg: str):
    stop(_ds.DR_QSTOP)
    status.set_state(ProcessState.COLLISION if code == ErrorCode.ERR_COLLISION else ProcessState.ERROR)
    status.publish_safety(code, msg)
    try:
        valve_close()
    except Exception:
        pass
    raise RuntimeError(f"{code.value}: {msg}")


def safety_watch(require_grasp: bool = False):
    if current_force_norm() > COLLISION_FORCE_N:
        raise_safety_stop(ErrorCode.ERR_COLLISION, "정격 토크 초과 충돌 감지")
    if require_grasp and not is_grasped():
        raise_safety_stop(ErrorCode.ERR_DROP, "이동 또는 털기 중 수거통 이탈 감지")


def safe_movej(target, vel=VELOCITYJ, acc=ACCJ, require_grasp=False):
    _ds.amovej(target, vel=vel, acc=acc)
    while _ds.check_motion():
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.05)


def safe_movel(target, vel=VELOCITYX, acc=ACCX, require_grasp=False):
    _ds.amovel(target, vel=vel, acc=acc)
    while _ds.check_motion():
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.05)


def safe_wait(seconds: float, require_grasp=False):
    start = time.time()
    while time.time() - start < seconds:
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.05)


# ==============================================================================
# [공정 단계]
# ==============================================================================
def check_system_ready():
    status.set_state(ProcessState.INIT, "시스템 및 센서 체크 중")
    coords = coordinates()

    # IDLE 위치 검증: 여기서는 직접 home으로 이동해 안전 위치를 확보한다.
    safe_movej(coords["home"])
    gripper_open()
    status.set_state(ProcessState.READY, "초기 대기 위치 및 그리퍼 확인 완료")
    return True


def pick_bin():
    status.set_state(ProcessState.MOVING, "수거통 위치 이동 및 파지")
    coords = coordinates()

    safe_movej(coords["home"])
    safe_movel(coords["bin_approach"])
    safe_movel(coords["bin_pick"], vel=SLOW_VELX, acc=SLOW_ACCX)

    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "수거통 미감지 또는 파지 불량")
        gripper_open()
        raise RuntimeError("수거통의 위치를 확인해 주세요")

    safe_movel(coords["bin_approach"], require_grasp=True)


def make_tilt_pose(base, tilt_deg: float, shake_deg: float = 0.0):
    """통 배출용 자세 생성. B축을 tilt, C축을 shake로 사용한다."""
    p = posx(base)
    p[4] = base[4] + tilt_deg
    p[5] = base[5] + shake_deg
    return p


def run_dump_motion(mode: int):
    status.set_state(ProcessState.DUMPING, f"mode={mode}")
    status.publish_mode(mode)
    coords = coordinates()

    if mode == DUMP_MODE_NORMAL:
        tilt_deg = 45.0
        shake_count = 3
        shake_amp = 5.0
    elif mode == DUMP_MODE_STRONG:
        tilt_deg = 60.0
        shake_count = 6
        shake_amp = 10.0
    else:
        raise ValueError("dump mode must be 1(normal) or 2(strong)")

    safe_movel(coords["dump_approach"], require_grasp=True)
    safe_movel(coords["dump_base"], require_grasp=True)

    tilted = make_tilt_pose(coords["dump_base"], tilt_deg)
    safe_movel(tilted, vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)

    for idx in range(shake_count):
        left = make_tilt_pose(coords["dump_base"], tilt_deg, -shake_amp)
        right = make_tilt_pose(coords["dump_base"], tilt_deg, shake_amp)
        status.set_state(ProcessState.DUMPING, f"shaking {idx + 1}/{shake_count}")
        safe_movel(left, vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)
        safe_movel(right, vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)

    safe_movel(coords["dump_base"], vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)
    safe_movel(coords["dump_approach"], require_grasp=True)


def execute_wash():
    status.set_state(ProcessState.WASHING, "세척 위치 이동")
    coords = coordinates()

    safe_movel(coords["wash_approach"], require_grasp=True)
    safe_movel(coords["wash_base"], require_grasp=True)

    status.set_state(ProcessState.WASHING, "급수 2초")
    valve_open()
    safe_wait(2.0, require_grasp=True)
    valve_close()

    status.set_state(ProcessState.WASHING, "가벼운 세척 흔들기")
    for idx in range(4):
        p1 = posx(coords["wash_base"])
        p2 = posx(coords["wash_base"])
        p1[5] -= 7.0
        p2[5] += 7.0
        safe_movel(p1, vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)
        safe_movel(p2, vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)

    # 오수 배출: 세척 위치에서 살짝 기울여 배수
    drain = make_tilt_pose(coords["wash_base"], 35.0)
    safe_movel(drain, vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)
    safe_wait(1.0, require_grasp=True)
    safe_movel(coords["wash_approach"], require_grasp=True)


def return_home_and_complete():
    coords = coordinates()
    status.set_state(ProcessState.MOVING, "초기 위치 복귀")
    safe_movej(coords["home"], require_grasp=True)
    gripper_open()
    status.set_state(ProcessState.COMPLETE, "배출 및 세척 완료")


def run_process(mode: int):
    try:
        check_system_ready()
        pick_bin()
        run_dump_motion(mode)
        execute_wash()
        return_home_and_complete()
        return True, "배출 및 세척 완료"
    except Exception as exc:
        status.set_state(ProcessState.ERROR, str(exc))
        try:
            stop(_ds.DR_QSTOP)
        except Exception:
            pass
        return False, str(exc)


# ==============================================================================
# [Service entry]
# ==============================================================================
def handle_dump_start(request, response):
    mode = int(g_node.get_parameter("dump_mode").value)
    ok, msg = run_process(mode)
    response.success = ok
    response.message = msg
    return response


# ==============================================================================
# [메인]
# ==============================================================================
def main(args=None):
    global g_node, status

    rclpy.init(args=args)
    node = rclpy.create_node("food_waste_dump_robot", namespace=ROBOT_ID)
    g_node = node
    DR_init.__dsr__node = node

    node.declare_parameter("dump_mode", DUMP_MODE_NORMAL)
    node.declare_parameter("autostart", False)

    status = StatusBus(node)

    init_robot_api()
    init_gripper_api()

    # 명세서의 /robot/dump_cmd 역할: HMI/FastAPI에서 Trigger 호출 시 전체 공정 시작
    node.create_service(Trigger, "/robot/dump_cmd", handle_dump_start)

    status.set_state(ProcessState.IDLE, "작업 대기")

    if node.get_parameter("autostart").value:
        mode = int(node.get_parameter("dump_mode").value)
        run_process(mode)

    try:
        rclpy.spin(node)
    finally:
        try:
            valve_close()
            gripper_open()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()