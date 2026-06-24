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

# 새로 추가: 털기 왕복 동작은 일반 이송보다 빠르게 수행한다.
SHAKE_VEL, SHAKE_ACC = 50, 50
SHAKE_REPEAT_COUNT = 3

# 충돌/이탈 감지 기준
COLLISION_FORCE_N = 30.0      # F/T 센서 외력 임계값[N]
GRIPPER_INPUT_IDX = 1         # 실제 파지 확인용 Tool DI 번호. 현장 배선에 맞게 수정

# 외력제어 파라미터
FORCE_TH = 20.0  # place시 외력감지 Threshold
DESIRED_FORCE_X = 25.0     # 세척 위치 안착 방향 힘[N] - 베이스 좌표계 +X 방향
DESIRED_FORCE_Z = 10.0     # 세척 위치 Z방향 힘[N] - 실기 테스트 후 조정
COMPLIANCE_X = 300         # X 순응 강성 - 낮을수록 +X 방향 접촉면을 부드럽게 따라감
COMPLIANCE_Y = 3000        # Y 순응 강성 - Y방향의 불필요한 움직임을 억제
COMPLIANCE_Z = 3000        # Z 순응 강성 - 낮을수록 부드럽게 눌림
FORCE_CONTROL_TIME = 10.0   # 목표 외력을 유지하며 안착시킬 시간[s]

# 밸브 - 굳이 필요는 없을 듯
VALVE_DO_IDX = 1              # 수도 밸브 제어용 Digital Output 번호. 현장 배선에 맞게 수정

# 배출 모드: 1=일반 배출, 2=강하게 털기
DUMP_MODE_NORMAL = 1
DUMP_MODE_STRONG = 2

# DRL/DSR 모듈은 DR_init.__dsr__node 등록 후 import해야 함
# 함수 내 import한 모듈을 담기 위한 변수 선언
_ds = None
posx = None
posj = None
g_node = None
gripper_client = None

# 공정의 상태를 정해진 값으로 관리하기 위한 클래스
# Enum은 클래스 기본 문법 작성 없이도 이름=값 쌍으로 묶어서 표현 가능하게 함
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

# 오류코드를 정해진 값으로 관리하기 위해 넣은 클래스
class ErrorCode(str, Enum):
    ERR_PICK = "ERR_PICK"
    ERR_DROP = "ERR_DROP"
    ERR_COLLISION = "ERR_COLLISION"
    ERR_SYSTEM = "ERR_SYSTEM"


# ==============================================================================
# [좌표 정의]
# ==============================================================================
def coordinates():
    """실기기에서 측정한 좌표 묶음."""
    return {
        # 수거통 픽업 위치
        "bin_approach": posx(457.76, -244.42, 59.15, 92.33, 86.93, -86.84),
        "bin_pick": posx(466.08, -202.23, 66.98, 95.02, 88.55, -85.96),
        "bin_pick_top": posx(466.04, -196.34, 199.37, 92.36, 90.60, -89.12),

        # 음식물 배출 및 털기 위치
        "dump_approach": posx(466.08, -4.19, 199.37, 90.27, 92.28, -87.35),
        "dump_tilt": posj(-15.97, 35.06, 102.51, 76.35, 99.09, -80.00),
        "shake_weak_x": posx(453.60, -0.61, 238.28, 90.26, 92.19, 54.22),
        "shake_weak_j": posj(-15.97, 29.29, 103.47, 77.22, 100.26, -80.00),
        "shake_strong_j": posj(-15.93, 28.53, 103.30, 77.89, 104.24, -80.00),
        "shake_strong_x": posx(466.12, -4.19, 194.30, 90.26, 92.28, 58.99),

        # 배출 위치에서 세척 위치로 이동할 때의 경유점
        "way_point": posx(311.45, -272.72, 129.42, 57.31, 88.51, -85.82),

        # 세척 위치
        "wash_approach": posx(454.14, 17.88, 112.29, 0.72, 93.48, -87.55),
        "wash_place": posx(482.61, 17.07, 112.91, 0.82, 93.14, -87.60),
        "wash_pick": posx(484.18, 8.30, 91.97, 2.53, 93.50, -88.50),
        "wash_close": posx(685.11, -87.56, 361.56, 84.15, 134.72, -92.30),
        "wash_open": posj(-14.96, 55.93, 21.05, 46.65, 105.56, -99.25),

        # 세척수 배출 위치
        "water_out_approach": posx(557.53, -170.89, 145.02, 90.10, 94.08, -91.72),
        "water_out_tilt": posj(-28.34, 54.57, 71.90, 68.79, 109.80, -90.00),
        "water_out_shake": posx(554.47, -173.86, 182.70, 89.97, 91.09, 27.51),

        # 초기 대기 및 종료 위치
        "home": posj(-0.45, 0.66, 88.77, -0.77, 87.93, -234.35),
    }


# ==============================================================================
# [초기화]
# ==============================================================================
def init_robot_api():
    # 정의했던 변수를 전역 변수 선언
    global _ds, posx, posj

    import DSR_ROBOT2 as dsr_module
    from DR_common2 import posx as posx_class, posj as posj_class

    # 
    _ds = dsr_module
    posx = posx_class
    posj = posj_class

    # 실제 필요한 서비스 리스트
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
        _ds._ros2_task_compliance_ctrl,
        _ds._ros2_set_desired_force,
        _ds._ros2_release_force,
        _ds._ros2_release_compliance_ctrl,
    ]

    g_node.get_logger().info("Waiting for DSR controller services...")
    # 서비스 리스트 각각이 생성되었는지 확인
    for client in required_services:
        if not client.wait_for_service(timeout_sec=30.0):
            # 없으면 에러
            raise RuntimeError(f"DSR service is not available: {client.srv_name}")
    # 다 체크 완료되면 준비완 표시
    g_node.get_logger().info("DSR controller services are ready")

    # 노드에 대한 파라미터 선언
    operation_mode = g_node.declare_parameter(
        "operation_mode", "virtual"
    ).get_parameter_value().string_value
    # 실제모드면 set_tool, set_tcp 값 확인
    if operation_mode == "real":
        # set_tool과 set_tcp는 리턴값이 0이어야 성공으로 인식
        if _ds.set_tool("Tool Weight_2FG") != 0:
            raise RuntimeError("Tool Weight_2FG is not registered on the real robot")
        if _ds.set_tcp("2FG_TCP") != 0:
            raise RuntimeError("2FG_TCP is not registered on the real robot")
    
    # 가상모드면 set_tool, set_tcp 값 확인 넘어감
    elif operation_mode == "virtual":
        g_node.get_logger().info("Virtual mode: skip real Tool/TCP registration")
    else:
        raise RuntimeError("operation_mode must be 'virtual' or 'real'")

    if _ds.set_singularity_handling(_ds.DR_AVOID) != 0:
        raise RuntimeError("Failed to set singularity handling")

# 그리퍼 초기화 함수
def init_gripper_api():
    global gripper_client
    gripper_client = g_node.create_client(SetCommand, "/onrobot/sendCommand")
    if not gripper_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("RG2 service is not available: /onrobot/sendCommand")
    g_node.get_logger().info("RG2 service is ready")


# ==============================================================================
# [ROS 상태 출력] - 로봇의 상태 정보 토픽으로 전달(상태 알림 담당 클래스)
# ==============================================================================
class StatusBus:
    def __init__(self, node):
        self.node = node  # 전달받은 노드를 클래스 내에서 사용하기 위한 변수 할당
        self.state_pub = node.create_publisher(String, "/robot/process_state", 10)
        self.motion_pub = node.create_publisher(String, "/robot/motion_status", 10)
        self.safety_pub = node.create_publisher(String, "/robot/safety_event", 10)
        self.gripper_pub = node.create_publisher(Bool, "/gripper/status", 10)
        self.mode_pub = node.create_publisher(Int32, "/hmi/mode_cmd", 10)
        self.state = ProcessState.IDLE

    def set_state(self, state: ProcessState, msg: str = ""):
        self.state = state
        # msg값이 비어있지 않으면 값을 내보내고 아니면 값과 msg값을 같이 내보냄 
        text = state.value if not msg else f"{state.value}:{msg}"
        
        self.state_pub.publish(String(data=text)) # state 발행
        self.motion_pub.publish(String(data=text)) # motion 발행
        self.node.get_logger().info(text) # text 출력

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

# 파지 확인 함수
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


def apply_wash_place_force():
    """세척 위치에서 베이스 좌표계 +X와 Z방향 외력으로 수거통을 안착시킨다."""
    compliance_active = False
    force_active = False

    try:
        # 순응 제어의 강성 순서는 [X, Y, Z, Rx, Ry, Rz]이다.
        if _ds.task_compliance_ctrl([
            COMPLIANCE_X, COMPLIANCE_Y, COMPLIANCE_Z,
            100, 100, 100,
        ]) != 0:
            raise RuntimeError("세척 위치 순응 제어 시작 실패")
        compliance_active = True

        # 현재 기본 기준 좌표계인 DR_BASE에서 +X 및 Z방향 힘 제어를 활성화한다.
        if _ds.set_desired_force(
            [DESIRED_FORCE_X, 0, DESIRED_FORCE_Z, 0, 0, 0],
            [1, 0, 1, 0, 0, 0],
        ) != 0:
            raise RuntimeError("세척 위치 외력 제어 시작 실패")
        force_active = True

        safe_wait(FORCE_CONTROL_TIME, require_grasp=True)
    finally:
        # 중간에 안전 정지나 예외가 발생해도 외력/순응 제어는 반드시 해제한다.
        if force_active and _ds.release_force(0.2) != 0:
            g_node.get_logger().error("세척 위치 외력 제어 해제 실패")
        if compliance_active:
            _ds.wait(0.2)
            if _ds.release_compliance_ctrl() != 0:
                g_node.get_logger().error("세척 위치 순응 제어 해제 실패")


# ==============================================================================
# [공정 단계]
# ==============================================================================

# 로봇 연결시 시스템 체크
def check_system_ready():
    status.set_state(ProcessState.INIT, "시스템 및 센서 체크 중")
    # 좌표 불러오기
    coords = coordinates()

    # IDLE 위치 검증: 여기서는 직접 home으로 이동해 안전 위치를 확보한다.
    #safe_movel(coords["bin_pick_top"])
    safe_movej(coords["home"])
    gripper_open()
    status.set_state(ProcessState.READY, "초기 대기 위치 및 그리퍼 확인 완료")
    return True

# bin pick위치로 이동
def pick_bin():
    status.set_state(ProcessState.MOVING, "수거통 위치 이동 및 파지")
    coords = coordinates()

    # bin approach 위치로 이동
    safe_movel(coords["bin_pick_top"])
    safe_movel(coords["bin_approach"])
    # bin_pick 위치로 이동
    safe_movel(coords["bin_pick"], vel=SLOW_VELX, acc=SLOW_ACCX)

    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "수거통 미감지 또는 파지 불량")
        gripper_open()
        raise RuntimeError("수거통의 위치를 확인해 주세요")

    safe_movel(coords["bin_pick_top"], require_grasp=True)

# 음식물 쓰레기 폐기통에 버리는 모션
def run_dump_motion(mode: int):
    status.set_state(ProcessState.DUMPING, f"mode={mode}")
    status.publish_mode(mode)
    coords = coordinates()

    if mode == DUMP_MODE_NORMAL:
        shake_x = coords["shake_weak_x"]
    elif mode == DUMP_MODE_STRONG:
        shake_x = coords["shake_strong_x"]
    else:
        raise ValueError("dump mode must be 1(normal) or 2(strong)")

    safe_movel(coords["dump_approach"], require_grasp=True)
    safe_movej(coords["dump_tilt"], vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)

    # 새로 추가: 모드에 맞는 털기 좌표와 dump_tilt를 빠르게 왕복한다.
    for idx in range(SHAKE_REPEAT_COUNT):
        status.set_state(
            ProcessState.DUMPING,
            f"shaking {idx + 1}/{SHAKE_REPEAT_COUNT}",
        )
        safe_movel(shake_x, vel=SHAKE_VEL, acc=SHAKE_ACC, require_grasp=True)
        safe_movej(
            coords["dump_tilt"],
            vel=SHAKE_VEL,
            acc=SHAKE_ACC,
            require_grasp=True,
        )

    safe_movel(coords["dump_approach"], require_grasp=True)

# 세척
def execute_wash():
    status.set_state(ProcessState.WASHING, "세척 위치 이동")
    coords = coordinates()

    safe_movel(coords["way_point"], require_grasp=True)
    safe_movel(coords["wash_approach"], require_grasp=True)
    safe_movel(coords["wash_place"], require_grasp=True)
    apply_wash_place_force()

    # 세척 위치에 수거통을 내려놓고 수도 레버를 조작한다.
    gripper_open()
    safe_movel(coords["wash_approach"])
    safe_movel(coords["way_point"])
    safe_movel(coords["wash_close"])
    gripper_close()
    _ds.wait(1.5)
    safe_movej(coords["wash_open"], vel=SLOW_VELX, acc=SLOW_ACCX)
    safe_movel(coords["wash_close"])
    gripper_open()

    # 세척이 끝난 수거통을 다시 파지한다.
    safe_movel(coords["way_point"])
    safe_movel(coords["wash_approach"])
    safe_movel(coords["wash_pick"], vel=SLOW_VELX, acc=SLOW_ACCX)
    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "세척 후 수거통 파지 불량")
        gripper_open()
        raise RuntimeError("세척 후 수거통의 위치를 확인해 주세요")

    safe_movel(coords["wash_approach"], require_grasp=True)
    safe_movel(coords["way_point"], require_grasp=True)

    # 오수 배출: 교시된 배출/기울임/흔들기 좌표를 순서대로 사용한다.
    safe_movel(coords["water_out_approach"], require_grasp=True)
    safe_movej(coords["water_out_tilt"], vel=SLOW_VELX, acc=SLOW_ACCX, require_grasp=True)

    # 새로 추가: water_out_tilt와 water_out_shake를 빠르게 왕복해 물을 턴다.
    for idx in range(SHAKE_REPEAT_COUNT):
        status.set_state(
            ProcessState.WASHING,
            f"water shaking {idx + 1}/{SHAKE_REPEAT_COUNT}",
        )
        safe_movel(
            coords["water_out_shake"],
            vel=SHAKE_VEL,
            acc=SHAKE_ACC,
            require_grasp=True,
        )
        safe_movej(
            coords["water_out_tilt"],
            vel=SHAKE_VEL,
            acc=SHAKE_ACC,
            require_grasp=True,
        )

    safe_movel(coords["water_out_approach"], require_grasp=True)


def return_bin_and_complete():
    coords = coordinates()

    # 새로 추가: 세척과 배수가 끝난 수거통을 원래 위치에 내려놓는다.
    status.set_state(ProcessState.MOVING, "수거통 원위치 및 초기 위치 복귀")
    safe_movel(coords["way_point"], require_grasp=True)
    safe_movel(coords["bin_approach"], require_grasp=True)
    safe_movel(
        coords["bin_pick"],
        vel=SLOW_VELX,
        acc=SLOW_ACCX,
        require_grasp=True,
    )
    gripper_open()
    safe_movel(coords["bin_pick_top"])
    safe_movej(coords["home"])
    status.set_state(ProcessState.COMPLETE, "배출 및 세척 완료")


def run_process(mode: int):
    try:
        check_system_ready()
        pick_bin()
        run_dump_motion(mode)
        execute_wash()
        return_bin_and_complete()
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
    # 로봇 제어 노드 생성
    node = rclpy.create_node("food_waste_dump_robot", namespace=ROBOT_ID)
    # 전역변수로 할당
    g_node = node
    # DR_init의 __dsr__node에 할당
    DR_init.__dsr__node = node

    # 노드에 덤프 모드 파라미터 선언
    node.declare_parameter("dump_mode", DUMP_MODE_NORMAL)
    # 노드에 자동시작 파라미터 선언
    node.declare_parameter("autostart", False)

    status = StatusBus(node)

    init_robot_api()
    init_gripper_api()

    # 명세서의 /robot/dump_cmd 역할: HMI/FastAPI에서 Trigger 호출 시 전체 공정 시작
    # 프론트엔트/백엔드 연동 필요
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
