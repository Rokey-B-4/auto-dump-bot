"""
음식물 수거통 자동 배출/세척 로봇 제어 노드
- gear_insert.py 구조를 기반으로 작성
- B-4 요구사항 명세서 REQ-01 ~ REQ-09 반영

실행 예시:
  ros2 run <pkg> food_waste_dump_robot --ros-args -p mode:=virtual -p dump_mode:=1 -p autostart:=true

주의:
  아래 좌표는 반드시 실제 지그/수거통/세척 위치에 맞게 교시 후 수정해야 한다.
"""

import json
import math
import time
from enum import Enum

import rclpy
import DR_init
from std_msgs.msg import String, Bool, Int32
from dsr_msgs2.srv import MoveStop
from onrobot_rg_msgs.srv import SetCommand

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==============================================================================
# [운전 파라미터]
# ==============================================================================
# 일반 이동은 하나의 속도 프로파일을 사용한다.
# movej: [deg/s], [deg/s^2]
# movel: [선속도 mm/s, 각속도 deg/s], [선가속도 mm/s^2, 각가속도 deg/s^2]
# 두 명령의 단위가 다르므로 같은 숫자를 쓰지 않고, 회전 속도도 명시한다.
VELOCITYJ, ACCJ = 30, 30
VELOCITYX, ACCX = [80, 15], [120, 30] # [80, 15], [160, 30]

# 실제 장착 공구/TCP 설정
TOOL_NAME = "GripperDA_v1"
TOOL_WEIGHT_KG = 0.900
TOOL_CENTER_OF_GRAVITY_MM = [-13.780, 102.440, 86.330] # 실제값 적용
TOOL_INERTIA = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
TCP_NAME = "Tool_Weight1"
TCP_OFFSET = [0.0, 0.0, 228.0, 0.0, 0.0, 0.0]

# 주기 털기 반복 횟수
SHAKE_REPEAT_COUNT = 5

# move_periodic 털기 파라미터: [X, Y, Z, Rx, Ry, Rz]
DUMP_NORMAL_PERIODIC_AMP = [0, 10, 0, 0, 0, 0]
DUMP_STRONG_PERIODIC_AMP = [0, 18, 0, 0, 0, 0]
WATER_PERIODIC_AMP = [0, 12, 0, 0, 0, 0]
DUMP_NORMAL_PERIOD = 0.8
DUMP_STRONG_PERIOD = 0.65
WATER_PERIOD = 0.8
PERIODIC_SHAKE_ATIME = 0.2

# 충돌/이탈 감지 기준
COLLISION_FORCE_N = 30.0      # F/T 센서 외력 임계값[N]
GRIPPER_INPUT_IDX = 1         # 실제 파지 확인용 Tool DI 번호. 현장 배선에 맞게 수정

# 외력제어 파라미터
FORCE_TH = 20.0  # place시 외력감지 Threshold
DESIRED_FORCE_X = 0.0     # 세척 위치 안착 방향 힘[N] - 베이스 좌표계 +X 방향
DESIRED_FORCE_Z = 10.0     # 세척 위치 Z방향 힘[N] - 실기 테스트 후 조정
COMPLIANCE_X = 300         # X 순응 강성 - 낮을수록 +X 방향 접촉면을 부드럽게 따라감
COMPLIANCE_Y = 3000        # Y 순응 강성 - Y방향의 불필요한 움직임을 억제
COMPLIANCE_Z = 2000        # Z 순응 강성 - 낮을수록 부드럽게 눌림
FORCE_CONTROL_TIME = 5.0   # 목표 외력을 유지하며 안착시킬 시간[s]

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
_last_grasp_log = None


status = None
dsr_node = None
gripper_client = None

# 1. 상태 및 에러 정의서 (Enum 클래스) 
# 프로그램 전체에서 쓰이는 '단어장' 역할, 오타로 인한 버그를 막기 위해 사용
# "소문자 idle인가? 대문자 IDLE인가?" 헷갈리지 않도록 통신 규격을 완벽하게 통일해 주는 역할
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
        # 초기 대기 및 종료 위치
        "home": posj(0, 0, 90, 0, 90, 0), # start end 
        
        # 배출 위치에서 세척 위치로 이동할 때의 경유점
        "way_point_j": posj(-76.27, 47.33, 97.16, 65.18, 105.46, -56.09),

        # way - home

        # 수거통 픽업 위치
        "bin_approach": posj(-43.93, 60.63, 77.03, 55.39, 117.12, -56.03),

        # 상대좌표: 직전 위치 기준, 로봇 베이스(조인트) 좌표계
        "bin_pick": posx(0, 0, 95, 0, 0, 0),
        "bin_pick_top": posx(0, 130, 0, 0, 0, 0),
        "dump_approach": posx(0, 0, 180, 0, 0, 0),
        "dump_tilt": posx(0, 0, 0, 160, 0, 0),
        "dump_tilt_back": posx(0, 0, 0, -160, 0, 0),

        # 세척 위치
        "wash_approach_x": posx(672.14, 17.01, 87.32, 0.88, 90.95, 90.88),
        "wash_approach_j": posj(0.92, 38.79, 130.96, 180.1, 78.8, -89.2),
        "wash_place": posx(0, 0, 45, 0, 0, 0),
        # 수도꼭지 컨트롤
        "wash_app_j": posj(-16.92, 52.98, 31.28, 50.5, 101.02, -56.09),
        "wash_open": posx(0, 0, 0, -140, 0, 0),
        "wash_close": posx(0, 0, 0, 140, 0, 0),
        "wash_pick": posx(0, -15, 40, 0, 0, 0),
        "wash_up": posx(0, 15, 0, 0, 0, 0),
        
        # 세척수 배출 위치
        "water_out_approach_j": posj(-28.57, 57.89, 69.03, 71.59, 113.57, -39.85),
        "water_out_tilt": posx(0, 0, 0, 140, 0, 0),
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

    # 로봇의 팔다리를 움직이고 감각을 느끼게 해줄 20개의 '신경망(ROS2 Service)'
    # (실제 필요한 서비스 리스트)
    required_services = [
        _ds._ros2_set_current_tool,
        _ds._ros2_set_current_tcp,
        _ds._ros2_config_create_tool,
        _ds._ros2_config_create_tcp,
        _ds._ros2_get_current_tool,
        _ds._ros2_get_current_tcp,
        _ds._ros2_set_singularity_handling,
        _ds._ros2_movej,
        _ds._ros2_movel,
        _ds._ros2_move_periodic,
        _ds._ros2_check_motion,
        _ds._ros2_get_tool_force,
        _ds._ros2_get_current_posx,
        _ds._ros2_get_current_posj,
        _ds._ros2_get_digital_input,
        _ds._ros2_get_tool_digital_input,
        _ds._ros2_set_digital_output,
        _ds._ros2_task_compliance_ctrl,
        _ds._ros2_set_desired_force,
        _ds._ros2_release_force,
        _ds._ros2_release_compliance_ctrl,
    ]

    # 서비스 리스트 각각이 생성되었는지 확인
    g_node.get_logger().info("Waiting for DSR controller services...")
    for client in required_services:
        if not client.wait_for_service(timeout_sec=30.0):
            # 없으면 에러
            raise RuntimeError(f"DSR service is not available: {client.srv_name}")
    # 다 체크 완료되면 준비완 표시
    g_node.get_logger().info("DSR controller services are ready")

    # 노드에 대한 파라미터 선언
    mode = g_node.declare_parameter(
        "mode", "virtual"
    ).get_parameter_value().string_value

    # 실제모드에서는 교시 좌표와 동일한 공구/TCP를 반드시 선택한다.
    if mode == "real":
        if _ds.set_tcp(TCP_NAME) != 0:
            if _ds.add_tcp(TCP_NAME, TCP_OFFSET) != 0 or _ds.set_tcp(TCP_NAME) != 0:
                g_node.get_logger().info(f"TCP 등록/선택 실패: {TCP_NAME}")

        if _ds.set_tool(TOOL_NAME) != 0:
            if TOOL_CENTER_OF_GRAVITY_MM is None or TOOL_INERTIA is None:
                g_node.get_logger().info(
                    f"공구 '{TOOL_NAME}'가 미등록 상태입니다. "
                    "TOOL_CENTER_OF_GRAVITY_MM과 TOOL_INERTIA를 입력해 주세요."
                )
            if (
                _ds.add_tool(TOOL_NAME, TOOL_WEIGHT_KG, TOOL_CENTER_OF_GRAVITY_MM, TOOL_INERTIA) != 0
                or _ds.set_tool(TOOL_NAME) != 0
            ):
                g_node.get_logger().info(f"공구 등록/선택 실패: {TOOL_NAME}")

        g_node.get_logger().info(f"Tool/TCP selected: {_ds.get_tool()} / {_ds.get_tcp()}")
    
    # 가상모드면 set_tool, set_tcp 값 확인 넘어감
    elif mode == "virtual":
        g_node.get_logger().info("Virtual mode: skip real Tool/TCP registration")
    else:
        raise RuntimeError("mode must be 'virtual' or 'real'")

    if _ds.set_singularity_handling(_ds.DR_AVOID) != 0:
        raise RuntimeError("Failed to set singularity handling")

# 그리퍼 초기화 함수
# 그리퍼가 단순히 열리고 닫히기 전, 시스템과 안전하게 통신하기 위한 세팅
def init_gripper_api():
    global gripper_client
    gripper_client = dsr_node.create_client(SetCommand, "/onrobot/sendCommand")
    if not gripper_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("RG2 service is not available: /onrobot/sendCommand")
    g_node.get_logger().info("RG2 service is ready")


# ==============================================================================
# [ROS 상태 출력] - 로봇의 상태 정보 토픽으로 전달(상태 알림 담당 클래스)
# ==============================================================================
# 통신 중계소 (StatusBus 클래스)
# ROS2 시스템과 외부(HMI/웹)를 연결해 주는 핵심 브릿지
# 역할: 로봇 내부에서 일어나는 일을 /robot/... 이라는 ROS2 토픽(Topic)으로 바깥 세상에 방송
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

# 역할: 충돌이 나거나 통을 떨어뜨렸을 때 호출되는 비상 알림 시스템
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
def stop(mode=None): # 역할: 로봇을 멈추게 하는 브레이크
    """DRL stop()과 동일한 목적의 ROS2 MoveStop 래퍼."""
    stop_mode = _ds.DR_QSTOP if mode is None else mode #  비상 정지 버튼의 강도를 정하고, 로봇에게 신호를 보낼 직통 전화선을 연다.
    client = g_node.create_client(
        MoveStop,
        f"/{ROBOT_ID}/dsr_controller2/motion/move_stop",
    )
    if not client.wait_for_service(timeout_sec=1.0):
        return -1
    # 역할: 브레이크 명령을 전송하고, 로봇이 실제로 멈췄는지 확인

    req = MoveStop.Request()
    req.stop_mode = int(stop_mode)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(g_node, future, timeout_sec=2.0)
    result = future.result() if future.done() else None
    return 0 if result and result.success else -1

# ==============================================================================
# [그리퍼 / 센서 / 밸브]
# ==============================================================================
def send_gripper_command(command: str):
    req = SetCommand.Request()
    req.command = command
    future = gripper_client.call_async(req)
    rclpy.spin_until_future_complete(dsr_node, future, timeout_sec=10.0)

    if not future.done():# 만약 10초가 지났는데도 그리퍼가 안 닫히면 에러 띄우고 멈추기. 
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
    global _last_grasp_log
    mode = g_node.get_parameter("mode").value
    if mode == "virtual":
        status.publish_gripper(True)
        return True

    try:# 역할: 실제 현장에서 그리퍼에 달린 센서의 전기 신호를 읽어옵니다.
        val = int(_ds.get_digital_input(GRIPPER_INPUT_IDX))
    except Exception as exc: # 역할: 센서 자체가 고장 났거나 선이 끊어졌을 때를 대비한 방어막
        status.publish_gripper(False)
        status.publish_safety(ErrorCode.ERR_PICK, f"gripper_digital_input 읽기 실패: {exc}")
        return False
    # 0 이 ON임
    grasped = not bool(val)
    status.publish_gripper(grasped)
    
    # 이전 로그와 같으면 출력을 안하고 다를때만 한번 출력
    log_state = (GRIPPER_INPUT_IDX, val, grasped)
    if log_state != _last_grasp_log:
        g_node.get_logger().info(
            f"Gripper DI grasp check: index={GRIPPER_INPUT_IDX}, value={val}, grasped={grasped}"
        )
        _last_grasp_log = log_state
    return grasped


# ==============================================================================
# [안전 감시]
# ==============================================================================
# 역할: X, Y, Z축으로 들어오는 3차원 힘을 계산하여 하나의 스칼라 값으로 만듬
def current_force_norm() -> float:
    force = _ds.get_tool_force(_ds.DR_BASE)
    return math.sqrt(force[0] ** 2 + force[1] ** 2 + force[2] ** 2)

# 역할: 최후의 비상 정지(Kill Switch) 버튼
# 행동: 충돌이나 통 떨어짐이 감지되면 이 함수가 호출됨 
# ➔ 즉시 stop()으로 로봇을 세움 
# ➔ ERROR 상태를 웹으로 쏨 
# ➔ 혹시 물을 틀어놨을까 봐 valve_close()로 수도꼭지를 강제로 잠금 
# ➔ 파이썬 시스템을 완전히 다운시킴. (완벽한 2차 사고 방지)
def raise_safety_stop(code: ErrorCode, msg: str):
    stop(_ds.DR_QSTOP)
    status.set_state(ProcessState.COLLISION if code == ErrorCode.ERR_COLLISION else ProcessState.ERROR)
    status.publish_safety(code, msg)

# safety_watch 역할: 순찰대원
# current_force_norm이 35N을 넘는지 감시하고, require_grasp=True일 때는 is_grasped()로 통을 쥐고 있는지도 동시에 감시
# 하나라도 어긋나면 위에서 말한 raise_safety_stop을 누름
def safety_watch(require_grasp: bool = False):
    
    # 판단 하나: 합성력이 임계값 초과?
    force_norm = current_force_norm()
    if force_norm > COLLISION_FORCE_N:
        stop(_ds.DR_QSTOP)                          # 즉시 급정지
        msg = f"외력 감지로 긴급 정지: {force_norm:.1f}N"
        status.set_state(ProcessState.COLLISION, msg)
        status.publish_safety(ErrorCode.ERR_COLLISION, msg)
        raise RuntimeError(msg)  # 런타임 에러가 아니라 다른 식으로 에러 정보를 띄우고 긴급 정지로?
    
    # 수거통 이탈 감지 (기존 기능 유지)
    if require_grasp and not is_grasped():
        stop(_ds.DR_QSTOP)
        msg = "이동 중 수거통 이탈 감지"
        status.set_state(ProcessState.COLLISION, msg)
        status.publish_safety(ErrorCode.ERR_DROP, msg)
        raise RuntimeError(msg)  # 런타임 에러가 아니라 다른 식으로 에러 정보를 띄우고 긴급 정지로?

# safe_movej, safe_movel, safe_wait 의 역할: 로봇이 움직이는(amovej, amovel) 동안,
# 도착할 때까지 멍 때리지 않고 0.05초마다 계속 safety_watch()를 실행시키는 함수 
def safe_movej(target, vel=VELOCITYJ, acc=ACCJ, require_grasp=False):
    _ds.amovej(target, vel=vel, acc=acc)
    while _ds.check_motion():
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.01)


def safe_movel(target, vel=VELOCITYX, acc=ACCX, require_grasp=False, ref=None, mod=None):
    ref = _ds.DR_BASE if ref is None else ref
    mod = _ds.DR_MV_MOD_ABS if mod is None else mod
    _ds.amovel(target, vel=vel, acc=acc, ref=ref, mod=mod)
    while _ds.check_motion():
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.01)


def safe_movel_relative(target, require_grasp=False, vel=VELOCITYX, acc=ACCX):
    """직전 TCP 자세에서 Tool 좌표계로 상대 이동한다."""
    safe_movel(target, vel=vel, acc=acc, require_grasp=require_grasp,
               ref=_ds.DR_TOOL, mod=_ds.DR_MV_MOD_REL)


def safe_move_periodic(amp, period, atime, repeat, ref, require_grasp=False):
    """비동기 주기 운동 중 충돌과 수거통 이탈을 계속 감시한다."""
    if _ds.amove_periodic(
        amp=amp,
        period=period,
        atime=atime,
        repeat=repeat,
        ref=ref,
    ) != 0:
        raise RuntimeError("주기 털기 동작 시작 실패")

    while _ds.check_motion():
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.05)


def safe_wait(seconds: float, require_grasp=False):
    start = time.time()
    while time.time() - start < seconds:
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.05)

# 수거통을 세척 통에 내려놓을 때 쾅 부딪히지 않고 사람이 손으로 꾹 눌러,
# 끼우듯 일정한 힘(X축 65N, Z축 10N)으로 부드럽게 밀어 넣습니다.
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

# 1단계: 준비 및 초기화(check_system_ready)
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

# 2단계: 수거통 집어 들기 (pick_bin)
def pick_bin():
    status.set_state(ProcessState.MOVING, "수거통 위치 이동 및 파지")
    coords = coordinates()

    safe_movej(coords["way_point_j"])
    safe_movej(coords["bin_approach"]) # 수거통 근처 절대좌표(bin_approach)로 다가갑니다.
    safe_movel_relative(coords["bin_pick"])

    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "수거통 미감지 또는 파지 불량")
        gripper_open()
        raise RuntimeError("수거통의 위치를 확인해 주세요")

    safe_movel_relative(coords["bin_pick_top"], require_grasp=True)

# 음식물 쓰레기 폐기통에 버리는 모션
# 주기적 shake 모션 함수
def run_periodic_dump_shake(mode):
    """dump_tilt 자세를 중심으로 Tool Y축 주기 운동을 수행한다."""
    if mode == DUMP_MODE_NORMAL:
        amp = DUMP_NORMAL_PERIODIC_AMP
        period = DUMP_NORMAL_PERIOD
    elif mode == DUMP_MODE_STRONG:
        amp = DUMP_STRONG_PERIODIC_AMP
        period = DUMP_STRONG_PERIOD
    else:
        raise ValueError("dump mode must be 1(normal) or 2(strong)")

    status.set_state(
        ProcessState.DUMPING,
        f"periodic shaking {SHAKE_REPEAT_COUNT} cycles",
    )
    safe_move_periodic(
        amp=amp,
        period=period,
        atime=PERIODIC_SHAKE_ATIME,
        repeat=SHAKE_REPEAT_COUNT,
        ref=_ds.DR_TOOL,
        require_grasp=True,
    )

#3 단계: 배출 및 강력 털기 (run_dump_motion)
def run_dump_motion(mode: int):
    status.set_state(ProcessState.DUMPING, f"mode={mode}")
    status.publish_mode(mode)
    coords = coordinates()

    if mode not in (DUMP_MODE_NORMAL, DUMP_MODE_STRONG):
        raise ValueError("dump mode must be 1(normal) or 2(strong)")

    safe_movel_relative(coords["dump_approach"], require_grasp=True)
    safe_movel_relative(coords["dump_tilt"], require_grasp=True)

    run_periodic_dump_shake(mode)
    safe_movel_relative(coords["dump_tilt_back"], require_grasp=True)

def run_periodic_water_shake():
    """water_out_tilt 자세를 중심으로 Tool X축 주기 운동을 수행한다."""
    status.set_state(
        ProcessState.WASHING,
        f"periodic water shaking {SHAKE_REPEAT_COUNT} cycles",
    )
    safe_move_periodic(
        amp=WATER_PERIODIC_AMP,
        period=WATER_PERIOD,
        atime=PERIODIC_SHAKE_ATIME,
        repeat=SHAKE_REPEAT_COUNT,
        ref=_ds.DR_TOOL,
        require_grasp=True,
    )

# 4단계: 세척 및 오수 배출 (execute_wash)
def execute_wash():
    status.set_state(ProcessState.WASHING, "세척 위치 이동")
    coords = coordinates()

    safe_movej(coords["way_point_j"], require_grasp=True)
    safe_movej(coords["wash_approach_j"], require_grasp=True) #세척기 앞(wash_approach_x)으로 이동
    safe_movel_relative(coords["wash_place"], require_grasp=True)
    apply_wash_place_force() # 수거통을 세척 지그(Jig)에 내려놓을 때 쾅 부딪히지 않고 사람이 손으로 꾹 눌러 끼우듯,
    # 일정한 힘(X축 65N, Z축 10N)으로 부드럽게 밀어 넣습니다.

    # 세척 위치에 수거통을 내려놓고 수도 레버를 조작한다.
    gripper_open()
    safe_movej(coords["wash_approach_j"])
    safe_movej(coords["way_point_j"])
    safe_movej(coords["wash_app_j"])
    gripper_close()
    safe_movel_relative(coords["wash_close"])
    safe_movel_relative(coords["wash_open"])
    gripper_open()

    # 세척이 끝난 수거통을 다시 파지한다.
    safe_movej(coords["way_point_j"])
    safe_movej(coords["wash_approach_j"])
    safe_movel_relative(coords["wash_pick"])
    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "세척 후 수거통 파지 불량")
        gripper_open()
        raise RuntimeError("세척 후 수거통의 위치를 확인해 주세요")

    safe_movel_relative(coords["wash_up"], require_grasp=True)
    safe_movej(coords["wash_approach_j"], require_grasp=True)
    safe_movej(coords["way_point_j"], require_grasp=True)

    # 오수 배출: 교시된 배출/기울임/흔들기 좌표를 순서대로 사용한다.
    safe_movej(coords["water_out_approach_j"], require_grasp=True)
    safe_movel_relative(coords["water_out_tilt"], require_grasp=True)

    run_periodic_water_shake()

    safe_movej(coords["water_out_approach_j"], require_grasp=True)


# 5단계: 원위치 복귀 및 종료 (return_bin_and_complete)
def return_bin_and_complete():
    coords = coordinates()

    # 새로 추가: 세척과 배수가 끝난 수거통을 원래 위치에 내려놓는다.
    status.set_state(ProcessState.MOVING, "수거통 원위치 및 초기 위치 복귀")
    safe_movej(coords["way_point_j"], require_grasp=True)
    safe_movej(coords["bin_approach"], require_grasp=True)
    safe_movel_relative(coords["bin_pick"], require_grasp=True)
    gripper_open()
    safe_movel_relative(coords["bin_pick_top"])
    safe_movej(coords["home"])
    status.set_state(ProcessState.COMPLETE, "배출 및 세척 완료")


def run_process(mode: int):
    try:
        check_system_ready() # 1단계: 준비 및 초기화 (check_system_ready)
        pick_bin() # 2단계: 수거통 집어 들기 (pick_bin)
        run_dump_motion(mode) # 3단계: 모드별 배출 및 털기 (run_dump_motion)
        execute_wash() # 4단계: 세척 및 오수 배출 (execute_wash)
        return_bin_and_complete() # 5단계: 원위치 복귀 및 종료 (return_bin_and_complete)
        return True, "배출 및 세척 완료"
    except Exception as exc:
        status.set_state(ProcessState.ERROR, str(exc))
        try:
            stop(_ds.DR_QSTOP)
        except Exception:
            pass
        return False, str(exc)

import threading

_process_lock = threading.Lock()


def _run_process_worker(task_id, mode_id):
    """run_process를 별도 스레드에서 실행 (handle_robot_command 콜백을 즉시 반환시켜
    g_node의 spin이 막히지 않게 함 -> EMERGENCY_STOP 등 다른 명령이 즉시 처리 가능해짐)"""
    try:
        ok, result_message = run_process(mode_id)
        log = g_node.get_logger().info if ok else g_node.get_logger().error
        log(f"task_id={task_id}: {result_message}")
    except Exception as exc:
        g_node.get_logger().error(f"task_id={task_id}: run_process 실행 중 예외 발생: {exc}")
    finally:
        if _process_lock.locked():
            try:
                _process_lock.release()
            except RuntimeError:
                pass

import threading

_process_lock = threading.Lock()


# ==============================================================================
# [Command topic entry]
# ==============================================================================


def handle_robot_command(msg: String):
    """FastAPI가 발행한 작업 명령(START, MOVE_JOINT, HARDWARE_CONTROL)을 받아 처리한다."""
    try:
        command = json.loads(msg.data)
    except (json.JSONDecodeError, TypeError) as exc:
        g_node.get_logger().error(f"/robot/command JSON 파싱 실패: {exc}")
        return

    # 백엔드 브릿지 매니저에서 꽂아준 command_type 확인
    cmd_type = command.get("command_type") or command.get("command")

    # --------------------------------------------------------------------------
    # 케이스 1: [START] 전체 배출/세척 공정 시작
    # --------------------------------------------------------------------------
    if cmd_type == "START":
        task_id = command.get("task_id")
        mode_id = command.get("mode_id")

        if not isinstance(task_id, str) or not task_id.strip():
            g_node.get_logger().error("/robot/command에 유효한 task_id가 없습니다.")
            return
        if isinstance(mode_id, bool) or not isinstance(mode_id, int):
            g_node.get_logger().error(f"task_id={task_id}: mode_id는 정수여야 합니다: {mode_id!r}")
            return
        if mode_id not in (DUMP_MODE_NORMAL, DUMP_MODE_STRONG):
            g_node.get_logger().error(f"task_id={task_id}: 지원하지 않는 mode_id={mode_id}")
            return
        if not _process_lock.acquire(blocking=False):
            g_node.get_logger().warning(f"이미 공정이 진행 중입니다. task_id={task_id} 명령 무시.")
            return

        g_node.get_logger().info(f"공정 시작 명령 수신: task_id={task_id}, mode_id={mode_id}")

        # ★ 핵심 변경: run_process를 별도 스레드로 던지고 콜백은 즉시 반환
        #    -> g_node의 spin이 막히지 않아 EMERGENCY_STOP/RESET이 동작 중에도 즉시 처리됨
        threading.Thread(
            target=_run_process_worker,
            args=(task_id, mode_id),
            daemon=True,
        ).start()
        return

    # --------------------------------------------------------------------------
    # 케이스 2: [HARDWARE_CONTROL] 수동 그리퍼 우회 제어 연동 🔥 (추가)
    # --------------------------------------------------------------------------
    elif cmd_type == "HARDWARE_CONTROL":
        payload = command.get("payload", {})
        action = payload.get("action")       # "OPEN" 또는 "CLOSE"
        base_angle = payload.get("base_angle", 0.0)

        g_node.get_logger().info(f"[수동 하드웨어 제어 수신] action={action}, base_angle={base_angle}")
        
        # TODO: 필요하다면 base_angle 만큼 관절 1축(J1)을 먼저 회전시키는 로직을 추가할 수 있습니다.
        # 예: safe_movej_j1_only(base_angle)

        if action == "OPEN":
            g_node.get_logger().info("-> 로봇 그리퍼 OPEN 구동 수행")
            gripper_open() # 상단에 정의된 원래 로봇 그리퍼 오픈 함수 호출
        elif action == "CLOSE":
            g_node.get_logger().info("-> 로봇 그리퍼 CLOSE 구동 수행")
            gripper_close() # 상단에 정의된 원래 로봇 그리퍼 클로즈 함수 호출
        else:
            g_node.get_logger().warning(f"알 수 없는 하드웨어 액션: {action}")
        return

    # --------------------------------------------------------------------------
    # 케이스 3: [MOVE_JOINT] 수동 6축 관절각 슬라이더 제어 🔥 (추가)
    # --------------------------------------------------------------------------
    elif cmd_type == "MOVE_JOINT":
        payload = command.get("payload", {})
        g_node.get_logger().info(f"[수동 관절 이동 수신] joint_data={payload}")
        
        # 백엔드 payload에 담긴 J1~J6 실수값들을 순서대로 추출
        try:
            joint_angles = [
                float(payload.get("J1", 0.0)),
                float(payload.get("J2", 0.0)),
                float(payload.get("J3", 0.0)),
                float(payload.get("J4", 0.0)),
                float(payload.get("J5", 0.0)),
                float(payload.get("J6", 0.0))
            ]
            g_node.get_logger().info(f"-> 관절각 일괄 이동(MoveJ) 수행: {joint_angles}")
            safe_movej(joint_angles) 
            g_node.get_logger().info("-> MoveJ 완료")
            # 원래 ROS 코드에 설계되어 있는 safe_movej 나 로봇 제어 함수에 
            # 해당 관절각 배열(joint_angles)을 던져주시면 됩니다.
            # 예: safe_movej(joint_angles)
            
        except Exception as e:
            g_node.get_logger().error(f"MOVE_JOINT 구동 중 예외 발생: {e}")
        return

    # --------------------------------------------------------------------------
    # 케이스 3.5: [EMERGENCY_STOP] 비상정지 — 즉시 처리, 락 상태와 무관하게 동작
    # --------------------------------------------------------------------------
    elif cmd_type == "EMERGENCY_STOP":
        g_node.get_logger().warning("비상정지 명령 수신 — 즉시 정지 수행")
        try:
            raise_safety_stop(ErrorCode.ERR_SYSTEM, "사용자 비상정지 요청")
        except Exception as exc:
            g_node.get_logger().error(f"비상정지 처리 중 예외 발생: {exc}")
        finally:
            if _process_lock.locked():
                try:
                    _process_lock.release()
                except RuntimeError:
                    pass
        return

    # --------------------------------------------------------------------------
    # 케이스 3.6: [RESET] 비상정지/에러 후 안전 경유점을 거쳐 초기 위치로 복귀
    # --------------------------------------------------------------------------
    elif cmd_type == "RESET":
        if _process_lock.locked():
            g_node.get_logger().warning("공정 진행 중 - RESET 명령 무시됨")
            return

        g_node.get_logger().info("RESET 명령 수신 — 안전 경유점을 거쳐 초기 위치로 복귀")
        try:
            coords = coordinates()
            safe_movej(coords["way_point_j"])   # 경유점을 먼저 거쳐서
            safe_movej(coords["home"])          # 초기 위치로 복귀
            gripper_open()
            status.set_state(ProcessState.IDLE, "RESET 완료 - 초기 위치 복귀")
            g_node.get_logger().info("RESET 완료 — IDLE 상태로 복귀")
        except Exception as exc:
            g_node.get_logger().error(f"RESET 처리 중 예외 발생: {exc}")
            status.set_state(ProcessState.ERROR, str(exc))
        return

    # --------------------------------------------------------------------------
    # 케이스 4: 그 외 정의되지 않은 명령 필터링
    # --------------------------------------------------------------------------
    else:
        g_node.get_logger().warning(
            f"지원하지 않는 command_type 무시: {cmd_type!r}"
        )
        return

# ==============================================================================
# [메인]
# ==============================================================================
import threading
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

def main(args=None):
    global g_node, status, dsr_node

    rclpy.init(args=args)

    # ---- 노드 1: DSR API 전용 노드 (모션/그리퍼 제어) ----
    dsr_node = rclpy.create_node("food_waste_dump_robot_dsr", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node   # DSR API가 내부적으로 쓰는 노드는 이쪽 하나만

    dsr_executor = SingleThreadedExecutor()
    dsr_executor.add_node(dsr_node)
    dsr_spin_thread = threading.Thread(target=dsr_executor.spin, daemon=True)
    dsr_spin_thread.start()

    # ---- 노드 2: 토픽 구독/명령 처리 전용 노드 (메인 스레드) ----
    node = rclpy.create_node("food_waste_dump_robot", namespace=ROBOT_ID)
    g_node = node

    node.declare_parameter("dump_mode", DUMP_MODE_NORMAL)
    node.declare_parameter("autostart", False)

    status = StatusBus(node)
    init_robot_api()
    init_gripper_api()

    # 콜백 그룹 분리: START(오래 걸림)와 EMERGENCY_STOP/RESET(즉시 처리)를 분리
    process_callback_group = MutuallyExclusiveCallbackGroup()
    command_callback_group = MutuallyExclusiveCallbackGroup()

    node.create_subscription(
        String, "/robot/command", handle_robot_command, 10,
        callback_group=process_callback_group   # 일단 모든 명령을 같은 구독에서 받음
    )

    status.set_state(ProcessState.IDLE, "작업 대기")

    if node.get_parameter("autostart").value:
        mode = int(node.get_parameter("dump_mode").value)
        run_process(mode)

    # 멀티스레드 executor로 spin -> START가 한 스레드를 차지해도 다른 스레드가 비상정지 처리 가능
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        rclpy.spin(node)   # 메인 스레드는 명령 노드만 spin
    finally:
        try:
            gripper_open()
        except Exception:
            pass
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()