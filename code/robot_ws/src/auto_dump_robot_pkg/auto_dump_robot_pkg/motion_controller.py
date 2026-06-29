"""
음식물 수거통 자동 배출/세척 로봇 제어 노드
- gear_insert.py 구조를 기반으로 작성
- B-4 요구사항 명세서 REQ-01 ~ REQ-09 반영

실행 예시:
  ros2 run <pkg> food_waste_dump_robot --ros-args -p mode:=virtual -p dump_mode:=1 -p autostart:=true

주의:
  아래 좌표는 반드시 실제 지그/수거통/세척 위치에 맞게 교시 후 수정해야 한다.
"""

# 표준 라이브러리: JSON 파싱, 수학 계산, 멀티스레딩, 시간 처리, 열거형 정의
import json
import math
import threading
import time
from enum import Enum

# ROS2 통신 미들웨어 및 두산 로봇 전용 초기화 모듈
import rclpy
import DR_init
from std_msgs.msg import String, Bool, Int32
from dsr_msgs2.srv import GetRobotState, MoveStop
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
COLLISION_FORCE_N = 40.0      # F/T 센서 외력 임계값[N]
GRIPPER_INPUT_IDX = 1         # 실제 파지 확인용 Tool DI 번호. 현장 배선에 맞게 수정

# Doosan 컨트롤러가 system/get_robot_state 서비스로 반환하는 상태값 이름표.
# ProcessState/RecoveryStage와 달리 로봇 컨트롤러 내부 상태이므로 RESET 전에 별도로 확인한다.
ROBOT_STATE_NAMES = {
    0: "INITIALIZING",
    1: "STANDBY",
    2: "MOVING",
    3: "SAFE_OFF",
    4: "TEACHING",
    5: "SAFE_STOP",
    6: "EMERGENCY_STOP",
    7: "HOMMING",
    8: "RECOVERY",
    9: "SAFE_STOP2",
    10: "SAFE_OFF2",
    15: "NOT_READY",
}
# 아래 상태는 컨트롤러 안전정지/복구/준비불가 상태로 보고 프로그램 복구 모션을 차단한다.
ROBOT_CONTROLLER_SAFETY_STATES = {3, 5, 6, 8, 9, 10, 15}

# 외력제어 파라미터
FORCE_TH = 20.0  # place시 외력감지 Threshold
DESIRED_FORCE_X = 30.0     # 세척 위치 안착 방향 힘[N] - 베이스 좌표계 +X 방향
DESIRED_FORCE_Z = 10.0     # 세척 위치 Z방향 힘[N] - 실기 테스트 후 조정
COMPLIANCE_X = 300         # X 순응 강성 - 낮을수록 +X 방향 접촉면을 부드럽게 따라감
COMPLIANCE_Y = 3000        # Y 순응 강성 - Y방향의 불필요한 움직임을 억제
COMPLIANCE_Z = 2000        # Z 순응 강성 - 낮을수록 부드럽게 눌림
FORCE_CONTROL_TIME = 3.0   # 목표 외력을 유지하며 안착시킬 시간[s]

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

# ROS2 노드 및 컨트롤러 전역 참조 객체 (main()에서 초기화됨)
status = None
dsr_node = None
stop_node = None
gripper_client = None

# 1. 상태 및 에러 정의서 (Enum 클래스) 
# 프로그램 전체에서 쓰이는 '단어장' 역할, 오타로 인한 버그를 막기 위해 사용
# "소문자 idle인가? 대문자 IDLE인가?" 헷갈리지 않도록 통신 규격을 완벽하게 통일해 주는 역할
class ProcessState(str, Enum):
    """프로그램이 외부로 발행하는 공정 상태값 모음이다."""
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
    """안전 이벤트와 에러 로그에 사용하는 표준 오류 코드 모음이다."""
    ERR_PICK = "ERR_PICK"
    ERR_DROP = "ERR_DROP"
    ERR_COLLISION = "ERR_COLLISION"
    ERR_SYSTEM = "ERR_SYSTEM"

# 비상정지때 쓰는 에러 클래스
class EmergencyStopError(RuntimeError):
    """사용자 비상정지 또는 안전 감지로 공정을 즉시 중단할 때 사용한다."""

# 비상정지/충돌 후 RESET이 어떤 후동작을 해야 하는지 판단하기 위한 공정 체크포인트.
# 실제 로봇 상태가 아니라 프로그램이 마지막으로 통과한 안전한 의미 단위를 저장한다.
class RecoveryStage(str, Enum):
    """비상정지나 외력 정지 후 RESET이 이어갈 위치를 판단하기 위한 공정 체크포인트이다.
    
    실제 로봇 컨트롤러 상태가 아니라, 프로그램이 마지막으로 통과한 의미 있는 단계를 저장한다.
    """
    IDLE = "IDLE"
    READY = "READY"
    BIN_PICKING = "BIN_PICKING"
    BIN_PICK_FAILED = "BIN_PICK_FAILED"
    BIN_GRASPED = "BIN_GRASPED"
    DUMP_STARTED = "DUMP_STARTED"
    DUMP_APPROACHED = "DUMP_APPROACHED"
    DUMP_TILTING = "DUMP_TILTING"
    DUMP_TILTED = "DUMP_TILTED"
    DUMP_SHAKING = "DUMP_SHAKING"
    WASTE_DUMPED = "WASTE_DUMPED"
    DUMP_LEVELING = "DUMP_LEVELING"
    DUMP_LEVELED = "DUMP_LEVELED"
    # 세척 지그 주변에서는 바로 way_point_j로 빠지면 구조물과 부딪힐 수 있어
    # 접근장소(wash_approach_j)를 기준으로 복구 경로를 나눈다.
    # WASH_APPROACHING: 배출 위치 → 경유점 이동 중(아직 세척 지그 쪽이 아님 → 복구 시 경유점으로만 간다).
    # WASH_WAYPOINT_TO_APPROACH: 경유점 → 세척 진입장소 이동 중(세척 지그 주변 → 진입장소 경유 복구).
    WASH_APPROACHING = "WASH_APPROACHING"
    WASH_WAYPOINT_TO_APPROACH = "WASH_WAYPOINT_TO_APPROACH"
    WASH_APPROACH_REACHED = "WASH_APPROACH_REACHED"
    WASH_PLACING = "WASH_PLACING"
    WASH_JIG_PLACED = "WASH_JIG_PLACED"
    WASH_JIG_RELEASED = "WASH_JIG_RELEASED"
    WASH_JIG_LEAVING = "WASH_JIG_LEAVING"
    WASH_JIG_LEFT = "WASH_JIG_LEFT"
    WASH_TO_WAYPOINT = "WASH_TO_WAYPOINT"
    WASH_WAYPOINT_REACHED = "WASH_WAYPOINT_REACHED"
    WATER_VALVE_APPROACHING = "WATER_VALVE_APPROACHING"
    WATER_VALVE_READY = "WATER_VALVE_READY"
    WATER_VALVE_GRASPING = "WATER_VALVE_GRASPING"
    WATER_VALVE_GRASPED = "WATER_VALVE_GRASPED"
    WATER_VALVE_TURNING = "WATER_VALVE_TURNING"
    WATER_VALVE_TURNED = "WATER_VALVE_TURNED"
    WATER_VALVE_RETURNING = "WATER_VALVE_RETURNING"
    WATER_VALVE_RETURNED = "WATER_VALVE_RETURNED"
    WATER_IN_BIN = "WATER_IN_BIN"
    WASH_BIN_GRASPED = "WASH_BIN_GRASPED"
    WASH_BIN_LIFTED = "WASH_BIN_LIFTED"
    WASH_BIN_AT_APPROACH = "WASH_BIN_AT_APPROACH"
    WASH_BIN_LEFT = "WASH_BIN_LEFT"
    WATER_DUMPING = "WATER_DUMPING"
    WATER_DUMPED = "WATER_DUMPED"
    RETURNING_BIN = "RETURNING_BIN"
    BIN_RETURNED = "BIN_RETURNED"
    COMPLETE = "COMPLETE"


# RecoveryStage 열거값을 선언 순서(index)로 매핑한 딕셔너리 — recovery_stage_at_least() 비교에 사용
_RECOVERY_STAGE_ORDER = {
    stage: index for index, stage in enumerate(RecoveryStage)
}

# 스레드 간 경쟁 조건 방지를 위한 Lock/Event 전역 객체
_stage_lock = threading.Lock()
_stop_call_lock = threading.Lock()
_emergency_command_lock = threading.Lock()
_process_stage = RecoveryStage.IDLE
_emergency_stop_requested = threading.Event()
_dump_level_pose = None
# 밸브 돌림 복구용: 정상 공정에서 '밸브 시작(원위치) 자세'와 '끝까지 돌린 자세'를
# 관절값으로 기록해 두고, 돌림/복귀 도중 멈췄을 때 남은 부분만 movej로 마저 수행한다.
_valve_turn_start_posj = None
_valve_turn_target_posj = None

# 복구단계 설정 함수
def set_recovery_stage(stage: RecoveryStage, msg: str = ""):
    """현재 공정 체크포인트를 갱신한다.
    
    RESET 복구는 이 값 하나를 기준으로 분기하므로, 위험 구간 진입 전후에 반드시 호출한다.
    """
    global _process_stage
    with _stage_lock:
        _process_stage = stage
    # [branch_motion.py 대비 추가] 체크포인트가 바뀔 때마다 HMI용 복구 단계 토픽도 함께 발행
    # 기존에는 노드 내부에만 단계가 남아 HMI가 배출 전 HOME과 배출 후 RESUME을 구분할 수 없었음
    if status is not None:
        status.publish_recovery_stage(stage)
    if msg:
        g_node.get_logger().info(f"복구 체크포인트: {stage.value} - {msg}")

# 현재 복구단계 가져오는 함수
def get_recovery_stage() -> RecoveryStage:
    """스레드 락을 잡고 현재 공정 체크포인트를 읽는다."""
    with _stage_lock:
        return _process_stage


# 현재 복구 단계(stage)가 기준 단계(threshold) 이상인지 Enum 선언 순서로 비교하는 함수
def recovery_stage_at_least(stage: RecoveryStage, threshold: RecoveryStage) -> bool:
    """RecoveryStage 선언 순서를 기준으로 stage가 threshold 이후 단계인지 비교한다."""
    return _RECOVERY_STAGE_ORDER[stage] >= _RECOVERY_STAGE_ORDER[threshold]


# 비상정지 플래그(_emergency_stop_requested)가 SET 상태이면 즉시 모션을 멈추고 EmergencyStopError를 발생시키는 함수
def emergency_stop_if_requested():
    """비상정지 플래그가 켜져 있으면 즉시 정지 요청을 보내고 공정 루프를 탈출한다."""
    # START 공정 스레드 안의 모든 safe_* 루프가 이 플래그를 보고 즉시 탈출한다.
    if _emergency_stop_requested.is_set():
        request_motion_stop("사용자 비상정지 요청")
        raise EmergencyStopError("사용자 비상정지 요청")


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
        # [branch_motion.py 대비 수정] 실제 밸브 접근 자세에 맞춰 보정된 현재 관절 좌표입니다.
        # 기존 좌표(-16.92, 52.98, 31.28, 50.5, 101.02, -56.09)를 대체합니다.
        "wash_app_j": posj(-11.23, 50.7, 25.46, 36.49, 101.75, -56.09),
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
    """Doosan DSR API 모듈을 로드하고 로봇 ID, 모델, 좌표 생성 함수를 초기화한다."""
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
    """OnRobot RG2 그리퍼 서비스 클라이언트를 생성하고 서비스 준비 상태를 확인한다."""
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
    """공정 상태, 안전 이벤트, 그리퍼 상태, 모드 정보를 ROS 토픽으로 발행하는 중계 객체이다."""
    def __init__(self, node):
        """상태 발행에 필요한 ROS publisher들을 생성하고 초기 상태를 IDLE로 둔다."""
        self.node = node  # 전달받은 노드를 클래스 내에서 사용하기 위한 변수 할당
        self.state_pub = node.create_publisher(String, "/robot/process_state", 10)
        self.motion_pub = node.create_publisher(String, "/robot/motion_status", 10)
        self.safety_pub = node.create_publisher(String, "/robot/safety_event", 10)
        # [branch_motion.py 대비 추가] 로봇 내부 RecoveryStage를 백엔드/HMI로 전달하는 전용 토픽
        self.recovery_stage_pub = node.create_publisher(String, "/robot/recovery_stage", 10)
        self.gripper_pub = node.create_publisher(Bool, "/gripper/status", 10)
        self.mode_pub = node.create_publisher(Int32, "/hmi/mode_cmd", 10)
        self.state = ProcessState.IDLE

    def set_state(self, state: ProcessState, msg: str = ""):
        """공정 상태를 내부에 저장하고 /robot/process_state, /robot/motion_status로 동시에 발행한다."""
        self.state = state
        # msg값이 비어있지 않으면 값을 내보내고 아니면 값과 msg값을 같이 내보냄 
        text = state.value if not msg else f"{state.value}:{msg}"
        
        self.state_pub.publish(String(data=text)) # state 발행
        self.motion_pub.publish(String(data=text)) # motion 발행
        self.node.get_logger().info(text) # text 출력

# 역할: 충돌이 나거나 통을 떨어뜨렸을 때 호출되는 비상 알림 시스템
    def publish_safety(self, code: ErrorCode, msg: str):
        """안전 이벤트를 표준 오류 코드와 메시지 형태로 발행한다."""
        text = f"{code.value}:{msg}"
        self.safety_pub.publish(String(data=text))
        self.node.get_logger().error(text)

    # [branch_motion.py 대비 추가] Enum 체크포인트를 문자열로 발행해 백엔드가 그대로 중계할 수 있게 함
    # 공정 상태 설명 문구와 분리했기 때문에 "재파지" 같은 단어로 단계가 오인되는 문제도 방지
    def publish_recovery_stage(self, stage: RecoveryStage):
        """HMI가 배출 완료 이후인지 정확히 판단할 수 있도록 복구 체크포인트를 발행한다."""
        self.recovery_stage_pub.publish(String(data=stage.value))

    def publish_gripper(self, grasped: bool):
        """현재 그리퍼 파지 여부를 /gripper/status로 발행한다."""
        self.gripper_pub.publish(Bool(data=grasped))

    def publish_mode(self, mode: int):
        """HMI가 보고 있는 모드 값을 /hmi/mode_cmd로 발행한다."""
        self.mode_pub.publish(Int32(data=mode))


# main()에서 StatusBus(node)로 초기화될 전역 상태 버스 인스턴스
status = None


# ==============================================================================
# [DR에 없거나 불안정한 함수 보완] 
# ==============================================================================
def stop(mode=None): # 역할: 로봇을 멈추게 하는 브레이크
    """Jazzy와 Humble의 서비스 경로를 모두 지원하는 MoveStop 래퍼."""
    stop_mode = _ds.DR_QSTOP if mode is None else mode
    node = stop_node if stop_node is not None else dsr_node
    service_names = (
        f"/{ROBOT_ID}/dsr_controller2/motion/move_stop",  # Jazzy/new driver
        f"/{ROBOT_ID}/motion/move_stop",                  # Humble driver
        "motion/move_stop",                               # node namespace relative
    )

    with _stop_call_lock:
        for service_name in service_names:
            client = node.create_client(MoveStop, service_name)
            if not client.wait_for_service(timeout_sec=0.2):
                continue

            req = MoveStop.Request()
            req.stop_mode = int(stop_mode)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.8)
            result = future.result() if future.done() else None
            if result is not None and result.success:
                g_node.get_logger().warning(
                    f"MoveStop 성공: service={service_name}, mode={int(stop_mode)}"
                )
                return 0

            g_node.get_logger().error(
                f"MoveStop 응답 실패: service={service_name}, mode={int(stop_mode)}"
            )

    g_node.get_logger().error(
        f"MoveStop 서비스를 찾지 못했거나 호출에 실패했습니다: {service_names}"
    )
    return -1

def request_motion_stop(reason: str = ""):
    """현재 모션에 정지 요청을 보낸다. QSTOP 실패 시 QSTOP_STO까지 한 번 더 시도한다."""
    # stop_node를 통해 호출되어 DSR API용 dsr_node의 spin과 충돌할 가능성을 줄인다.
    if reason:
        g_node.get_logger().warning(f"모션 정지 요청: {reason}")

    try:
        if stop(_ds.DR_QSTOP) == 0:
            return
    except Exception as exc:
        g_node.get_logger().error(f"DR_QSTOP 실패: {exc}")

    try:
        stop(_ds.DR_QSTOP_STO)
    except Exception as exc:
        g_node.get_logger().error(f"DR_QSTOP_STO 실패: {exc}")


def get_controller_robot_state():
    """Doosan 컨트롤러의 현재 robot_state를 조회한다. 조회 실패 시 None을 반환한다."""
    node = stop_node if stop_node is not None else dsr_node
    if node is None:
        return None

    client = node.create_client(
        GetRobotState,
        f"/{ROBOT_ID}/dsr_controller2/system/get_robot_state",
    )
    if not client.wait_for_service(timeout_sec=0.3):
        g_node.get_logger().warning("로봇 상태 조회 서비스가 준비되지 않았습니다: get_robot_state")
        return None

    future = client.call_async(GetRobotState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=0.8)
    if not future.done():
        g_node.get_logger().warning("로봇 상태 조회 시간 초과: get_robot_state")
        return None

    result = future.result()
    if result is None or not result.success:
        g_node.get_logger().warning("로봇 상태 조회 실패: get_robot_state")
        return None
    return int(result.robot_state)


def block_reset_if_controller_safety_stop() -> bool:
    """컨트롤러 자체 안전정지 상태이면 RESET 복구 모션을 차단한다."""
    # 로봇 자체 SAFE_STOP/EMERGENCY_STOP 상태에서는 movej/movel 명령이 먹지 않을 수 있으므로
    # 티치펜던트/컨트롤러에서 안전 해제가 끝나기 전까지 소프트웨어 복구를 시작하지 않는다.
    robot_state = get_controller_robot_state()
    if robot_state is None:
        return False

    state_name = ROBOT_STATE_NAMES.get(robot_state, f"UNKNOWN({robot_state})")
    g_node.get_logger().info(f"RESET 전 로봇 컨트롤러 상태 확인: {robot_state} {state_name}")

    if robot_state not in ROBOT_CONTROLLER_SAFETY_STATES:
        return False

    msg = (
        f"ROBOT_SAFETY_STOP:{state_name} - "
        "로봇 컨트롤러 안전정지 상태입니다. 현장 안전 확인 후 컨트롤러/티치펜던트에서 안전정지를 해제하세요."
    )
    status.set_state(ProcessState.ERROR, msg)
    status.publish_safety(ErrorCode.ERR_SYSTEM, msg)
    return True

# ==============================================================================
# [그리퍼 / 센서 / 밸브]
# ==============================================================================
# OnRobot RG2 그리퍼에 ROS2 서비스로 명령 문자열을 전달하고 완료 응답을 기다리는 하위 함수
def send_gripper_command(command: str):
    """RG2 그리퍼에 문자열 명령을 보내고 서비스 응답 성공 여부를 검사한다."""
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


# 그리퍼 열기: "o" 명령 전송 → 0.3초 대기 → 파지 상태 False 발행
def gripper_open():
    """그리퍼를 열고 외부 상태 토픽에 미파지 상태를 발행한다."""
    send_gripper_command("o")
    _ds.wait(0.3)
    status.publish_gripper(False)


# 그리퍼 닫기: "c" 명령 전송 → 0.5초 대기 (파지 확인은 is_grasped()로 별도 수행)
def gripper_close():
    """그리퍼를 닫는다. 실제 파지 성공 여부는 별도로 is_grasped()에서 확인한다."""
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
    """툴 기준 외력 벡터의 크기를 계산해 충돌 임계값 비교에 사용한다."""
    force = _ds.get_tool_force(_ds.DR_BASE)
    return math.sqrt(force[0] ** 2 + force[1] ** 2 + force[2] ** 2)


def current_posx_base():
    """현재 TCP pose를 DR_BASE 기준 posx로 읽는다. 실패하면 None을 반환한다."""
    try:
        result = _ds.get_current_posx(_ds.DR_BASE)
    except Exception as exc:
        g_node.get_logger().warning(f"현재 TCP pose 조회 실패: {exc}")
        return None
    if isinstance(result, tuple):
        return result[0]
    return result


def current_posj_or_none():
    """현재 관절 자세를 posj로 읽는다. 실패하면 None을 반환한다.

    밸브 돌림처럼 '멈춘 지점부터 남은 부분만 마저' 수행해야 하는 동작의 목표를
    관절 자세로 기록해 두기 위해 사용한다. 관절 절대 이동(movej)은 어디서 멈췄든
    기록된 자세로 정확히 이동하므로, cartesian 절대 pose보다 신뢰할 수 있다.
    """
    try:
        result = _ds.get_current_posj()
    except Exception as exc:
        g_node.get_logger().warning(f"현재 관절 자세 조회 실패: {exc}")
        return None
    if isinstance(result, tuple):
        return result[0]
    return result


# 사용자 비상정지, 프로그램 외력 감지, 수거통 이탈 감지를 모두 같은 순서로 처리한다.
# 정지 플래그 ON -> 컨트롤러 stop 요청 -> 상태/안전 이벤트 발행 -> 공정 루프 예외 탈출.
def trigger_safety_stop(code: ErrorCode, msg: str, state: ProcessState = None):
    """사용자 비상정지와 센서 안전정지를 같은 순서로 처리한다."""
    _emergency_stop_requested.set()
    request_motion_stop(msg)

    if state is None:
        state = ProcessState.COLLISION if code == ErrorCode.ERR_COLLISION else ProcessState.ERROR
    status.set_state(state, msg)
    status.publish_safety(code, msg)


# safety_watch 역할: 순찰대원
# current_force_norm이 35N을 넘는지 감시하고, require_grasp=True일 때는 is_grasped()로 통을 쥐고 있는지도 동시에 감시
# 하나라도 어긋나면 trigger_safety_stop으로 사용자 비상정지와 같은 정지 절차를 수행한다.
def safety_watch(require_grasp: bool = False):
    """모션 중 반복 호출되어 비상정지 플래그, 외력, 파지 상태를 감시한다."""
    emergency_stop_if_requested()
    
    # 판단 하나: 합성력이 임계값 초과?
    force_norm = current_force_norm()
    if force_norm > COLLISION_FORCE_N:
        msg = f"외력 감지로 긴급 정지: {force_norm:.1f}N"
        trigger_safety_stop(ErrorCode.ERR_COLLISION, msg, ProcessState.COLLISION)
        raise EmergencyStopError(msg)
    
    # 수거통 이탈 감지 (기존 기능 유지)
    if require_grasp and not is_grasped():
        msg = "이동 중 수거통 이탈 감지"
        trigger_safety_stop(ErrorCode.ERR_DROP, msg, ProcessState.COLLISION)
        raise EmergencyStopError(msg)

# safe_movej, safe_movel, safe_wait 의 역할: 로봇이 움직이는(amovej, amovel) 동안,
# 도착할 때까지 블로킹하지 않고 짧은 주기로 safety_watch()를 실행하는 함수
def safe_movej(target, vel=VELOCITYJ, acc=ACCJ, require_grasp=False):
    """관절 이동을 비동기로 시작한 뒤, 도착할 때까지 짧은 주기로 안전 감시를 수행한다."""
    emergency_stop_if_requested()
    _ds.amovej(target, vel=vel, acc=acc)
    while _ds.check_motion():
        safety_watch(require_grasp=require_grasp)
        _ds.wait(0.01)


def safe_movel(target, vel=VELOCITYX, acc=ACCX, require_grasp=False, ref=None, mod=None):
    """직선 이동을 비동기로 시작한 뒤, 이동 중 비상정지와 외력/파지 상태를 계속 확인한다."""
    emergency_stop_if_requested()
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
    emergency_stop_if_requested()
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
        _ds.wait(0.01)


# 지정 시간(seconds) 동안 50ms 간격으로 safety_watch를 호출하며 대기하는 함수 (외력/이탈 감시 유지)
def safe_wait(seconds: float, require_grasp=False):
    """단순 대기 중에도 안전 감시가 멈추지 않도록 짧게 쪼개서 기다린다."""
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
    """공정 시작 전 로봇/그리퍼/API 준비 상태를 확인하고 READY 체크포인트를 기록한다."""
    set_recovery_stage(RecoveryStage.IDLE, "공정 시작")
    status.set_state(ProcessState.INIT, "시스템 및 센서 체크 중")
    # 좌표 불러오기
    coords = coordinates()

    # IDLE 위치 검증: 여기서는 직접 home으로 이동해 안전 위치를 확보한다.
    #safe_movel(coords["bin_pick_top"])
    safe_movej(coords["home"])
    gripper_open()
    set_recovery_stage(RecoveryStage.READY, "초기 위치 및 그리퍼 오픈")
    status.set_state(ProcessState.READY, "초기 대기 위치 및 그리퍼 확인 완료")
    return True

# 2단계: 수거통 집어 들기 (pick_bin)
def pick_bin():
    """초기 위치에서 음식물 수거통을 집어 올리고 파지 성공 여부를 확인한다."""
    status.set_state(ProcessState.MOVING, "수거통 위치 이동 및 파지")
    coords = coordinates()

    set_recovery_stage(RecoveryStage.BIN_PICKING, "수거통 파지 위치 접근 중")
    safe_movej(coords["way_point_j"])
    safe_movej(coords["bin_approach"]) # 수거통 근처 절대좌표(bin_approach)로 다가갑니다.
    safe_movel_relative(coords["bin_pick"])

    gripper_close()
    set_recovery_stage(RecoveryStage.BIN_GRASPED, "수거통 파지 명령 완료")
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "수거통 미감지 또는 파지 불량")
        set_recovery_stage(RecoveryStage.BIN_PICK_FAILED, "수거통 미감지 - 통 없이 초기화 필요")
        gripper_open()
        raise RuntimeError("수거통의 위치를 확인해 주세요")

    set_recovery_stage(RecoveryStage.BIN_GRASPED, "음식물 수거통 파지 확인 완료")
    safe_movel_relative(coords["bin_pick_top"], require_grasp=True)

# 음식물 쓰레기 폐기통에 버리는 모션
# 주기적 shake 모션 함수
def run_periodic_dump_shake(mode):
    """dump_tilt 자세를 중심으로 Tool Y축 주기 운동을 수행한다."""
    if mode == DUMP_MODE_NORMAL:
        amp = DUMP_NORMAL_PERIODIC_AMP
        period = DUMP_NORMAL_PERIOD
        status_text = "일반 털기"
    elif mode == DUMP_MODE_STRONG:
        amp = DUMP_STRONG_PERIODIC_AMP
        period = DUMP_STRONG_PERIOD
        status_text = "강하게 털기"
    else:
        raise ValueError("dump mode must be 1(normal) or 2(strong)")

    status.set_state(
        ProcessState.DUMPING,
        status_text,
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
    """수거통을 배출 위치로 이동해 음식물을 버리고 다시 수평 자세로 복귀한다."""
    global _dump_level_pose
    status.set_state(ProcessState.DUMPING, f"mode={mode}")
    status.publish_mode(mode)
    set_recovery_stage(RecoveryStage.DUMP_STARTED, "음식물 배출 동작 진입")
    coords = coordinates()

    if mode not in (DUMP_MODE_NORMAL, DUMP_MODE_STRONG):
        raise ValueError("dump mode must be 1(normal) or 2(strong)")

    safe_movel_relative(coords["dump_approach"], require_grasp=True)
    set_recovery_stage(RecoveryStage.DUMP_APPROACHED, "음식물 배출 위치 접근 완료")
    _dump_level_pose = current_posx_base()

    set_recovery_stage(RecoveryStage.DUMP_TILTING, "음식물 배출 기울임 동작 중")
    safe_movel_relative(coords["dump_tilt"], require_grasp=True)
    set_recovery_stage(RecoveryStage.DUMP_TILTED, "음식물 배출 기울임 완료")

    set_recovery_stage(RecoveryStage.DUMP_SHAKING, "음식물 배출 털기 동작 중")
    run_periodic_dump_shake(mode)
    set_recovery_stage(RecoveryStage.WASTE_DUMPED, "음식물 배출 완료")

    set_recovery_stage(RecoveryStage.DUMP_LEVELING, "음식물 배출 후 수평 복귀 중")
    safe_movel_relative(coords["dump_tilt_back"], require_grasp=True)
    set_recovery_stage(RecoveryStage.DUMP_LEVELED, "음식물 배출 후 수평 복귀 완료")

def run_periodic_water_shake():
    """water_out_tilt 자세를 중심으로 Tool X축 주기 운동을 수행한다."""
    status.set_state(
        ProcessState.WASHING,
        f"물기 털기",
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
    """수거통을 세척 지그에 놓고 세척수 밸브를 조작한 뒤 세척수 배출 위치로 이동한다.
    
    밸브 조작 전후의 관절 자세를 기록해, 중간 정지 후 RESET 때 남은 동작만 이어갈 수 있게 한다.
    """
    status.set_state(ProcessState.WASHING, "세척 위치 이동")
    coords = coordinates()

    # 세척 지그 접근/안착 구간은 RESET 복구 경로가 달라서 세분화한다.
    # 배출 위치 → 경유점 이동과 경유점 → 세척 진입장소 이동을 별도 체크포인트로 나눈다.
    set_recovery_stage(RecoveryStage.WASH_APPROACHING, "배출 위치에서 경유점 이동 중")
    safe_movej(coords["way_point_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_WAYPOINT_TO_APPROACH, "경유점에서 세척 진입장소 이동 중")
    safe_movej(coords["wash_approach_j"], require_grasp=True) #세척기 앞(wash_approach_x)으로 이동
    set_recovery_stage(RecoveryStage.WASH_APPROACH_REACHED, "세척수 받는 진입장소 도착")
    set_recovery_stage(RecoveryStage.WASH_PLACING, "세척 위치에 수거통 내려놓는 중")
    safe_movel_relative(coords["wash_place"], require_grasp=True)
    apply_wash_place_force() # 수거통을 세척 지그(Jig)에 내려놓을 때 쾅 부딪히지 않고 사람이 손으로 꾹 눌러 끼우듯,
    # 일정한 힘(X축 65N, Z축 10N)으로 부드럽게 밀어 넣습니다.
    set_recovery_stage(RecoveryStage.WASH_JIG_PLACED, "세척 위치 안착 완료")

    # 세척 위치에 수거통을 내려놓고 수도 레버를 조작한다.
    gripper_open()
    set_recovery_stage(RecoveryStage.WASH_JIG_RELEASED, "세척 위치 수거통 릴리즈 완료")
    # 통을 내려놓은 뒤 지그에서 빠져나가는 중에 멈추면,
    # RESET 때 어느 안전 지점까지 지났는지 구분하기 위해 세부 체크포인트를 남긴다.
    set_recovery_stage(RecoveryStage.WASH_JIG_LEAVING, "세척 위치 통 안착 후 지그 이탈 중")
    safe_movej(coords["wash_approach_j"])
    set_recovery_stage(RecoveryStage.WASH_JIG_LEFT, "세척 위치 접근장소 경유 완료")
    set_recovery_stage(RecoveryStage.WASH_TO_WAYPOINT, "세척 위치 접근장소에서 경유점 이동 중")
    safe_movej(coords["way_point_j"])
    set_recovery_stage(RecoveryStage.WASH_WAYPOINT_REACHED, "세척 위치 이탈 후 경유점 도착")
    set_recovery_stage(RecoveryStage.WATER_VALVE_APPROACHING, "세척수 밸브 위치 이동 중")
    safe_movej(coords["wash_app_j"])
    # [branch_motion.py 대비 추가] 밸브 접근 이동의 잔여 진동이 가라앉은 뒤 파지하도록 안정화 시간을 둡니다.
    safe_wait(1.0)
    set_recovery_stage(RecoveryStage.WATER_VALVE_READY, "세척수 밸브 위치 도착 - 파지 전")
    set_recovery_stage(RecoveryStage.WATER_VALVE_GRASPING, "세척수 밸브 파지 중")
    gripper_close()
    set_recovery_stage(RecoveryStage.WATER_VALVE_GRASPED, "세척수 밸브 파지 완료 - 열기 전")

    # 돌림/복귀 도중 멈췄을 때 '남은 부분만' 마저 수행할 수 있도록,
    # 밸브 시작(원위치) 자세와 끝까지 돌린 자세를 관절값으로 기록해 둔다.
    global _valve_turn_start_posj, _valve_turn_target_posj
    _valve_turn_start_posj = current_posj_or_none()
    set_recovery_stage(RecoveryStage.WATER_VALVE_TURNING, "세척수 밸브 돌림 동작 중")
    safe_movel_relative(coords["wash_close"])
    _valve_turn_target_posj = current_posj_or_none()
    set_recovery_stage(RecoveryStage.WATER_VALVE_TURNED, "세척수 밸브 돌림 완료")

    set_recovery_stage(RecoveryStage.WATER_VALVE_RETURNING, "세척수 밸브 복귀 동작 중")
    safe_movel_relative(coords["wash_open"])
    set_recovery_stage(RecoveryStage.WATER_VALVE_RETURNED, "세척수 밸브 복귀 완료")
    # [branch_motion.py 대비 추가] 밸브 복귀 직후 바로 그리퍼를 열지 않도록 1초 대기합니다.
    safe_wait(1.0)
    gripper_open()
    set_recovery_stage(RecoveryStage.WATER_IN_BIN, "세척수 주입 완료")

    # 밸브 위치에서 세척 지그 접근점으로 직접 가지 않고 항상 경유점을 먼저 거친다.
    # 이 구간을 생략하면 주변 구조물과 간섭될 수 있다.
    safe_movej(coords["way_point_j"])

    # 세척이 끝난 수거통을 다시 파지한다.
    safe_movej(coords["wash_approach_j"])
    safe_movel_relative(coords["wash_pick"])
    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "세척 후 수거통 파지 불량")
        gripper_open()
        raise RuntimeError("세척 후 수거통의 위치를 확인해 주세요")

    set_recovery_stage(RecoveryStage.WASH_BIN_GRASPED, "세척 후 수거통 재파지 완료")
    safe_movel_relative(coords["wash_up"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_BIN_LIFTED, "세척 후 수거통 들어올림 완료")
    safe_movej(coords["wash_approach_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_BIN_AT_APPROACH, "세척 후 수거통 접근장소 도착")
    safe_movej(coords["way_point_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_BIN_LEFT, "세척 후 수거통 경유점 도착")

    # 오수 배출: 교시된 배출/기울임/흔들기 좌표를 순서대로 사용한다.
    safe_movej(coords["water_out_approach_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WATER_DUMPING, "세척수 배출 동작 진입")
    safe_movel_relative(coords["water_out_tilt"], require_grasp=True)

    run_periodic_water_shake()
    set_recovery_stage(RecoveryStage.WATER_DUMPED, "세척수 배출 완료")

    safe_movej(coords["water_out_approach_j"], require_grasp=True)


# 5단계: 원위치 복귀 및 종료 (return_bin_and_complete)
def return_bin_and_complete():
    """세척이 끝난 수거통을 원래 위치에 내려놓고 공정을 COMPLETE로 마무리한다."""
    coords = coordinates()

    # 새로 추가: 세척과 배수가 끝난 수거통을 원래 위치에 내려놓는다.
    status.set_state(ProcessState.MOVING, "수거통 원위치 및 초기 위치 복귀")
    set_recovery_stage(RecoveryStage.RETURNING_BIN, "수거통 원위치 복귀 시작")
    safe_movej(coords["way_point_j"], require_grasp=True)
    safe_movej(coords["bin_approach"], require_grasp=True)
    safe_movel_relative(coords["bin_pick"], require_grasp=True)
    gripper_open()
    set_recovery_stage(RecoveryStage.BIN_RETURNED, "수거통 원위치 내려놓기 완료")
    safe_movel_relative(coords["bin_pick_top"])
    safe_movej(coords["home"])
    set_recovery_stage(RecoveryStage.COMPLETE, "공정 완료")
    status.set_state(ProcessState.COMPLETE, "배출 및 세척 완료")


def recover_dump_pose_to_waypoint_after_reset(stage: RecoveryStage):
    """음식물 배출 중 멈춘 경우 기울어진 통을 수평으로 맞춘 뒤 경유점까지 복구한다."""
    # 음식물 배출 중/직후에 멈춘 경우에는 먼저 기울이기 전 pose로 되돌려
    # 수거통을 평행에 가깝게 맞춘 뒤 경유점으로 빠져나간다.
    coords = coordinates()
    status.set_state(ProcessState.DUMPING, "RESET: 음식물 배출 자세 복구")

    if stage in (
        RecoveryStage.DUMP_TILTING,
        RecoveryStage.DUMP_TILTED,
        RecoveryStage.DUMP_SHAKING,
        RecoveryStage.WASTE_DUMPED,
        RecoveryStage.DUMP_LEVELING,
    ):
        if _dump_level_pose is not None:
            safe_movel(_dump_level_pose, require_grasp=True)
        else:
            g_node.get_logger().warning("배출 전 기준 pose가 없어 상대 복귀 동작으로 수평 복구를 시도합니다.")
            safe_movel_relative(coords["dump_tilt_back"], require_grasp=True)
        set_recovery_stage(RecoveryStage.DUMP_LEVELED, "RESET 중 음식물 배출 자세 수평 복구 완료")

    safe_movej(coords["way_point_j"], require_grasp=True)


# RESET 복구: 수거통을 파지한 채로 원래 위치에 내려놓은 뒤 홈 위치로 복귀하는 함수
def return_bin_to_home_after_reset(reason: str):
    """통을 파지한 상태에서 RESET할 때 원래 위치에 내려놓고 홈으로 복귀한다."""
    coords = coordinates()
    status.set_state(ProcessState.MOVING, reason)
    set_recovery_stage(RecoveryStage.RETURNING_BIN, reason)
    safe_movej(coords["way_point_j"], require_grasp=True)
    safe_movej(coords["bin_approach"], require_grasp=True)
    safe_movel_relative(coords["bin_pick"], require_grasp=True)
    gripper_open()
    set_recovery_stage(RecoveryStage.BIN_RETURNED, "수거통 원위치 내려놓기 완료")
    safe_movel_relative(coords["bin_pick_top"])
    safe_movej(coords["home"])


# RESET 복구: 수거통 없이 경유점(way_point_j)을 거쳐 홈 위치로 복귀하고 그리퍼를 여는 함수
def reset_home_without_bin(reason: str):
    """통을 잡고 있지 않은 상태에서 RESET할 때 경유점을 거쳐 홈으로 복귀한다."""
    coords = coordinates()
    status.set_state(ProcessState.MOVING, reason)
    safe_movej(coords["way_point_j"])
    safe_movej(coords["home"])
    gripper_open()


# RESET 복구: 첫 수거통 파지 시도에서 통이 없던 경우 전용 복귀.
# 이때 로봇은 bin_pick 위치로 내려가 있으므로, 통 놓기 동작 없이
# bin_pick_top으로 곧장 들어올린 뒤 홈으로 복귀한다(통이 없으므로 파지 여부 무시).
def reset_bin_pick_failed_to_home(reason: str):
    """초기 파지 실패 상태에서 RESET할 때 통 내려놓기 없이 파지 위치를 빠져나와 홈으로 간다."""
    coords = coordinates()
    status.set_state(ProcessState.MOVING, reason)
    gripper_open()
    safe_movel_relative(coords["bin_pick_top"])
    safe_movej(coords["home"])
    gripper_open()


def retract_wash_area_to_waypoint(require_grasp: bool, reason: str):
    """세척 지그 주변에서 바로 경유점으로 가지 않고 접근 위치를 거쳐 구조물 충돌을 피한다."""
    # 세척 지그 주변에서 바로 way_point_j로 가면 구조물과 간섭될 수 있어
    # 항상 wash_approach_j를 먼저 거쳐 안전하게 빠져나간다.
    coords = coordinates()
    status.set_state(ProcessState.WASHING, reason)
    safe_movej(coords["wash_approach_j"], require_grasp=require_grasp)
    safe_movej(coords["way_point_j"], require_grasp=require_grasp)


def leave_wash_jig_after_reset():
    """세척 지그에 통을 내려놓은 뒤 멈춘 경우, 접근 위치와 경유점을 순서대로 거쳐 안전하게 빠져나온다."""
    # 통을 세척 위치에 내려놓은 뒤 밸브 위치로 가기 전까지의 이탈 경로를
    # 마지막 체크포인트에 맞춰 이어 간다.
    coords = coordinates()
    stage = get_recovery_stage()

    if stage == RecoveryStage.WASH_JIG_PLACED:
        gripper_open()
        set_recovery_stage(RecoveryStage.WASH_JIG_RELEASED, "RESET 중 세척 위치 수거통 릴리즈 완료")
        stage = RecoveryStage.WASH_JIG_RELEASED

    if stage in (RecoveryStage.WASH_JIG_RELEASED, RecoveryStage.WASH_JIG_LEAVING):
        set_recovery_stage(RecoveryStage.WASH_JIG_LEAVING, "RESET 중 세척 위치 통 안착 후 지그 이탈 중")
        safe_movej(coords["wash_approach_j"])
        set_recovery_stage(RecoveryStage.WASH_JIG_LEFT, "RESET 중 세척 위치 접근장소 도착")
        stage = RecoveryStage.WASH_JIG_LEFT

    if stage == RecoveryStage.WASH_JIG_LEFT:
        set_recovery_stage(RecoveryStage.WASH_TO_WAYPOINT, "RESET 중 세척 위치 접근장소에서 경유점 이동 중")
        safe_movej(coords["way_point_j"])
        set_recovery_stage(RecoveryStage.WASH_WAYPOINT_REACHED, "RESET 중 세척 위치 이탈 후 경유점 도착")
        stage = RecoveryStage.WASH_WAYPOINT_REACHED

    if stage == RecoveryStage.WASH_TO_WAYPOINT:
        safe_movej(coords["way_point_j"])
        set_recovery_stage(RecoveryStage.WASH_WAYPOINT_REACHED, "RESET 중 세척 위치 이탈 후 경유점 도착")


def _release_valve_and_leave(coords):
    """밸브 돌림/복귀가 끝난 뒤 밸브를 놓고 경유점으로 빠지는 공통 마무리."""
    gripper_open()
    set_recovery_stage(RecoveryStage.WATER_IN_BIN, "RESET 중 세척수 주입 완료")
    safe_movej(coords["way_point_j"])


def operate_water_valve_after_reset(stage: RecoveryStage):
    """세척수 밸브 주변에서 멈춘 상태를 복구해 밸브 열기/닫기 동작을 완결한다.
    
    밸브를 잡은 상태, 돌리는 중, 돌린 후, 복귀 중을 나눠 그리퍼 개방 시점을 늦춘다.
    """
    global _valve_turn_start_posj, _valve_turn_target_posj
    # 밸브 조작 복구 원칙:
    #  - 밸브를 잡기 전/직후(돌리기 시작 전)에 멈춘 경우: 밸브 위치(wash_app_j, 관절 목표)에서
    #    처음부터 파지하고 상대 이동으로 돌린다.
    #  - 돌리는 도중/복귀 도중에 멈춘 경우: '남은 부분만' 마저 수행한다.
    #    정상 공정에서 기록해 둔 관절 자세(_valve_turn_target_posj/_valve_turn_start_posj)로
    #    movej 이동하므로, 멈춘 지점에서 목표 자세까지 남은 각도만 움직인다(각도 어긋남 없음).
    #    (과거의 trans 절대 pose movel은 실제로 움직이지 않았고, 풀 상대 회전은 각도가 어긋난다.)
    coords = coordinates()
    status.set_state(ProcessState.WASHING, "복구: 세척수 밸브 조작")

    # 세척 지그 주변/경유 단계면 먼저 경유점까지 안전하게 빠져나온다.
    if stage in (
        RecoveryStage.WASH_JIG_PLACED,
        RecoveryStage.WASH_JIG_RELEASED,
        RecoveryStage.WASH_JIG_LEAVING,
        RecoveryStage.WASH_JIG_LEFT,
        RecoveryStage.WASH_TO_WAYPOINT,
    ):
        leave_wash_jig_after_reset()
        stage = RecoveryStage.WASH_WAYPOINT_REACHED

    # 밸브 돌림·복귀가 모두 끝난 뒤 멈춘 경우: 다시 돌릴 필요 없이 놓고 빠진다.
    if stage == RecoveryStage.WATER_VALVE_RETURNED:
        _release_valve_and_leave(coords)
        return

    # 돌림 중/돌림 완료/복귀 중에 멈춘 경우: 남은 부분만 마저 수행한다.
    if stage in (
        RecoveryStage.WATER_VALVE_TURNING,
        RecoveryStage.WATER_VALVE_TURNED,
        RecoveryStage.WATER_VALVE_RETURNING,
    ):
        # 돌리던 도중 멈췄으면 끝까지 돌린 자세로 마저 이동(남은 각도만).
        if stage == RecoveryStage.WATER_VALVE_TURNING:
            set_recovery_stage(RecoveryStage.WATER_VALVE_TURNING, "RESET 중 세척수 밸브 남은 돌림 마저 수행")
            if _valve_turn_target_posj is not None:
                safe_movej(_valve_turn_target_posj)
            else:
                g_node.get_logger().warning(
                    "밸브 끝각 관절 자세 기록이 없어 상대 이동으로 대체합니다(각도 어긋남 가능)."
                )
                safe_movel_relative(coords["wash_close"])
            set_recovery_stage(RecoveryStage.WATER_VALVE_TURNED, "RESET 중 세척수 밸브 돌림 완료")

        # 복귀: 밸브 시작(원위치) 자세로 마저 이동(돌림 완료/복귀 중 모두 동일).
        set_recovery_stage(RecoveryStage.WATER_VALVE_RETURNING, "RESET 중 세척수 밸브 복귀 동작 중")
        if _valve_turn_start_posj is not None:
            safe_movej(_valve_turn_start_posj)
        else:
            g_node.get_logger().warning(
                "밸브 시작 관절 자세 기록이 없어 상대 이동으로 대체합니다(각도 어긋남 가능)."
            )
            safe_movel_relative(coords["wash_open"])
        set_recovery_stage(RecoveryStage.WATER_VALVE_RETURNED, "RESET 중 세척수 밸브 복귀 완료")

        _release_valve_and_leave(coords)
        return

    # 그 외(밸브 잡기 전/직후, 돌리기 시작 안 함): 밸브 위치에서 처음부터 다시 조작한다.
    set_recovery_stage(RecoveryStage.WATER_VALVE_APPROACHING, "RESET 중 세척수 밸브 위치 이동 중")
    safe_movej(coords["wash_app_j"])
    set_recovery_stage(RecoveryStage.WATER_VALVE_READY, "RESET 중 세척수 밸브 위치 도착 - 파지 전")

    set_recovery_stage(RecoveryStage.WATER_VALVE_GRASPING, "RESET 중 세척수 밸브 파지 중")
    gripper_close()
    set_recovery_stage(RecoveryStage.WATER_VALVE_GRASPED, "RESET 중 세척수 밸브 파지 완료")

    # 여기부터는 밸브를 잡은 상태다. gripper_open은 돌림/복귀가 모두 끝난 뒤에만 수행한다.
    _valve_turn_start_posj = current_posj_or_none()
    set_recovery_stage(RecoveryStage.WATER_VALVE_TURNING, "RESET 중 세척수 밸브 돌림 동작 중")
    safe_movel_relative(coords["wash_close"])
    _valve_turn_target_posj = current_posj_or_none()
    set_recovery_stage(RecoveryStage.WATER_VALVE_TURNED, "RESET 중 세척수 밸브 돌림 완료")

    set_recovery_stage(RecoveryStage.WATER_VALVE_RETURNING, "RESET 중 세척수 밸브 복귀 동작 중")
    safe_movel_relative(coords["wash_open"])
    set_recovery_stage(RecoveryStage.WATER_VALVE_RETURNED, "RESET 중 세척수 밸브 복귀 완료")

    _release_valve_and_leave(coords)


def recover_bin_from_wash_jig():
    """세척 지그에 놓인 수거통을 다시 잡아 들어 올리고 경유점까지 이동한다."""
    coords = coordinates()
    status.set_state(ProcessState.WASHING, "복구: 세척 위치 수거통 재파지")
    # 밸브 위치 또는 기타 세척 주변 위치에서 바로 접근점으로 가지 않고 경유점을 먼저 거친다.
    safe_movej(coords["way_point_j"])
    safe_movej(coords["wash_approach_j"])
    safe_movel_relative(coords["wash_pick"])
    gripper_close()
    safe_wait(0.8)

    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "RESET 중 세척 위치 수거통 파지 불량")
        gripper_open()
        raise RuntimeError("RESET 중 세척 위치 수거통의 위치를 확인해 주세요")

    set_recovery_stage(RecoveryStage.WASH_BIN_GRASPED, "RESET 중 세척 위치 수거통 재파지 완료")
    safe_movel_relative(coords["wash_up"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_BIN_LIFTED, "RESET 중 세척 위치 수거통 들어올림 완료")
    safe_movej(coords["wash_approach_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_BIN_AT_APPROACH, "RESET 중 세척 위치 수거통 접근장소 도착")
    safe_movej(coords["way_point_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WASH_BIN_LEFT, "RESET 중 세척 위치 수거통 경유점 도착")


def move_wash_bin_to_waypoint_after_reset(stage: RecoveryStage):
    """세척 위치에서 통을 다시 잡은 뒤 멈춘 세부 단계에 맞춰 경유점까지 이어서 이동한다."""
    # 세척 위치에서 통을 다시 잡은 뒤 멈춘 위치에 맞춰 안전하게 경유점까지 빠져나온다.
    coords = coordinates()
    if stage == RecoveryStage.WASH_BIN_GRASPED:
        safe_movel_relative(coords["wash_up"], require_grasp=True)
        set_recovery_stage(RecoveryStage.WASH_BIN_LIFTED, "RESET 중 세척 위치 수거통 들어올림 완료")
        stage = RecoveryStage.WASH_BIN_LIFTED
    if stage == RecoveryStage.WASH_BIN_LIFTED:
        safe_movej(coords["wash_approach_j"], require_grasp=True)
        set_recovery_stage(RecoveryStage.WASH_BIN_AT_APPROACH, "RESET 중 세척 위치 수거통 접근장소 도착")
        stage = RecoveryStage.WASH_BIN_AT_APPROACH
    if stage == RecoveryStage.WASH_BIN_AT_APPROACH:
        safe_movej(coords["way_point_j"], require_grasp=True)
        set_recovery_stage(RecoveryStage.WASH_BIN_LEFT, "RESET 중 세척 위치 수거통 경유점 도착")


def dump_remaining_water_after_reset(start_from_water_area: bool = False):
    """세척수가 들어 있는 수거통을 세척수 배출 위치로 가져가 남은 물을 버린다."""
    coords = coordinates()
    status.set_state(ProcessState.WASHING, "복구: 세척수 배출")
    if not start_from_water_area:
        safe_movej(coords["way_point_j"], require_grasp=True)
    safe_movej(coords["water_out_approach_j"], require_grasp=True)
    set_recovery_stage(RecoveryStage.WATER_DUMPING, "RESET 중 세척수 배출 동작 진입")
    safe_movel_relative(coords["water_out_tilt"], require_grasp=True)
    run_periodic_water_shake()
    set_recovery_stage(RecoveryStage.WATER_DUMPED, "RESET 중 세척수 배출 완료")
    safe_movej(coords["water_out_approach_j"], require_grasp=True)


def recover_after_emergency_stop():
    """마지막 RecoveryStage와 실제 파지 여부를 기준으로 RESET 후동작 전체를 분기한다."""
    # RESET의 핵심 분기점. 마지막 RecoveryStage를 기준으로
    # 음식물 배출 여부, 세척수 주입 여부, 세척수 배출 여부에 맞는 후동작을 선택한다.
    stage = get_recovery_stage()
    g_node.get_logger().warning(f"RESET 복구 분기 시작: last_stage={stage.value}")
    _emergency_stop_requested.clear()

    if stage in (RecoveryStage.IDLE, RecoveryStage.READY, RecoveryStage.COMPLETE):
        if status.state == ProcessState.COLLISION and is_grasped():
            set_recovery_stage(RecoveryStage.BIN_GRASPED, "RESET 중 실제 파지 상태 확인")
            return_bin_to_home_after_reset("RESET: 충돌 후 수거통 파지 확인 - 수거통 원위치 후 초기 위치 복귀")
        else:
            reset_home_without_bin("RESET: 파지 중인 수거통 없음 - 초기 위치 복귀")

    elif stage == RecoveryStage.BIN_PICK_FAILED:
        # 처음 통을 잡으러 갔는데 통이 없던 경우: 통이 없으므로 파지 여부와 무관하게
        # 통 놓기 동작을 하지 않고, 내려가 있던 파지 위치에서 bin_pick_top으로 들어올린 뒤 홈으로 복귀한다.
        reset_bin_pick_failed_to_home("RESET: 수거통 미감지 - bin_pick_top 경유 후 홈 복귀")

    elif stage == RecoveryStage.BIN_PICKING:
        if is_grasped():
            return_bin_to_home_after_reset("RESET: 수거통 파지 확인 - 원위치 내려놓고 초기 위치 복귀")
        else:
            reset_home_without_bin("RESET: 수거통 미파지 - 경유점 경유 후 초기 위치 복귀")

    elif stage == RecoveryStage.BIN_RETURNED:
        reset_home_without_bin("RESET: 수거통 원위치 내려놓기 완료 - 초기 위치 복귀")

    elif stage in (
        RecoveryStage.DUMP_STARTED,
        RecoveryStage.DUMP_APPROACHED,
        RecoveryStage.DUMP_TILTING,
        RecoveryStage.DUMP_TILTED,
        RecoveryStage.DUMP_SHAKING,
    ):
        recover_dump_pose_to_waypoint_after_reset(stage)
        return_bin_to_home_after_reset("RESET: 음식물 배출 중 정지 - 수평 복구 후 수거통 원위치 복귀")

    elif not recovery_stage_at_least(stage, RecoveryStage.WASTE_DUMPED):
        if is_grasped():
            return_bin_to_home_after_reset("RESET: 음식물 미배출 - 수거통 원위치 후 초기 위치 복귀")
        else:
            reset_home_without_bin("RESET: 수거통 미파지 - 경유점 경유 후 초기 위치 복귀")

    elif not recovery_stage_at_least(stage, RecoveryStage.WATER_DUMPED):
        if stage in (RecoveryStage.WASTE_DUMPED, RecoveryStage.DUMP_LEVELING):
            recover_dump_pose_to_waypoint_after_reset(stage)
            execute_wash()
        # 배출 후 경유점으로 가던 중(또는 그 직전)에 멈춘 경우: 세척 지그 쪽이 아니라
        # 경유점으로만 이동한 뒤 세척 공정을 처음부터 다시 진행한다.
        elif stage in (RecoveryStage.DUMP_LEVELED, RecoveryStage.WASH_APPROACHING):
            safe_movej(coordinates()["way_point_j"], require_grasp=True)
            execute_wash()
        # 경유점을 지나 세척 진입장소로 가던 중/도착/안착 전이라면, 세척 지그 주변이므로
        # 먼저 진입장소를 거쳐 경유점까지 안전하게 빠져나온 뒤 세척 진입 순서를 다시 수행한다.
        elif stage in (
            RecoveryStage.WASH_WAYPOINT_TO_APPROACH,
            RecoveryStage.WASH_APPROACH_REACHED,
            RecoveryStage.WASH_PLACING,
        ):
            retract_wash_area_to_waypoint(
                True,
                "RESET: 세척 위치 안착 전 정지 - 세척 진입장소 경유 후 재진입",
            )
            execute_wash()
        elif not recovery_stage_at_least(stage, RecoveryStage.WASH_JIG_PLACED):
            status.set_state(ProcessState.WASHING, "RESET: 음식물 배출 완료 - 세척/배수 후 복귀")
            execute_wash()
        else:
            if stage in (
                RecoveryStage.WASH_JIG_PLACED,
                RecoveryStage.WASH_JIG_RELEASED,
                RecoveryStage.WASH_JIG_LEAVING,
                RecoveryStage.WASH_JIG_LEFT,
                RecoveryStage.WASH_TO_WAYPOINT,
                RecoveryStage.WASH_WAYPOINT_REACHED,
                RecoveryStage.WATER_VALVE_APPROACHING,
                RecoveryStage.WATER_VALVE_READY,
                RecoveryStage.WATER_VALVE_GRASPING,
                RecoveryStage.WATER_VALVE_GRASPED,
                RecoveryStage.WATER_VALVE_TURNING,
                RecoveryStage.WATER_VALVE_TURNED,
                RecoveryStage.WATER_VALVE_RETURNING,
                RecoveryStage.WATER_VALVE_RETURNED,
            ):
                operate_water_valve_after_reset(stage)
                recover_bin_from_wash_jig()
            elif stage == RecoveryStage.WATER_IN_BIN:
                recover_bin_from_wash_jig()
            elif stage in (
                RecoveryStage.WASH_BIN_GRASPED,
                RecoveryStage.WASH_BIN_LIFTED,
                RecoveryStage.WASH_BIN_AT_APPROACH,
            ):
                move_wash_bin_to_waypoint_after_reset(stage)
            dump_remaining_water_after_reset(start_from_water_area=(stage == RecoveryStage.WATER_DUMPING))
        return_bin_to_home_after_reset("RESET: 세척수 배출 완료 - 수거통 원위치 후 초기 위치 복귀")

    else:
        return_bin_to_home_after_reset("RESET: 세척수 배출 완료 상태 - 수거통 원위치 후 초기 위치 복귀")

    # [branch_motion.py 대비 수정] RESET 종료를 무조건 IDLE로 보내던 동작을 공정 의미에 맞게 분리
    # 배출 이후 남은 세척·배수·복귀까지 수행했으면 COMPLETE/작업 완료로 알리고,
    # 배출 전 안전 복귀만 수행한 경우에만 IDLE/원위치 복귀 완료로 알림
    if recovery_stage_at_least(stage, RecoveryStage.WASTE_DUMPED):
        set_recovery_stage(RecoveryStage.COMPLETE, "중단된 후속 공정 복구 완료")
        status.set_state(ProcessState.COMPLETE, "작업 완료")
    else:
        set_recovery_stage(RecoveryStage.IDLE, "RESET 원위치 복귀 완료")
        status.set_state(ProcessState.IDLE, "원위치 복귀 완료")


# 배출/세척 전체 공정을 1~5단계 순서대로 실행하는 최상위 공정 함수
# EmergencyStopError 발생 시 즉시 종료, 그 외 예외 시 모션 정지 후 ERROR 상태로 전환
def run_process(mode: int):
    """START 명령으로 실행되는 전체 음식물 배출/세척 공정의 최상위 순서 함수이다."""
    try:
        check_system_ready() # 1단계: 준비 및 초기화 (check_system_ready)
        pick_bin() # 2단계: 수거통 집어 들기 (pick_bin)
        run_dump_motion(mode) # 3단계: 모드별 배출 및 털기 (run_dump_motion)
        execute_wash() # 4단계: 세척 및 오수 배출 (execute_wash)
        return_bin_and_complete() # 5단계: 원위치 복귀 및 종료 (return_bin_and_complete)
        return True, "배출 및 세척 완료"
    except EmergencyStopError as exc:
        if status.state != ProcessState.COLLISION:
            status.set_state(ProcessState.ERROR, str(exc))
        return False, str(exc)
    except Exception as exc:
        status.set_state(ProcessState.ERROR, str(exc))
        try:
            request_motion_stop(str(exc))
        except Exception:
            pass
        return False, str(exc)


def retry_pick_and_continue(mode: int):
    """최초 파지 실패 위치에서 수거통을 재파지하고 남은 공정을 이어서 수행한다."""
    if get_recovery_stage() != RecoveryStage.BIN_PICK_FAILED:
        raise RuntimeError(f"재파지를 수행할 수 없는 단계입니다: {get_recovery_stage().value}")
    status.set_state(ProcessState.MOVING, "수거통 재파지 중")
    gripper_close()
    safe_wait(0.8)
    if not is_grasped():
        status.publish_safety(ErrorCode.ERR_PICK, "수거통 재파지 실패 - 배치 상태를 다시 확인해 주세요")
        set_recovery_stage(RecoveryStage.BIN_PICK_FAILED, "수거통 재파지 실패")
        gripper_open()
        raise RuntimeError("수거통의 위치를 다시 확인해 주세요")
    set_recovery_stage(RecoveryStage.BIN_GRASPED, "수거통 재파지 확인 완료")
    safe_movel_relative(coordinates()["bin_pick_top"], require_grasp=True)
    run_dump_motion(mode)
    execute_wash()
    return_bin_and_complete()
    return True, "수거통 재파지 후 배출 및 세척 완료"

# 공정 실행 중복 방지 락: acquire 상태에서 새 START 명령이 들어오면 즉시 거부(blocking=False)
_process_lock = threading.Lock()


def _run_process_worker(task_id, mode_id):
    """START 콜백에서 하던 run_process 본문을 워커 스레드에서 실행한다.

    콜백은 즉시 반환시켜 EMERGENCY_STOP/RESET 명령을 계속 받을 수 있게 하고,
    기존 START 코드의 결과 로그와 process lock 해제 흐름은 여기에서 유지한다.
    """
    try:
        ok, result_message = run_process(mode_id)
        if ok:
            g_node.get_logger().info(f"task_id={task_id}: {result_message}")
        elif _emergency_stop_requested.is_set():
            g_node.get_logger().info(f"task_id={task_id}: 안전정지 처리 완료 - {result_message}")
        else:
            g_node.get_logger().error(f"task_id={task_id}: {result_message}")
    finally:
        _process_lock.release()

def _run_retry_pick_worker(mode_id):
    try:
        ok, message = retry_pick_and_continue(mode_id)
        (g_node.get_logger().info if ok else g_node.get_logger().error)(message)
    except Exception as exc:
        status.set_state(ProcessState.ERROR, str(exc))
        g_node.get_logger().error(f"수거통 재파지 처리 실패: {exc}")
    finally:
        _process_lock.release()


# ==============================================================================
# [Command topic entry]
# ==============================================================================


def handle_emergency_stop(source: str = "/robot/command"):
    """EMERGENCY_STOP 공통 처리. 여러 콜백에서 동시에 들어와도 정지 요청은 직렬화한다."""
    with _emergency_command_lock:
        g_node.get_logger().warning(f"비상정지 명령 수신({source}) — 즉시 정지 수행")
        try:
            trigger_safety_stop(
                ErrorCode.ERR_SYSTEM,
                "사용자 비상정지 요청",
                ProcessState.ERROR,
            )
        except Exception as exc:
            g_node.get_logger().error(f"비상정지 처리 중 예외 발생: {exc}")


def handle_emergency_stop_priority(msg: String):
    """같은 /robot/command 토픽에서 EMERGENCY_STOP만 별도 콜백으로 먼저 처리한다."""
    # 일반 명령 콜백이 바쁘더라도 ReentrantCallbackGroup에서 비상정지만 별도로 잡기 위한 구독이다.
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

        _emergency_stop_requested.clear()
        g_node.get_logger().info(f"공정 시작 명령 수신: task_id={task_id}, mode_id={mode_id}")

        try:
            process_thread = threading.Thread(
                target=_run_process_worker,
                args=(task_id, mode_id),
                daemon=True,
                name=f"process-{task_id}",
            )
            process_thread.start()
        except Exception:
            # 워커 시작 실패 시 다음 START가 영구 차단되지 않도록 락을 되돌린다.
            _process_lock.release()
            raise
        return

    elif cmd_type == "RETRY_PICK":
        mode_id = command.get("mode_id")
        if isinstance(mode_id, bool) or mode_id not in (DUMP_MODE_NORMAL, DUMP_MODE_STRONG):
            g_node.get_logger().error(f"RETRY_PICK mode_id 오류: {mode_id!r}")
            return
        if get_recovery_stage() != RecoveryStage.BIN_PICK_FAILED:
            g_node.get_logger().warning(f"RETRY_PICK 무시 - 현재 단계={get_recovery_stage().value}")
            return
        if not _process_lock.acquire(blocking=False):
            g_node.get_logger().warning("이미 공정이 진행 중입니다. RETRY_PICK 무시.")
            return
        _emergency_stop_requested.clear()
        try:
            threading.Thread(target=_run_retry_pick_worker, args=(mode_id,), daemon=True, name="retry-bin-pick").start()
        except Exception:
            _process_lock.release()
            raise
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
                float(payload.get("J3", 90.0)),
                float(payload.get("J4", 0.0)),
                float(payload.get("J5", 90.0)),
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
        handle_emergency_stop("/robot/command")
        return

    # --------------------------------------------------------------------------
    # 케이스 3.6: [RESET] 비상정지/에러 후 안전 경유점을 거쳐 초기 위치로 복귀
    # --------------------------------------------------------------------------
    elif cmd_type == "RESET":
        if _process_lock.locked():
            g_node.get_logger().warning("공정 진행 중 - RESET 명령 무시됨")
            return

        g_node.get_logger().info("RESET 명령 수신 — 마지막 체크포인트 기준 복구 수행")
        try:
            # 컨트롤러 자체 안전정지 상태면 프로그램 복구 모션을 보내지 않는다.
            if block_reset_if_controller_safety_stop():
                return
            recover_after_emergency_stop()
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

def main(args=None):
    """ROS 노드, DSR API, 구독 콜백, executor를 초기화하고 명령 수신을 시작한다."""
    global g_node, status, dsr_node, stop_node

    rclpy.init(args=args)

    # ---- 노드 1: DSR API 전용 노드 (모션/그리퍼 제어) ----
    dsr_node = rclpy.create_node("food_waste_dump_robot_dsr", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node   # DSR API가 내부적으로 쓰는 노드는 이쪽 하나만

    # DSR_ROBOT2 API와 그리퍼 서비스 호출부가 spin_until_future_complete(dsr_node, ...)를
    # 직접 수행하므로 dsr_node를 별도 executor에서 상시 spin하지 않는다.
    # 정지 서비스는 DSR API spin과 충돌하지 않도록 별도 노드에서 호출한다.
    stop_node = rclpy.create_node("food_waste_dump_robot_stop", namespace=ROBOT_ID)

    # ---- 노드 2: 토픽 구독/명령 처리 전용 노드 (메인 스레드) ----
    node = rclpy.create_node("food_waste_dump_robot", namespace=ROBOT_ID)
    g_node = node

    node.declare_parameter("dump_mode", DUMP_MODE_NORMAL)
    node.declare_parameter("autostart", False)

    status = StatusBus(node)
    init_robot_api()
    init_gripper_api()

    # 일반 명령과 비상정지 전용 콜백을 분리한다.
    # 같은 /robot/command 토픽이라도 EMERGENCY_STOP은 별도 Reentrant 그룹에서 먼저 처리될 수 있다.
    process_callback_group = MutuallyExclusiveCallbackGroup()
    emergency_callback_group = ReentrantCallbackGroup()

    node.create_subscription(
        String, "/robot/command", handle_emergency_stop_priority, 10,
        callback_group=emergency_callback_group
    )
    node.create_subscription(
        String, "/robot/command", handle_robot_command, 10,
        callback_group=process_callback_group
    )

    status.set_state(ProcessState.IDLE, "작업 대기")

    if node.get_parameter("autostart").value:
        mode = int(node.get_parameter("dump_mode").value)
        run_process(mode)

    # 멀티스레드 executor로 spin -> START가 한 스레드를 차지해도 다른 스레드가 비상정지 처리 가능
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()   # 메인 스레드는 명령 노드만 spin
    finally:
        try:
            gripper_open()
        except Exception:
            pass
        node.destroy_node()
        dsr_node.destroy_node()
        stop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
