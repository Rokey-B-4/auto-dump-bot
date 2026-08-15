"""공정 실행/RESET 복구 시퀀스.

motion_runtime.py의 하드웨어/상태 헬퍼를 사용해 실제 작업 순서만 담당한다.
"""

import threading

from . import motion_runtime as rt
from .motion_runtime import (
    DUMP_MODE_NORMAL,
    DUMP_MODE_STRONG,
    DUMP_NORMAL_PERIOD,
    DUMP_NORMAL_PERIODIC_AMP,
    DUMP_STRONG_PERIOD,
    DUMP_STRONG_PERIODIC_AMP,
    ErrorCode,
    EmergencyStopError,
    PERIODIC_SHAKE_ATIME,
    ProcessState,
    RecoveryStage,
    SHAKE_REPEAT_COUNT,
    WATER_PERIOD,
    WATER_PERIODIC_AMP,
    _emergency_stop_requested,
    apply_wash_place_force,
    coordinates,
    current_posj_or_none,
    current_posx_base,
    get_recovery_stage,
    gripper_close,
    gripper_open,
    is_grasped,
    recovery_stage_at_least,
    request_motion_stop,
    safe_move_periodic,
    safe_movej,
    safe_movel,
    safe_movel_relative,
    safe_wait,
    set_recovery_stage,
    check_system_ready
)

# 아래 세 참조는 main()에서 런타임 초기화가 끝난 뒤 sync_runtime_refs()로 갱신한다.
_ds = None
g_node = None
status = None
_dump_level_pose = None
_valve_turn_start_posj = None
_valve_turn_target_posj = None


def sync_runtime_refs():
    """런타임 모듈에서 초기화된 노드/API/status 참조를 공정 모듈에 연결한다."""
    global _ds, g_node, status
    _ds = rt._ds
    g_node = rt.g_node
    status = rt.status


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
    # 일정한 힘(X축 30N, Z축 10N)으로 부드럽게 밀어 넣습니다.
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
