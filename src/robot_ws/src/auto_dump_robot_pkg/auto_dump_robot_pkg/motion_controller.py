import rclpy
import DR_init
from dsr_msgs2.srv import MoveStop
from onrobot_rg_msgs.srv import SetCommand

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITYX, ACCX = 30, 30
VELOCITYJ, ACCJ = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==============================================================================
# [배출 작업 파라미터]
# ==============================================================================
DUMP_ANGLE = 60.0
SHAKE_AMP = 8.0
SHAKE_REPEAT = 10

FORCE_MOVE_TH = 35.0
FORCE_SHAKE_TH = 45.0
FORCE_BIN_DETECT_TH = 5.0
FORCE_GRASP_DELTA_TH = 3.0

Z_SEARCH_STEP = 5.0
Z_SEARCH_MAX = 60.0

dsr = None
posx = None
posj = None
g_node = None
gripper_client = None


# ==============================================================================
# [초기화]
# ==============================================================================
def init_robot_api():
    global dsr, posx, posj

    import DSR_ROBOT2 as dsr_module
    from DR_common2 import posx as posx_class, posj as posj_class

    dsr = dsr_module
    posx = posx_class
    posj = posj_class

    required_services = [
        dsr._ros2_set_current_tool,
        dsr._ros2_set_current_tcp,
        dsr._ros2_set_singularity_handling,
        dsr._ros2_movej,
        dsr._ros2_movel,
        dsr._ros2_move_periodic,
        dsr._ros2_get_tool_force,
        dsr._ros2_get_current_posx,
        dsr._ros2_get_current_posj,
    ]

    g_node.get_logger().info("Waiting for DSR services...")
    for client in required_services:
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"DSR service unavailable: {client.srv_name}")

    dsr.set_tool("Tool Weight_2FG")
    dsr.set_tcp("2FG_TCP")
    dsr.set_singularity_handling(dsr.DR_AVOID)

    g_node.get_logger().info("Robot API ready")


def init_gripper_api():
    global gripper_client

    gripper_client = g_node.create_client(SetCommand, "/onrobot/sendCommand")

    if not gripper_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("RG2 service unavailable: /onrobot/sendCommand")

    g_node.get_logger().info("RG2 service ready")


# ==============================================================================
# [좌표 설정]
# ==============================================================================
def coordinate():
    P_WAIT = posj(9.46, 1.33, 103.26, -0.75, 74.64, 2.23)

    # 컵 파지 위치
    P_PICK_APPROACH = posx(423.65, -3.58, 330.00, 97.76, -178.69, 95.48)
    P_PICK_SEARCH_START = posx(424.74, -4.82, 290.00, 80.35, -178.76, 78.04)
    P_PICK = posx(424.74, -4.82, 266.78, 80.35, -178.76, 78.04)

    # 미니어처 대형 배출통 위치
    P_DUMP_APPROACH = posx(277.21, 147.71, 330.0, 103.54, -178.79, 101.07)
    P_DUMP = posx(277.77, 147.23, 299.95, 89.86, -178.71, 87.42)

    return {
        "P_WAIT": P_WAIT,
        "P_PICK_APPROACH": P_PICK_APPROACH,
        "P_PICK_SEARCH_START": P_PICK_SEARCH_START,
        "P_PICK": P_PICK,
        "P_DUMP_APPROACH": P_DUMP_APPROACH,
        "P_DUMP": P_DUMP,
    }


# ==============================================================================
# [그리퍼]
# ==============================================================================
def send_gripper_command(command):
    req = SetCommand.Request()
    req.command = command

    future = gripper_client.call_async(req)
    rclpy.spin_until_future_complete(g_node, future, timeout_sec=10.0)

    if not future.done():
        raise RuntimeError(f"RG2 command timeout: {command}")

    result = future.result()
    if result is None or not result.success:
        msg = result.message if result else "no response"
        raise RuntimeError(f"RG2 command failed: {command}, {msg}")


def gripper_open():
    send_gripper_command("o")
    dsr.wait(0.3)


def gripper_close():
    send_gripper_command("c")
    dsr.wait(0.5)


# ==============================================================================
# [안전 / 힘 감지]
# ==============================================================================
def stop(mode=1):
    client = g_node.create_client(
        MoveStop,
        f"/{ROBOT_ID}/dsr_controller2/motion/move_stop"
    )

    if not client.wait_for_service(timeout_sec=1.0):
        g_node.get_logger().error("MoveStop service unavailable")
        return -1

    req = MoveStop.Request()
    req.stop_mode = int(mode)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(g_node, future, timeout_sec=2.0)

    result = future.result()
    return 0 if result and result.success else -1


def get_force_norm():
    force = dsr.get_tool_force(dsr.DR_BASE)
    return (force[0] ** 2 + force[1] ** 2 + force[2] ** 2) ** 0.5


def check_external_force(limit):
    f_norm = get_force_norm()

    if f_norm > limit:
        g_node.get_logger().error(f"External force detected: {f_norm:.2f} N")
        stop(dsr.DR_QSTOP)
        return False

    return True


# ==============================================================================
# [예외 상황 1: 통 없음]
# ==============================================================================
def check_bin_exists_by_z():
    p = coordinate()
    search_pose = posx(p["P_PICK_SEARCH_START"])

    g_node.get_logger().info("Checking bin existence by Z search")

    moved_depth = 0.0

    while moved_depth < Z_SEARCH_MAX:
        search_pose[2] -= Z_SEARCH_STEP
        dsr.movel(search_pose, vel=15, acc=30)

        if get_force_norm() > FORCE_BIN_DETECT_TH:
            g_node.get_logger().info("Bin detected")
            return True

        moved_depth += Z_SEARCH_STEP

    g_node.get_logger().error("Bin not detected")
    return False


# ==============================================================================
# [예외 상황 2: 파지 실패]
# ==============================================================================
def check_grasp_success():
    p = coordinate()

    before = get_force_norm()

    lift_pose = posx(p["P_PICK"])
    lift_pose[2] += 30.0
    dsr.movel(lift_pose, vel=20, acc=30)

    after = get_force_norm()
    delta = abs(after - before)

    if delta < FORCE_GRASP_DELTA_TH:
        g_node.get_logger().error(f"Grasp failed. force delta={delta:.2f}")
        return False

    g_node.get_logger().info(f"Grasp success. force delta={delta:.2f}")
    return True


# ==============================================================================
# [0~1. 대기 위치 → 파지 위치 → 파지]
# ==============================================================================
def pick_bin():
    p = coordinate()

    g_node.get_logger().info("[STEP 0] Move to pick position")

    dsr.movej(p["P_WAIT"], vel=VELOCITYJ, acc=ACCJ)
    gripper_open()

    dsr.movel(p["P_PICK_APPROACH"], vel=VELOCITYX, acc=ACCX)

    if not check_bin_exists_by_z():
        stop(dsr.DR_QSTOP)
        return False

    g_node.get_logger().info("[STEP 1] Grasp bin")

    dsr.movel(p["P_PICK"], vel=20, acc=30)
    gripper_close()

    if not check_grasp_success():
        gripper_open()
        dsr.movej(p["P_WAIT"], vel=VELOCITYJ, acc=ACCJ)
        return False

    dsr.movel(p["P_PICK_APPROACH"], vel=VELOCITYX, acc=ACCX)
    return True


# ==============================================================================
# [2. 배출 위치로 이동]
# ==============================================================================
def move_to_dump_position():
    p = coordinate()

    g_node.get_logger().info("[STEP 2] Move to dump position")

    if not check_external_force(FORCE_MOVE_TH):
        return False

    # 먼 거리: movej
    dsr.movej(p["P_WAIT"], vel=VELOCITYJ, acc=ACCJ)

    if not check_external_force(FORCE_MOVE_TH):
        return False

    # 짧은 거리: movel
    dsr.movel(p["P_DUMP_APPROACH"], vel=VELOCITYX, acc=ACCX)

    if not check_external_force(FORCE_MOVE_TH):
        return False

    dsr.movel(p["P_DUMP"], vel=VELOCITYX, acc=ACCX)

    return check_external_force(FORCE_MOVE_TH)


# ==============================================================================
# [3. 음식물 배출 - 통 기울이기]
# ==============================================================================
def tilt_bin(dump_angle):
    p = coordinate()

    g_node.get_logger().info("[STEP 3] Tilt bin")

    dump_pose = posx(p["P_DUMP"])

    # 현재 TCP 자세 기준에 따라 A/B/C 중 어느 축을 돌릴지 실측 필요
    # 여기서는 B축 회전으로 가정
    dump_pose[4] += dump_angle

    dsr.movel(dump_pose, vel=20, acc=30)
    dsr.mwait()

    return True


# ==============================================================================
# [4. 잔여 음식물 배출 - 흔들기]
# ==============================================================================
def shake_bin(shake_amp, repeat):
    g_node.get_logger().info("[STEP 4] Shake bin")

    for i in range(repeat):
        if not check_external_force(FORCE_SHAKE_TH):
            g_node.get_logger().error("Slip or abnormal force during shaking")
            return False

        dsr.move_periodic(
            amp=[0, 0, 0, 0, shake_amp, 0],
            period=[0, 0, 0, 0, 0.4, 0],
            atime=0.1,
            repeat=1,
            ref=dsr.DR_BASE
        )
        dsr.mwait()

    return True


# ==============================================================================
# [5. 통 원위치 복귀]
# ==============================================================================
def return_bin_to_origin():
    p = coordinate()

    g_node.get_logger().info("[STEP 5] Return bin to origin")

    # 기울임 자세에서 배출 위치 자세로 복귀
    dsr.movel(p["P_DUMP"], vel=20, acc=30)
    dsr.movel(p["P_DUMP_APPROACH"], vel=VELOCITYX, acc=ACCX)

    if not check_external_force(FORCE_MOVE_TH):
        return False

    # 먼 거리: movej
    dsr.movej(p["P_WAIT"], vel=VELOCITYJ, acc=ACCJ)

    if not check_external_force(FORCE_MOVE_TH):
        return False

    # 짧은 거리: movel
    dsr.movel(p["P_PICK_APPROACH"], vel=VELOCITYX, acc=ACCX)
    dsr.movel(p["P_PICK"], vel=20, acc=30)

    gripper_open()

    retreat = posx(p["P_PICK"])
    retreat[2] += 80.0
    dsr.movel(retreat, vel=VELOCITYX, acc=ACCX)

    dsr.movej(p["P_WAIT"], vel=VELOCITYJ, acc=ACCJ)

    return True


# ==============================================================================
# [6. 작업 완료]
# ==============================================================================
def finish_task(success):
    if success:
        g_node.get_logger().info("[STEP 6] Task complete. System standby.")
    else:
        g_node.get_logger().error("[STEP 6] Task failed. System stopped or returned safely.")


# ==============================================================================
# [전체 흐름]
# ==============================================================================
def main_task():
    success = False

    try:
        g_node.get_logger().info("Food waste dumping task start")

        if not pick_bin():
            return False

        if not move_to_dump_position():
            return False

        if not tilt_bin(DUMP_ANGLE):
            return False

        if not shake_bin(SHAKE_AMP, SHAKE_REPEAT):
            return False

        if not return_bin_to_origin():
            return False

        success = True
        return True

    except Exception as e:
        g_node.get_logger().error(f"Task exception: {e}")
        stop(dsr.DR_QSTOP)
        return False

    finally:
        finish_task(success)


# ==============================================================================
# [메인]
# ==============================================================================
def main(args=None):
    global g_node

    rclpy.init(args=args)

    node = rclpy.create_node("food_waste_dump", namespace=ROBOT_ID)
    g_node = node
    DR_init.__dsr__node = node

    init_robot_api()
    init_gripper_api()

    main_task()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()