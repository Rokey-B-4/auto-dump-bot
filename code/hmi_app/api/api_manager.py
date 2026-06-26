from .api_base import BaseAPI

class AdminAPI(BaseAPI):

    # 🟢 [POST] /api/error/log (완)
    def send_error_log(self, task_id, code, message):
        """시스템 런타임 중 발생한 에러 이력을 데이터베이스에 기록"""
        return self._post(
            "/api/error/log",
            {"task_id": task_id, "error_code": code, "error_msg": message},
        )

    # 🟢 [POST] /api/robot/emergency-stop (완)
    def send_emergency_stop(self, task_id=None):
        """로봇 동작을 즉시 중단하고 시스템 비상 인터록을 가동"""
        return self._post("/api/robot/emergency-stop", {"task_id": task_id})

    # 🟢 [POST] /api/robot/reset (완)
    def reset_robot_system(self):
        """비상 정지 및 에러 인터록 상황을 해제하고 시스템을 초기 상태로 복구"""
        return self._post("/api/robot/reset", {})  # Request Body: 없음 ({})

    # 🟢 [POST] /api/robot/move-joint (완)
    def send_hardware_command(self, joint_angles):
        """협동로봇의 6축 관절 각도를 수동으로 제어(MoveJ)"""
        return self._post("/api/robot/move-joint", joint_angles)

    # 🔵 [POST] /api/robot/move-joint (★그리퍼 제어 우회 연동 기능 추가)
    def send_gripper_command(self, action_name, base_angle=0.0):
        """
        그리퍼 열기/닫기 명령을 전송합니다.
        현재 백엔드 move-joint의 Pydantic 검증 모델(J1~J6)을 통과하기 위해
        그리퍼 전용 커스텀 패킷 규격을 6축 배열 데이터 형태로 맵핑하여 우회 전송합니다.
        """
        # 백엔드 MoveJointRequest가 J1~J6 실수형태를 필수로 요구하므로,
        # J1에는 베이스 각도를 넣고, J6에 그리퍼 액션 플래그(예: 열기=1.0, 닫기=2.0)를 매핑하여 전송합니다.
        print(f"### DEBUG send_gripper_command 호출됨: action_name={action_name!r}", flush=True)
        gripper_flag = 1.0 if "OPEN" in action_name else 2.0
        print(f"### DEBUG gripper_flag 계산됨: {gripper_flag}", flush=True)
        
        gripper_packet = {
            "J1": float(base_angle),
            "J2": 0.0,
            "J3": 0.0,
            "J4": 0.0,
            "J5": 0.0,
            "J6": gripper_flag  # 로봇 제어 노드(ROS) 단에서 J6의 숫자를 보고 그리퍼 작동 판단
        }
        print(f"### DEBUG 전송 패킷: {gripper_packet}", flush=True)
        return self._post("/api/robot/move-joint", gripper_packet)