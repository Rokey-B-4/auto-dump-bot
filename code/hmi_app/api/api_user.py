from api.api_base import BaseAPI


class UserAPI(BaseAPI):

    # 작업 시작
    def request_task_start(
        self,
        mode_id
    ):
        return self._post(
            "/api/dump/start",
            {"mode_id": mode_id},
            timeout=10
        )

    # 에러 로그 전송 메서드 누락 대응 (FoodWasteGUI에서 호출하고 있으므로 추가)
    def send_error_log(self, task_id, error_code, message):
        return self._post(
            "/api/error/log",
            {
                "task_id": task_id,
                "error_code": error_code,
                "message": message
            }
        )
    
    # 로봇 관절 이동
    def send_hardware_command(
        self,
        joint_data
    ):
        return self._post(
            "/api/robot/move-joint",
            joint_data
        )
    
    # 그리퍼 제어 기능 추가
    def send_gripper_command(self, action_name, base_angle):
        # 백엔드 MoveJointRequest 스펙(J1~J6)에 맞춤
        gripper_flag = 1.0 if action_name == "OPEN" else 2.0

        gripper_packet = {
            "J1": float(base_angle),
            "J2": 0.0,
            "J3": 0.0,
            "J4": 0.0,
            "J5": 0.0,
            "J6": gripper_flag  # 백엔드에서 이 값을 보고 OPEN/CLOSE 분기함
        }

        return self._post(
            "/api/robot/move-joint",
            gripper_packet
        )