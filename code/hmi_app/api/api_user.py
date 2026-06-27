from .api_base import BaseAPI


class UserAPI(BaseAPI):

    # 작업 시작
    # 백엔드: POST /api/dump/start  → TaskStartRequest { mode_id: int }
    # 응답:   { task_id: str }
    def request_task_start(self, mode_id):
        return self._post(
            "/api/dump/start",
            {"mode_id": mode_id},
            timeout=10
        )

    def reset_robot_system(self):
        """긴급정지 후 로봇의 체크포인트 기반 복구 동작을 요청합니다."""
        return self._post("/api/robot/reset", {})

    # 에러 로그 전송
    # 백엔드: POST /api/error/log  → ErrorLogRequest { task_id, error_code, error_msg }
    # ★ 수정: 기존 "message" 키 → 백엔드 스펙에 맞게 "error_msg" 로 변경
    def send_error_log(self, task_id, error_code, message):
        return self._post(
            "/api/error/log",
            {
                "task_id": task_id,
                "error_code": error_code,
                "error_msg": message   # ← 수정: "message" → "error_msg"
            }
        )

    # 로봇 관절 이동 (수평 이동 버튼 등에서 사용)
    # 백엔드: POST /api/robot/move-joint  → MoveJointRequest { J1~J6: float }
    def send_hardware_command(self, joint_data):
        return self._post(
            "/api/robot/move-joint",
            joint_data
        )

    # 그리퍼 제어
    # 백엔드 MoveJointRequest 스펙(J1~J6)에 맞춰 J6에 플래그를 매핑해 우회 전송
    # ★ 수정: action_name 비교를 "in" 조건으로 통일 (AdminAPI와 동일)
    def send_gripper_command(self, action_name, base_angle):
        gripper_flag = 1.0 if "OPEN" in action_name else 2.0

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
    