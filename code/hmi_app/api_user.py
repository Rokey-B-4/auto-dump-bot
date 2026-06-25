from api_base import BaseAPI


class UserAPI(BaseAPI):

    # 작업 시작
    def request_task_start(
        self,
        mode_id
    ):
        return self._post(
            "/api/dump/start",
            {"mode_id": mode_id},
            timeout=3
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