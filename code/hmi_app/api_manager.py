from api_base import BaseAPI


class AdminAPI(BaseAPI):

    # 에러 기록
    def send_error_log(
        self,
        task_id,
        code,
        message
    ):
        return self._post(
            "/api/error/log",
            {
                "task_id": task_id,
                "error_code": code,
                "error_msg": message
            }
        )

    # 비상정지
    def send_emergency_stop(
        self,
        task_id=None
    ):
        return self._post(
            "/api/robot/emergency-stop",
            {
                "task_id": task_id
            }
        )

    # 시스템 리셋
    def reset_robot_system(
        self
    ):
        return self._post(
            "/api/robot/reset"
        )