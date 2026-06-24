import requests
import threading
import json
import asyncio
import websockets
import time

class APIService:
    def __init__(self, base_url="http://localhost:8000", ws_url="ws://localhost:8000/ws/robot/status"):
        self.base_url = base_url
        self.ws_url = ws_url

    def request_task_start(self, mode_id):
        try:
            res = requests.post(f"{self.base_url}/api/dump/start", json={"mode_id": mode_id}, timeout=3)
            return res.json() if res.status_code == 200 else None
        except: return None

    def send_error_log(self, task_id, code, message):
        try:
            requests.post(f"{self.base_url}/api/error/log", 
                          json={"task_id": task_id, "error_code": code, "error_msg": message}, 
                          timeout=2)
        except: pass

    # [핵심] 이 메서드가 없어서 에러가 발생했습니다
    def send_emergency_stop(self, task_id=None):
        try: 
            return requests.post(f"{self.base_url}/api/robot/emergency-stop", json={"task_id": task_id}, timeout=2)
        except: pass

    def send_hardware_command(self, joint_data):
        try: return requests.post(f"{self.base_url}/api/robot/move-joint", json=joint_data, timeout=2)
        except: return None

    # [핵심] 이 메서드도 추가하세요
    def reset_robot_system(self):
        try: return requests.post(f"{self.base_url}/api/robot/reset", timeout=2)
        except: pass

    def start_websocket_listener(self, callback):
        def run_loop():
            while True:
                try:
                    asyncio.run(self._listen(callback))
                except Exception as e:
                    time.sleep(3)
        threading.Thread(target=run_loop, daemon=True).start()

    async def _listen(self, callback):
        async with websockets.connect(self.ws_url) as ws:
            while True:
                data = await ws.recv()
                callback(json.loads(data))