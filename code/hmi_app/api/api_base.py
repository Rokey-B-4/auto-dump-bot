import requests
import threading
import json
import asyncio
import websockets
import time


class BaseAPI:
    def __init__(
        self,
        base_url="http://localhost:8000",
        ws_url="ws://localhost:8000/ws/robot/status"
    ):
        self.base_url = base_url
        self.ws_url = ws_url

    def _post(
        self,
        endpoint,
        data=None,
        timeout=2
    ):
        try:
            return requests.post(
                f"{self.base_url}{endpoint}",
                json=data,
                timeout=timeout
            )
        except requests.RequestException:
            return None

    # websocket listener
    def start_websocket_listener(
        self,
        callback
    ):
        def run_loop():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            while True:
                try:
                    loop.run_until_complete(
                        self._listen(callback)
                    )
                except Exception as e:
                    print(
                        f"WebSocket reconnect: {e}"
                    )
                    time.sleep(3)

        threading.Thread(
            target=run_loop,
            daemon=True
        ).start()

    async def _listen(
        self,
        callback
    ):
        async with websockets.connect(
            self.ws_url
        ) as ws:

            while True:
                data = await ws.recv()
                callback(
                    json.loads(data)
                )