import asyncio
import websockets
# FastAPI가 서버용 웹소켓이라면, 이 패키지는 파이썬으로 웹소켓 '클라이언트'를 만들 때 사용하는 표준 라이브러리
import json

# 백엔드의 웹소켓 중계 기능이 진짜 잘 작동하는지 검증하기 위한 가짜(Mock) 클라이언트

async def receive_robot_status():
    uri = "ws://localhost:8000/ws/robot/status"
    # uri: main.py에서 파놓은 웹소켓 엔드포인트 주소(ws://...)로 타겟을 지정
    print(f"백엔드 웹소켓 서버({uri})에 연결을 시도합니다...")
    
    try:
        async with websockets.connect(uri) as websocket:
            # 백엔드 서버에 지속적인 통신 연결(Handshake)을 수립
            print("연결 성공! 로봇 토픽 신호를 기다리는 중...\n")
            while True:
                data = await websocket.recv()
                # 백엔드의 ConnectionManager.broadcast()가 데이터를 밀어줄 때까지 이 라인에서 대기(await)
                result = json.loads(data)
                # 백엔드가 직렬화해서 보낸 JSON 문자열을 파이썬 딕셔너리 객체로 다시 파싱(역직렬화)하여 화면에 예쁘게 출력
                
                print(f"로봇 상태 수신: {result.get('status')}")
                print("-" * 40)
    except Exception as e:
        print(f"연결 끊김 또는 에러: {e}")

asyncio.run(receive_robot_status())

"""
# Tkinter참고

[ Tkinter GUI 대시보드 앱 ]
   │
   ├── (1) 버튼 클릭: HTTP POST ──> [ FastAPI HTTP API ] ──> DB 저장 / ROS 명령 하달
   │
   └── (2) 화면 갱신: WS Listen <── [ FastAPI WebSocket ] <── Async Queue <── [ ROS 2 Bridge ]

Tkinter (프론트엔드): 
제어 명령(배출 시작, 정지 등)은 HTTP API(POST)로 백엔드에 요청하고, 실시간 로봇 상태(x, y 좌표 등)는 웹소켓(WS) 통로를 열어두고 귀를 기울여 받아옴 

FastAPI (백엔드): 
Tkinter의 요청을 받아 무거운 로깅과 데이터베이스 처리를 대신 해주고, ROS 2 세계와 통신할 수 있는 창구 역할을 함 
"""