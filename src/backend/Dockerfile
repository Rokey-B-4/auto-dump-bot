# 프로젝트가 돌아갈 완벽한 컴퓨터(컨테이너)의 설계도
# 협동로봇 백엔드는 단순한 파이썬 웹 서버가 아니라 ROS 2 Humble 시스템이 내장된 특수한 환경이 필요함
# 개발자 PC의 OS나 패키지 버전에 상관없이, 도커만 있으면 어디서든 똑같은 환경(ROS 2 + Python 의존성 + SQLite)을 순식간에 복제해 내는 역할을 함 

# 1. ROS 2 Humble 공식 베이스 이미지 사용
FROM osrf/ros:humble-desktop-full

# 2. 필수 시스템 패키지 및 파이썬 pip 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    sqlite3 \
    && rm -rf /var/lib/apt/lists/*
    # && rm -rf /var/lib/apt/lists/*: 패키지 설치 후 더 이상 필요 없는 임시 캐시 파일들을 즉시 지워버려서 도커 이미지 용량을 수백 MB 이상 줄여줌 

# 3. 작업 디렉토리 설정
WORKDIR /app
# 리눅스의 cd /app과 같음 

# 4. 의존성 파일 복사 및 패키지 설치
COPY requirements.txt .
# 내 PC의 requirements.txt를 컨테이너의 /app/ 내부로 가져옴
RUN pip3 install --no-cache-dir -r requirements.txt
# 파이썬 패키지들을 설치함. --no-cache-dir을 붙여 내부 다운로드 캐시를 남기지 않아 용량을 또 한 번 최적화
# 굳이 코드를 다 복사하지 않고 이 두 줄을 먼저 쓴 이유: 
# 도커는 레이어 캐싱을 함. 소스 코드가 바뀔 때마다 무거운 pip install을 매번 처음부터 다시 하면 빌드가 너무 느려짐
# 의존성 파일을 먼저 복사해 두면, requirements.txt 내용이 바뀌지 않는 한 파이썬 패키지 설치 단계를 캐시로 순식간에 패스(Skip)

# 5. 소스 코드 전체 복사
COPY . .
# 현재 백엔드 폴더에 있는 모든 소스 코드(main.py, config.py 등)를 컨테이너의 /app 폴더로 통째로 밀어 넣음 

# 6. 컨테이너 실행 시 ROS 2 환경을 소싱하고 FastAPI 서버 실행
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && uvicorn main:app --host 0.0.0.0 --port 8000"]
# CMD [...]: 컨테이너가 성공적으로 켜졌을 때 가장 먼저, 그리고 딱 한 번 실행할 메인 명령어
# "/bin/bash", "-c": 단순 쉘이 아니라 배시(Bash) 쉘을 이용해 문자열로 된 명령어를 이어서 실행하겠다는 의미
# source /opt/ros/humble/setup.bash: ★ ROS 2 연동의 핵심. 
# 컨테이너 안에서 rclpy 같은 ROS 2 라이브러리를 파이썬이 인식하려면 반드시 이 환경 설정 파일을 로드(source)해야 함
# && uvicorn main:app --host 0.0.0.0 --port 8000: 환경 소싱이 끝나면 이어서 uvicorn 서버를 켬
# config.py에서 세팅한 대로 포트는 8000, 호스트는 도커 외부 통신을 위해 0.0.0.0으로 지정되어 정상 구동을 시작