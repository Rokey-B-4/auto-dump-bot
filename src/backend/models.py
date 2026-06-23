# 웹 백엔드(FastAPI) 개발에서 models.py는 쉽게 말해 "우리 서버에 들어오고 나가는 데이터의 규격(신분증 검사대)"을 정의하는 곳
# "너 나한테 데이터 보낼 거면 내가 정한 이 규칙(타입, 이름)에 정확히 맞춰서 보내야 해"라고 선언해 두는 역할
# FastAPI는 이 모델을 보고 데이터가 올바른 형식인지 자동으로 검증하고, 
# 만약 숫자가 와야 하는데 글자가 오면 알아서 에러(422 Unprocessable Entity)를 튕겨내줌
# Swagger UI(스웨거 문서)가 이 규격을 바탕으로 자동 완성

from pydantic import BaseModel
# 파이썬에서 가장 유명한 데이터 검증 라이브러리인 Pydantic에서 BaseModel이라는 핵심 클래스를 가져옴
# 우리가 만들 데이터 규격 클래스들이 이 BaseModel을 상속받아(기능을 물려받아) 강력한 데이터 검증 기능을 가질 수 있게함

from typing import Optional
# 파이썬 표준 라이브러리에서 Optional이라는 타입을 가져옴

class TaskStartRequest(BaseModel):
    # TaskStartRequest라는 이름의 새로운 데이터 규격 클래스를 만듦
    # (BaseModel)을 붙여서 Pydantic의 데이터 검증 기능을 입혔음 
    # "Tkinter가 서버에 배출 작업을 시작해 달라고 요청(Request)할 때 보낼 JSON 데이터 포맷이야"라는 뜻
    
    mode_id: int  # 1 (일반 배출) 또는 2 (강하게 털기)
    # 이 요청 안에는 반드시 mode_id라는 이름의 필드가 있어야 하며, 그 값의 타입은 int(정수)여야 한다고 선언한 것
    # 주석에 적힌 대로 사용자가 화면에서 1번 버튼을 눌렀는지, 2번 버튼을 눌렀는지 그 번호(정수)를 받겠다는 뜻
    # 만약 Tkinter가 실수로 mode_id: "일반배출" 이렇게 문자열로 보내면 FastAPI가 단칼에 거절함

class ErrorLogRequest(BaseModel):
    # ErrorLogRequest라는 이름의 새로운 데이터 규격 클래스를 만듦
    # (BaseModel)을 붙여서 Pydantic의 데이터 검증 기능을 입혔음
    # "Tkinter가 서버에 에러 로그를 보고할 때 보낼 JSON 데이터 포맷이야"라는 뜻
    
    task_id: str
    # 에러가 발생한 배출 작업의 ID를 받으며, 타입은 str(문자열)이어야함
    # "아까 발급받은 TASK-20260622-xxxx 중에서 어떤 작업 도중에 터진 에러인지" 매핑하기 위한 고유 키값

    error_code: str  # ERR_PICK, ERR_DROP, ERR_COLLISION 등
    # 에러의 종류를 나타내는 짧은 코드를 받으며, 타입은 str(문자열)
    # 나중에 아파트 관리자가 에러 통계를 내거나 필터링하기 쉽도록 ERR_PICK(통 못 잡음), ERR_COLLISION(충돌) 같은 표준화된 코드를 문자열로 받겠다는 설계
    
    error_msg: str
    # 에러의 상세한 내용을 담는 필드이며, 타입은 str(문자열)
    # "정격 토크 초과 충돌 감지"나 "그리퍼 파지력 부족"처럼 엔지니어가 나중에 보고 수리할 수 있도록 사람이 읽을 수 있는 구체적인 실패 원인 문장을 받음