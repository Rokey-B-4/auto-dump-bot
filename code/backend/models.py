# 웹 백엔드(FastAPI) 개발에서 models.py는 쉽게 말해 "우리 서버에 들어오고 나가는 데이터의 규격(신분증 검사대)"을 정의하는 곳
# "너 나한테 데이터 보낼 거면 내가 정한 이 규칙(타입, 이름)에 정확히 맞춰서 보내야 해"라고 선언해 두는 역할
# FastAPI는 이 모델을 보고 데이터가 올바른 형식인지 자동으로 검증하고, 
# 만약 숫자가 와야 하는데 글자가 오면 알아서 에러(422 Unprocessable Entity)를 튕겨내줌
# Swagger UI(스웨거 문서)가 이 규격을 바탕으로 자동 완성

from pydantic import BaseModel, Field
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

# ------------------------------------------------------------------
# 아래부터 신규 추가: 로봇 직접 제어용 요청 모델 3종
# ------------------------------------------------------------------

class EmergencyStopRequest(BaseModel):
    # "관리자 GUI에서 비상 정지 버튼을 눌렀을 때 보낼 JSON 데이터 포맷이야"라는 뜻
    task_id: Optional[str] = None
    # 특정 작업을 정지시키는 거면 그 작업의 task_id를 보내고,
    # 전체 비상정지(어떤 작업인지 특정 안 함)인 경우엔 안 보내도 되도록 Optional + 기본값 None 처리


class ResetRequest(BaseModel):
    """
    시스템 리셋 요청 바디.
    현재는 별도 입력 파라미터가 없지만, 추후 확장(예: 리셋 사유 코드 등)을
    고려해 빈 모델로 선언해둠. 요청 바디 없이 호출해도 FastAPI가 정상 처리함.
    """
    pass


class MoveJointRequest(BaseModel):
    # "관리자 GUI 슬라이더로 6축 관절각을 조정한 뒤 MoveJ 전송을 누르면 보낼 JSON 데이터 포맷이야"라는 뜻
    J1: float = Field(..., description="Joint 1 각도 (deg)")
    J2: float = Field(..., description="Joint 2 각도 (deg)")
    J3: float = Field(..., description="Joint 3 각도 (deg)")
    J4: float = Field(..., description="Joint 4 각도 (deg)")
    J5: float = Field(..., description="Joint 5 각도 (deg)")
    J6: float = Field(..., description="Joint 6 각도 (deg)")
    # 6개 필드 모두 필수(...)이며 float(실수) 타입.
    # 두산 협동로봇의 관절 가동범위 제한이 정해지면 Field(..., ge=최소값, le=최대값)으로
    # 범위 검증을 추가하는 게 안전함 (현재는 타입 검증만 수행)