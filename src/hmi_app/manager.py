import customtkinter as ctk
from datetime import datetime


class ManagerGUI:

    def __init__(self, parent, reset_callback):
        self.parent = parent
        self.reset_callback = reset_callback

        self.win = ctk.CTkToplevel(parent)
        self.win.title("관리자 시스템 제어 (Manager Mode)")
        self.win.geometry("600x560")
        
        # 메인 UI와 톤을 맞추되, 시인성을 위해 배경을 조금 더 짙게 처리
        self.win.configure(fg_color="#14171c")
        self.win.resizable(False, False)

        # 부모 창 위치 기준으로 관리자 창 띄우기
        parent.update_idletasks()

        px = parent.winfo_x()
        py = parent.winfo_y()
        pw = parent.winfo_width()
        
        x = px + pw + 10
        y = py

        self.win.geometry(f"600x560+{x}+{y}")
        self.create_ui()

    def create_ui(self):
        # 상단 비상 경고 헤더 (밝고 선명한 레드 사용)
        error_title = ctk.CTkLabel(
            self.win,
            text="⚠ SYSTEM EMERGENCY ALERT",
            font=("맑은 고딕", 26, "bold"),
            text_color="#ff4d4d"  # 채도를 높여 어두운 배경에서도 확 띄게 변경
        )
        error_title.pack(pady=(30, 5))

        error_subtitle = ctk.CTkLabel(
            self.win,
            text="관리자 승인 및 시스템 복구가 필요한 상태입니다.",
            font=("맑은 고딕", 14, "bold"),
            text_color="#ffffff"  # 텍스트가 묻히지 않도록 순백색으로 변경
        )
        error_subtitle.pack(pady=(0, 15))

        # 1. 외곽 인포 프레임 (대비감을 주기 위해 짙은 그레이 유지)
        info_frame = ctk.CTkFrame(
            self.win, 
            fg_color="#1e222b",
            corner_radius=15,
            border_width=2,
            border_color="#ff4d4d"  # 테두리를 더 두껍고 밝은 레드로 강조
        )
        info_frame.pack(pady=10, padx=25, fill="both", expand=True)

        now = datetime.now()

        # 2. 로그 박스 디자인 전면 수정 (★눈에 확 띄도록 내부는 밝은 크림 화이트 톤으로 반전)
        log_box = ctk.CTkTextbox(
            info_frame,
            width=500,
            height=260,
            font=("맑은 고딕", 14, "bold"), # 글씨를 굵게 하여 가독성 확보
            fg_color="#f5f6f8",             # 밝은 배경색으로 반전
            text_color="#111111",           # 글씨는 진한 먹색으로 인쇄물처럼 표현
            corner_radius=10
        )
        log_box.pack(pady=20, padx=20, fill="both", expand=True)

        log_box.insert(
            "0.0",
            f"""[SYSTEM LOG - EMERGENCY]

■ 에러 코드 : ER_001
■ 발생 시간 : {now.strftime("%Y-%m-%d %H:%M:%S")}

■ 오류 내용 :
  - 메인 그리퍼 통 탈락(Drop) 감지

■ 현재 로봇 상태 :
  - 하드웨어 및 모터 제어 즉시 정지 (STOP)
  - 인계 안전 모드 (Safety Fallback) 진입 완료
  - 하드웨어 점검 후 관리자 수동 초기화 대기 중...
"""
        )
        # 로그 박스 편집 불가 설정
        log_box.configure(state="disabled")

        # 3. 시스템 초기화 버튼 디자인 (더 밝고 경고 분위기가 강한 크림슨 레드)
        reset_btn = ctk.CTkButton(
            self.win,
            text="시스템 수동 초기화 (RESET)",
            width=320,
            height=50,
            font=("맑은 고딕", 16, "bold"),
            fg_color="#ff334b",      # 훨씬 밝고 강렬한 레드 톤
            hover_color="#d61c33",
            text_color="#ffffff",
            corner_radius=12,
            command=self.reset_system
        )
        reset_btn.pack(pady=(20, 30))

    def reset_system(self):
        print("관리자 시스템 초기화 명령 수신")
        self.reset_callback()
        self.win.destroy()