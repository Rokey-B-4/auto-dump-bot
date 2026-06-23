import customtkinter as ctk
from CTkMessagebox import CTkMessagebox
import threading
import time

ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")


# 전체적인 초안 GUI 
class FoodWasteGUI:
    def __init__(self):
        self.root=ctk.CTk()

        self.root.geometry("1000x650")
        self.root.title("음식물 처리 시스템")
        self.root.configure(
            fg_color="#d8c6ff"
        )

        self.home_button=None
        self.is_running=False
        self.emergency_stop=False

        # 단계별 UI 진행상황 업데이트 
        self.steps=[

            "통 파지",
            "배출 위치 이동",
            "음식물 배출",
            "세척 중",
            "초기 위치 복귀",
            "작업 완료"

        ]
        self.step_labels=[]
        self.create_ui()
        self.root.mainloop()

    # UI 디자인 파트 
    def create_ui(self):
        for widget in self.root.winfo_children():
            widget.destroy()

        self.step_labels=[]

        self.is_running=False
        self.emergency_stop=False
        self.root.configure(
            fg_color="#d8c6ff"
        )
        
        # 상단 제목 파트 
        title=ctk.CTkLabel(

            self.root,
            text="음식물 처리 시스템",
            font=("맑은 고딕",35,"bold")

        )
        title.pack(
            pady=(30,10)
        )

        # 작업 유형 선택 파트 
        subtitle=ctk.CTkLabel(
            self.root,
            text="작업 유형 선택",
            font=("맑은 고딕",18)
        )

        subtitle.pack()

        # 프레임 
        card=ctk.CTkFrame(
            self.root,
            width=800,
            height=180,
            fg_color="white",
            corner_radius=30
        )
        card.pack(
            pady=30
        )

        # 버튼 디자인 
        btn1=ctk.CTkButton(
            card,
            text="유형1\n일반 배출 + 세척",
            width=220,
            height=80,
            font=("맑은 고딕",16,"bold"),
            command=lambda:self.start(1)
        )

        btn1.place(
            x=120,
            y=50
        )

        btn2=ctk.CTkButton(
            card,
            text="유형2\n강한 흔들기 + 세척",
            width=220,
            height=80,
            fg_color="#ff5ba6",
            hover_color="#ff2f87",
            font=("맑은 고딕",16,"bold"),
            command=lambda:self.start(2)
        )

        btn2.place(
            x=450,
            y=50
        )

        # 진행 상황 
        self.status=ctk.CTkLabel(
            self.root,
            text="대기중",
            font=("맑은 고딕",30,"bold")
        )

        self.status.pack(
            pady=20
        )

        # 진행 바 디자인 
        self.progress=ctk.CTkProgressBar(
            self.root,
            width=600,
            height=25

        )

        self.progress.pack()
        self.progress.set(0)

        line_frame=ctk.CTkFrame(
            self.root,
            fg_color="transparent"
        )

        line_frame.pack(
            pady=50
        )

        for i,step in enumerate(self.steps):
            label=ctk.CTkLabel(
                line_frame,
                text=step,
                width=100,
                height=40,
                fg_color="#d9d9d9",
                corner_radius=15,
                font=("맑은 고딕",14,"bold")
            )

            label.pack(
                side="left",
                padx=5
            )

            self.step_labels.append(label)

            if i<len(self.steps)-1:
                arrow=ctk.CTkLabel(
                    line_frame,
                    text="➜",
                    font=("Arial",24)
                )

                arrow.pack(
                    side="left"
                )

        # 테스트용 비상 버튼
        error_btn=ctk.CTkButton(
            self.root,
            text="통 탈락 시뮬레이션",
            fg_color="red",
            hover_color="#aa0000",
            command=self.trigger_drop_error
        )

        error_btn.pack()

    # 시작 버튼 > 동작 
    def start(self,mode):
        if self.is_running:
            return

        self.is_running=True
        self.progress.set(0)

        thread=threading.Thread(
            target=self.run_process,
            args=(mode,)
        )

        thread.daemon=True
        thread.start()

    # 진행 함수 
    def run_process(self,mode):
        total=len(self.steps)

        for i,step in enumerate(self.steps):
            if self.emergency_stop:
                return
            progress=(i+1)/total
            self.root.after(
                0,
                self.update_ui,
                i,
                step,
                progress
            )
            time.sleep(2)

        self.root.after(
            0,
            self.show_home_button
        )

    # 단계별 UI 업데이트 
    def update_ui(
            self,
            current,
            step,
            progress
    ):
        self.status.configure(
            text=step
        )
        self.progress.set(progress)

        for i,label in enumerate(self.step_labels):
            if i<current:
                label.configure(
                    fg_color="#47c47d"
                )
            elif i==current:
                label.configure(
                    fg_color="#ff5ba6"
                )
            else:
                label.configure(
                    fg_color="#d9d9d9"
                )

    def trigger_drop_error(self):
        self.emergency_stop=True
        self.show_fatal_error()

    def show_fatal_error(self):
        self.root.configure(
            fg_color="#ff4040"
        )
        self.status.configure(
            text="⚠ 위험 : 통 탈락 감지"
        )
        self.progress.set(0)

        CTkMessagebox(
            title="비상 경고",
            message="""
통 이탈 감지

모든 동작 강제 중지
안전 상태 진입
관리자 확인 필요
            """,
            icon="cancel"
        )
        self.show_home_button()

    def show_home_button(self):
        if self.home_button:
            return

        self.home_button=ctk.CTkButton(
            self.root,
            text="처음으로",
            width=180,
            height=55,
            font=("맑은 고딕",18,"bold"),
            fg_color="#47c47d",
            hover_color="#2da863",
            command=self.create_ui
        )

        self.home_button.pack(
            pady=20
        )
        
FoodWasteGUI()