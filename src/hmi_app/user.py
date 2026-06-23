import customtkinter as ctk
from CTkMessagebox import CTkMessagebox
import threading
import time

from manager import ManagerGUI

# 어두운 테마 기반 설정
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class FoodWasteGUI:
    def __init__(self):
        self.root = ctk.CTk()

        self.root.geometry("1050x700")
        self.root.title("음식물 스마트 처리 시스템 (Auto Dump Bot)")
        self.root.configure(fg_color="#20242f")

        self.is_running = False
        self.emergency_stop = False
        self.selected_mode = 1  # 사용자가 선택한 유형 저장

        self.steps = [
            "통 파지",
            "배출 위치 이동",
            "음식물 배출",
            "세척 중",
            "초기 위치 복귀",
            "작업 완료"
        ]

        self.step_labels = []
        
        # 첫 실행 시 1단계: 인트로 화면 표시
        self.create_intro_ui()
        self.root.mainloop()

    # 화면 초기화 유틸리티 함수
    def clear_root(self):
        for widget in self.root.winfo_children():
            widget.destroy()

    # ----------------------------------------------------
    # [1단계] 첫 시작 화면 (인트로 UI)
    # ----------------------------------------------------
    def create_intro_ui(self):
        self.clear_root()

        intro_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        intro_frame.place(relx=0.5, rely=0.5, anchor="center")

        main_title = ctk.CTkLabel(
            intro_frame,
            text="쓰레기 배출 시스템",
            font=("맑은 고딕", 46, "bold"),
            text_color="#ffffff"
        )
        main_title.pack(pady=(0, 10))

        sub_title = ctk.CTkLabel(
            intro_frame,
            text="Auto Dump Bot",
            font=("Arial", 16, "bold"),
            text_color="#4fa3e3" 
        )
        sub_title.pack(pady=(0, 50))

        start_btn = ctk.CTkButton(
            intro_frame,
            text="시작하기 (START)",
            width=320,
            height=65,
            font=("맑은 고딕", 18, "bold"),
            fg_color="#1f7ecb", 
            hover_color="#145a93",
            corner_radius=15,
            command=self.create_mode_selection_ui  # 2단계 유형 선택으로 이동
        )
        start_btn.pack()

    # ----------------------------------------------------
    # [2단계] 유형 선택 화면
    # ----------------------------------------------------
    def create_mode_selection_ui(self):
        self.clear_root()

        title_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        title_frame.pack(pady=(60, 20))

        title = ctk.CTkLabel(
            title_frame,
            text="작업 유형 선택",
            font=("맑은 고딕", 35, "bold"),
            text_color="#ffffff"
        )
        title.pack()

        subtitle = ctk.CTkLabel(
            title_frame,
            text="배출 유형 선택해 주세요.",
            font=("맑은 고딕", 15),
            text_color="#cbd3dc"
        )
        subtitle.pack(pady=(10, 0))

        # 작업 선택 카드 프레임
        card = ctk.CTkFrame(
            self.root,
            width=850,
            height=200,
            fg_color="#2d3343",
            corner_radius=20,
            border_width=2,
            border_color="#3e475e"
        )
        card.pack(pady=40)
        card.pack_propagate(False)

        card.grid_columnconfigure((0, 1), weight=1)
        card.grid_rowconfigure(0, weight=1)

        # 유형 선택 시 -> 3단계(통 배치 안내)로 이동하면서 선택한 모드 저장
        btn1 = ctk.CTkButton(
            card,
            text="유형 01\n일반 배출 + 세척",
            width=320,
            height=100,
            font=("맑은 고딕", 18, "bold"),
            fg_color="#1f7ecb",
            hover_color="#145a93",
            corner_radius=15,
            command=lambda: self.go_to_placement_guide(1)
        )
        btn1.grid(row=0, column=0, padx=30, pady=30, sticky="nsew")

        btn2 = ctk.CTkButton(
            card,
            text="유형 02\n강한 흔들기 + 세척",
            width=320,
            height=100,
            fg_color="#ff4d6d",
            hover_color="#cc2a49",
            font=("맑은 고딕", 18, "bold"),
            corner_radius=15,
            command=lambda: self.go_to_placement_guide(2)
        )
        btn2.grid(row=0, column=1, padx=30, pady=30, sticky="nsew")

    def go_to_placement_guide(self, mode):
        self.selected_mode = mode
        self.create_placement_guide_ui()

    # ----------------------------------------------------
    # [3단계] 통 배치 안내 화면
    # ----------------------------------------------------
    def create_placement_guide_ui(self):
        self.clear_root()

        guide_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        guide_frame.place(relx=0.5, rely=0.5, anchor="center")

        icon_label = ctk.CTkLabel(
            guide_frame,
            text="🗑",
            font=("맑은 고딕", 80),
            text_color="#4fa3e3"
        )
        icon_label.pack(pady=(0, 10))

        guide_text = ctk.CTkLabel(
            guide_frame,
            text="지정된 위치에 통을 놓아주세요.",
            font=("맑은 고딕", 32, "bold"),
            text_color="#ffffff"
        )
        guide_text.pack(pady=20)

        sub_guide_text = ctk.CTkLabel(
            guide_frame,
            text="로봇이 통을 감지할 수 있도록 올바르게 밀착시켜 주세요.",
            font=("맑은 고딕", 14, "bold"),
            text_color="#cbd3dc"
        )
        sub_guide_text.pack(pady=(0, 40))

        # 가로 배치를 위한 프레임 생성
        btn_frame = ctk.CTkFrame(guide_frame, fg_color="transparent")
        btn_frame.pack()

        # 통 미배치 오류 시뮬레이션 버튼 (왼쪽 배치)
        placement_error_btn = ctk.CTkButton(
            btn_frame,
            text="⚠️ 배치 오류 시뮬레이션",
            width=240,
            height=55,
            font=("맑은 고딕", 20, "bold"),
            fg_color="#3d2326",
            hover_color="#5c1e24",
            text_color="#ff4d6d",
            border_color="#ff4d6d",
            border_width=1,
            corner_radius=12,
            command=self.show_placement_error
        )
        placement_error_btn.pack(side="left", padx=10)

        # 다음 버튼 누르면 -> 4단계(최종 프로세스 구동) 화면으로 이동 (오른쪽 배치)
        next_btn = ctk.CTkButton(
            btn_frame,
            text="배치 완료 (다음)",
            width=240,
            height=55,
            font=("맑은 고딕", 16, "bold"),
            fg_color="#00fa9a", 
            hover_color="#00c77b",
            text_color="#14171c",
            corner_radius=12,
            command=self.create_process_ui
        )
        next_btn.pack(side="left", padx=10)

    # 통 미지정 위치 배치 시 띄우는 예외 팝업창
    def show_placement_error(self):
        error_message = (
            "⚠️ [경고] 통이 감지되지 않았습니다!\n\n"
            "지정된 위치에 놓아주세요"
        )
        CTkMessagebox(
            title="⚠️ 경고",
            message=error_message,
            icon="warning",  # 사용자 주의 조치를 위한 경고 아이콘
            option_1="확인",
            corner_radius=12
        )

    # ----------------------------------------------------
    # [4단계] 현재 프로세스 진행 화면
    # ----------------------------------------------------
    def create_process_ui(self):
        self.clear_root()

        self.step_labels = []
        self.is_running = False
        self.emergency_stop = False

        # 상단 타이틀
        title_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        title_frame.pack(pady=(50, 10))

        title = ctk.CTkLabel(
            title_frame,
            text="♻ SYSTEM PROCESSING...",
            font=("맑은 고딕", 32, "bold"),
            text_color="#ffffff"
        )
        title.pack()

        # [에러 수정 및 기능 개선]: 모드 변수 매핑 순서 재배치
        mode_names = {
            1: "일반 배출 + 세척",
            2: "강한 흔들기 + 세척"
        }
        current_mode_name = mode_names.get(self.selected_mode, "알 수 없는 유형")
        mode_text = f"쓰레기 배출 : [{current_mode_name}]"
        
        subtitle = ctk.CTkLabel(
            title_frame,
            text=mode_text,
            font=("맑은 고딕", 16, "bold"),
            text_color="#4fa3e3"
        )
        subtitle.pack(pady=(8, 0))

        # 현재 상태 표시부
        status_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        status_frame.pack(pady=(40, 5))

        status_title = ctk.CTkLabel(
            status_frame,
            text="현재 상황",
            font=("Arial", 12, "bold"),
            text_color="#a8b3c2"
        )
        status_title.pack()

        self.status = ctk.CTkLabel(
            status_frame,
            text="준비 완료",
            font=("맑은 고딕", 28, "bold"),
            text_color="#ffffff"
        )
        self.status.pack(pady=5)

        # 프로그레스 바
        self.progress = ctk.CTkProgressBar(
            self.root,
            width=750,
            height=16,
            progress_color="#00fa9a",
            fg_color="#333b4c"
        )
        self.progress.pack(pady=10)
        self.progress.set(0)

        # 단계별 흐름도
        line_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        line_frame.pack(pady=45)

        for i, step in enumerate(self.steps):
            label = ctk.CTkLabel(
                line_frame,
                text=step,
                width=110,
                height=45,
                fg_color="#333b4c",
                text_color="#a8b3c2",
                corner_radius=12,
                font=("맑은 고딕", 13, "bold")
            )
            label.pack(side="left", padx=3)
            self.step_labels.append(label)

            if i < len(self.steps) - 1:
                arrow = ctk.CTkLabel(
                    line_frame,
                    text="→",
                    font=("맑은 고딕", 20, "bold"),
                    text_color="#5a677d"
                )
                arrow.pack(side="left", padx=4)

        # 하단 조작/비상정지 컨테이너 프레임
        self.bottom_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        self.bottom_frame.pack(side="bottom", pady=40)

        # 비상정지 버튼 생성 및 배치
        self.error_btn = ctk.CTkButton(
            self.bottom_frame,
            text="⚠ 통 탈락 시뮬레이션 발생",
            fg_color="#3d2326",
            hover_color="#5c1e24",
            text_color="#ff4d6d",
            border_color="#ff4d6d",
            border_width=1,
            width=250,
            height=40,
            font=("맑은 고딕", 13, "bold"),
            corner_radius=10,
            command=self.trigger_drop_error
        )
        self.error_btn.pack()

        # 처음으로 이동할 수 있는 유저 버튼 인스턴스 (초기엔 숨겨둠)
        self.home_btn = ctk.CTkButton(
            self.bottom_frame,
            text="처음으로 (HOME)",
            fg_color="#1f7ecb",
            hover_color="#145a93",
            text_color="#ffffff",
            width=250,
            height=45,
            font=("맑은 고딕", 15, "bold"),
            corner_radius=12,
            command=self.create_intro_ui  # 첫 인트로 화면 단계로 회귀
        )

        # 화면에 진입하자마자 저장된 모드로 자동 시작
        self.start(self.selected_mode)

    # ----------------------------------------------------
    # 백그라운드 로직 및 스레드 파트 (오류 발생 방지 위해 스레드 사)
    # ----------------------------------------------------
    def start(self, mode):
        if self.is_running:
            return

        self.is_running = True
        self.emergency_stop = False
        self.progress.set(0)

        thread = threading.Thread(target=self.run_process, args=(mode,))
        thread.daemon = True
        thread.start()

    def run_process(self, mode):
        total = len(self.steps)

        for i, step in enumerate(self.steps):
            if self.emergency_stop:
                self.root.after(0, lambda: self.status.configure(text="시스템 중단됨", text_color="#ff4d6d"))
                self.is_running = False
                return

            progress = (i + 1) / total
            self.root.after(0, self.update_ui, i, step, progress)
            time.sleep(2)

        self.is_running = False
        # 모든 공정 성공 완료 시 안전하게 UI 전환 콜백 작동
        self.root.after(0, self.handle_process_complete)

    def update_ui(self, current, step, progress):
        self.status.configure(text=step, text_color="#ffffff")
        self.progress.set(progress)

        for i, label in enumerate(self.step_labels):
            if i < current:
                label.configure(fg_color="#153e29", text_color="#00fa9a")
            elif i == current:
                label.configure(fg_color="#1f7ecb", text_color="#ffffff")
            else:
                label.configure(fg_color="#333b4c", text_color="#a8b3c2")

    # 모든 작업 완료 시 UI 전환 처리 메서드
    def handle_process_complete(self):
        self.status.configure(text="모든 작업 완료", text_color="#00fa9a")
        
        # 가독성을 위해 시뮬레이션용 에러 버튼은 숨기고 '처음으로' 유저 버튼 띄우기
        self.error_btn.pack_forget()
        self.home_btn.pack()

    def trigger_drop_error(self):
        self.emergency_stop = True
        for label in self.step_labels:
            label.configure(fg_color="#4c1f24", text_color="#ff4d6d")
        self.show_fatal_error()

    def show_fatal_error(self):
        # 사용자가 즉시 상황을 인지할 수 있도록 직관적인 문구로 압축
        emergency_message = (
            "⚠️ [오류 발생] 시스템 중단\n\n"
            "관리자 권한 필요\n"
        )

        CTkMessagebox(
            title="⚠️ 시스템 오류",
            message=emergency_message,
            icon="cancel", 
            option_1="확인",
            corner_radius=12
        )
        ManagerGUI(self.root, self.reset_system)

    def reset_system(self):
        self.is_running = False
        self.emergency_stop = False
        # 관리자 초기화 시 처음 유형 선택창 단계로 회귀하도록 설정
        self.create_mode_selection_ui()