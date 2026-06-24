import customtkinter as ctk
from CTkMessagebox import CTkMessagebox
import threading
import time
import requests
import queue

# 관리자 콘솔 클래스 임포트
from manager import ManagerGUI
from api_service import APIService

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class FoodWasteGUI:
    # ========================================================
    # 1. 초기화 / 설정 
    # ========================================================
    def __init__(self):
        self.root = ctk.CTk()
        self.api_service = APIService() # API 서비스 초기화
        self.event_queue = queue.Queue()
        self.api_service.start_websocket_listener(self.handle_ws_message)

        self.current_task_id = None     # 서버에서 받은 Task ID 저장용

        self.root.geometry("1050x700")
        self.root.title("음식물 스마트 처리 시스템 (Auto Dump Bot)")
        self.root.configure(fg_color="#20242f")

        # 제어 및 상태 변수
        self.is_running = False
        self.emergency_stop = False
        self.selected_mode = 1
        self._active_after_ids = []  # 실시간 스케줄러 ID 추적 리스트

        # 공정 시퀀스 정의
        self.steps = ["통 파지", "배출 위치 이동", "음식물 배출", "세척 중", "초기 위치 복귀", "작업 완료"]
        self.step_labels = []
        
        # [메인 컨테이너] 화면 전활 시 관리자 창 유실 방지용 부모 프레임
        self.main_container = ctk.CTkFrame(self.root, fg_color="transparent")
        self.main_container.pack(fill="both", expand=True)
        
        # 관리자 콘솔 가동 (인스턴스 연동)
        self.manager_console = ManagerGUI(self.reset_system, self)

        # 초기 뷰 설정
        self.create_intro_ui()
        
        # 관리자 윈도우 포커싱 최적화 후 루프 진입
        self.root.after(100, lambda: self.manager_console.window.lift())
        self.root.after(50, self.process_queue)
        self.root.mainloop()

    # ========================================================
    # 2. 큐 기반 이벤트 처리 시스템
    # ========================================================
    def process_queue(self):
        try:
            while not self.event_queue.empty():
                data = self.event_queue.get_nowait()
                msg_type = data.get("type")
                payload = data.get("payload")

                if msg_type == "PROCESS_STATE":
                    # 서버가 보내준 단계 이름이 들어오면 UI 업데이트
                    if hasattr(self, "status") and self.status.winfo_exists():
                        self.status.configure(text=payload)
                        # 서버가 특정 단계 이름을 보내면 그에 맞춰 진행률도 업데이트
                        self.update_step_ui_by_server(payload)

                elif msg_type == "SAFETY_EVENT":
                    if payload == "EMERGENCY_STOP":
                        self.trigger_drop_error()
                    
        except Exception as e:
            print(f"Queue Processing Error: {e}")
        self.root.after(50, self.process_queue)

    # 서버 메시지에 따라 단계 UI를 업데이트하는 함수 추가
    def update_step_ui_by_server(self, current_step_name):
        # self.steps = ["통 파지", "배출 위치 이동", ... ] 리스트 활용
        try:
            idx = self.steps.index(current_step_name)
            progress = (idx + 1) / len(self.steps)
            self.update_ui(idx, current_step_name, progress)
        except ValueError:
            pass # 정의되지 않은 단계면 무시

    def handle_ws_message(self, data):
        """웹소켓 스레드에서 호출됨: 데이터를 큐에 넣고 즉시 종료"""
        self.event_queue.put(data)

    def _real_ui_update(self, data):
        """여기서 안전하게 UI 조작"""
        # (예시)
        msg_type = data.get("type")
        if msg_type == "PROCESS_STATE":
            if hasattr(self, "status_label") and self.status_label.winfo_exists():
                self.status_label.configure(text=data.get("payload"))
    # ========================================================
    # 3. 코어 유틸리티 및 안전 매커니즘 시스템
    # ========================================================
    def clear_root(self):
        """현재 메인 컨테이너 내부의 모든 유저 인터페이스 위젯 제거"""
        for after_id in self._active_after_ids:
            try:
                self.root.after_cancel(after_id)
            except:
                pass
        self._active_after_ids.clear()

        for widget in self.main_container.winfo_children():
            widget.destroy()

    def _safe_after(self, ms, command, *args):
        """비상 정지 및 화면 파괴 시 스케줄러 유령 실행을 막는 방탄 스케줄러"""
        if self.emergency_stop and command.__name__ == "<lambda>":
            return
        after_id = self.root.after(ms, command, *args)
        self._active_after_ids.append(after_id)
        return after_id

    # ========================================================
    # 4. 화면 뷰 생성 및 공정 시퀀스 UI 시스템
    # VIEW 01: 인트로 시작 화면 (Intro UI)
    # ========================================================
    def create_intro_ui(self):
        self.clear_root()
        
        intro_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        intro_frame.place(relx=0.5, rely=0.5, anchor="center")

        main_title = ctk.CTkLabel(intro_frame, text="쓰레기 배출 시스템", font=("맑은 고딕", 46, "bold"), text_color="#ffffff")
        main_title.pack(pady=(0, 10))

        sub_title = ctk.CTkLabel(intro_frame, text="Auto Dump Bot", font=("Arial", 16, "bold"), text_color="#4fa3e3")
        sub_title.pack(pady=(0, 60))

        start_btn = ctk.CTkButton(
            intro_frame, 
            text="시작하기 (START)", 
            width=340, 
            height=65, 
            font=("맑은 고딕", 18, "bold"), 
            fg_color="#1f7ecb", 
            hover_color="#145a93", 
            corner_radius=15, 
            command=self.create_mode_selection_ui
        )
        start_btn.pack()

    # ========================================================
    # VIEW 02: 작업 유형 선택 화면 (Mode Selection UI)
    # ========================================================
    def create_mode_selection_ui(self):
        self.clear_root()
        
        # 상단 타이틀 구역
        title_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        title_frame.pack(pady=(80, 20))

        title = ctk.CTkLabel(title_frame, text="작업 유형 선택", font=("맑은 고딕", 36, "bold"), text_color="#ffffff")
        title.pack()

        subtitle = ctk.CTkLabel(title_frame, text="원하시는 배출 방식을 선택해 주세요.", font=("맑은 고딕", 15), text_color="#cbd3dc")
        subtitle.pack(pady=(12, 0))

        # 모드 선택 카드 보드
        card = ctk.CTkFrame(self.main_container, width=850, height=220, fg_color="#2d3343", corner_radius=20, border_width=2, border_color="#3e475e")
        card.pack(pady=40)
        card.pack_propagate(False)
        card.grid_columnconfigure((0, 1), weight=1)
        card.grid_rowconfigure(0, weight=1)

        # 유형 01 버튼
        btn1 = ctk.CTkButton(
            card, 
            text="유형 01\n일반 배출 + 세척", 
            width=340, 
            height=110, 
            font=("맑은 고딕", 18, "bold"), 
            fg_color="#1f7ecb", 
            hover_color="#145a93", 
            corner_radius=15, 
            command=lambda: self.go_to_placement_guide(1)
        )
        btn1.grid(row=0, column=0, padx=30, pady=30, sticky="nsew")

        # 유형 02 버튼
        btn2 = ctk.CTkButton(
            card, 
            text="유형 02\n강한 흔들기 + 세척", 
            width=340, 
            height=110, 
            font=("맑은 고딕", 18, "bold"), 
            fg_color="#ff4d6d", 
            hover_color="#cc2a49", 
            corner_radius=15, 
            command=lambda: self.go_to_placement_guide(2)
        )
        btn2.grid(row=0, column=1, padx=30, pady=30, sticky="nsew")

    def go_to_placement_guide(self, mode):
        self.selected_mode = mode
        self.create_placement_guide_ui()

    # ========================================================
    # VIEW 03: 하드웨어 수거통 배치 가이드 화면 (Placement Guide UI)
    # ========================================================
    def create_placement_guide_ui(self):
        self.clear_root()
        
        guide_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        guide_frame.place(relx=0.5, rely=0.5, anchor="center")

        icon_label = ctk.CTkLabel(guide_frame, text="🗑", font=("맑은 고딕", 85), text_color="#4fa3e3")
        icon_label.pack(pady=(0, 15))

        guide_text = ctk.CTkLabel(guide_frame, text="지정된 위치에 통을 놓아주세요.", font=("맑은 고딕", 32, "bold"), text_color="#ffffff")
        guide_text.pack(pady=15)

        sub_guide_text = ctk.CTkLabel(guide_frame, text="로봇이 통을 감지할 수 있도록 올바르게 밀착시켜 주세요.", font=("맑은 고딕", 14), text_color="#cbd3dc")
        sub_guide_text.pack(pady=(0, 50))

        # 하단 조작 제어 존
        btn_frame = ctk.CTkFrame(guide_frame, fg_color="transparent")
        btn_frame.pack()

        placement_error_btn = ctk.CTkButton(
            btn_frame, 
            text="⚠️ 배치 오류 시뮬레이션", 
            width=240, 
            height=55, 
            font=("맑은 고딕", 15, "bold"), 
            fg_color="#3d2326", 
            hover_color="#5c1e24", 
            text_color="#ff4d6d", 
            border_color="#ff4d6d", 
            border_width=1, 
            corner_radius=12, 
            command=self.show_placement_error
        )
        placement_error_btn.pack(side="left", padx=12)

        next_btn = ctk.CTkButton(
            btn_frame, 
            text="배치 완료 (다음)", 
            width=240, 
            height=55, 
            font=("맑은 고딕", 15, "bold"), 
            fg_color="#00fa9a", 
            hover_color="#00c77b", 
            text_color="#14171c", 
            corner_radius=12, 
            command=self.create_process_ui
        )
        next_btn.pack(side="left", padx=12)

    def show_placement_error(self):
        error_message = (
            "▶ [배치 오류] 통이 감지되지 않았습니다! ◀\n"
            "━━━━━━━━━━━━━━━━━━━━━━\n\n"
            "지정된 위치에 수거통이 없거나 올바르게 밀착되지 않았습니다.\n"
            "수거통의 정렬 상태를 다시 확인한 후 확실하게 밀착시켜 주세요!"
        )
        CTkMessagebox(title="⚠️ 수거통 배치 오류 안내", message=error_message, icon="warning", option_1="확인", corner_radius=12, width=500)

    # ========================================================
    # VIEW 04: 실시간 공정 진행 화면 (Process Monitoring UI)
    # =======================================================
    def create_process_ui(self):
        self.clear_root()
        self.step_labels = []
        self.is_running = False
        self.emergency_stop = False

        # 1. 상태 메인 대시보드 타이틀 헤더
        title_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        title_frame.pack(pady=(50, 10))

        title = ctk.CTkLabel(title_frame, text="♻ SYSTEM PROCESSING...", font=("맑은 고딕", 32, "bold"), text_color="#ffffff")
        title.pack()

        mode_names = {1: "일반 배출 + 세척", 2: "강한 흔들기 + 세척"}
        current_mode_name = mode_names.get(self.selected_mode, "알 수 없는 유형")
        
        subtitle = ctk.CTkLabel(title_frame, text=f"선택 유형 :   {current_mode_name}", font=("맑은 고딕", 15, "bold"), text_color="#4fa3e3")
        subtitle.pack(pady=(10, 0))

        # 2. 실시간 구동 상태 모니터 레이블
        status_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        status_frame.pack(pady=(40, 5))

        status_title = ctk.CTkLabel(status_frame, text="현재 공정 단계", font=("맑은 고딕", 12, "bold"), text_color="#a8b3c2")
        status_title.pack()

        self.status = ctk.CTkLabel(status_frame, text="준비 완료", font=("맑은 고딕", 28, "bold"), text_color="#ffffff")
        self.status.pack(pady=8)

        # 3. 메인 프로세스 진행 바
        self.progress = ctk.CTkProgressBar(self.main_container, width=780, height=16, progress_color="#00fa9a", fg_color="#333b4c")
        self.progress.pack(pady=15)
        self.progress.set(0)

        # 4. 6단계 시퀀스 타임라인 노드 레이아웃 Zone
        line_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        line_frame.pack(pady=50)

        for i, step in enumerate(self.steps):
            label = ctk.CTkLabel(
                line_frame, 
                text=step, 
                width=115, 
                height=48, 
                fg_color="#333b4c", 
                text_color="#a8b3c2", 
                corner_radius=12, 
                font=("맑은 고딕", 13, "bold")
            )
            label.pack(side="left", padx=4)
            self.step_labels.append(label)

            # 노드 간 방향 화살표
            if i < len(self.steps) - 1:
                arrow = ctk.CTkLabel(line_frame, text="→", font=("맑은 고딕", 20, "bold"), text_color="#5a677d")
                arrow.pack(side="left", padx=5)

        # 5. 하단 하드웨어 이벤트 액션 패널
        self.bottom_frame = ctk.CTkFrame(self.main_container, fg_color="transparent")
        self.bottom_frame.pack(side="bottom", pady=40)

        self.error_btn = ctk.CTkButton(
            self.bottom_frame, 
            text="⚠️ 통 탈락 시뮬레이션 발생", 
            fg_color="#3d2326", 
            hover_color="#5c1e24", 
            text_color="#ff4d6d", 
            border_color="#ff4d6d", 
            border_width=1, 
            width=260, 
            height=45, 
            font=("맑은 고딕", 13, "bold"), 
            corner_radius=10, 
            command=self.trigger_drop_error
        )
        self.error_btn.pack()

        self.home_btn = ctk.CTkButton(
            self.bottom_frame, 
            text="처음으로 (HOME)", 
            fg_color="#1f7ecb", 
            hover_color="#145a93", 
            text_color="#ffffff", 
            width=260, 
            height=48, 
            font=("맑은 고딕", 15, "bold"), 
            corner_radius=12, 
            command=self.create_intro_ui
        )

        # 자동 공정 프로세스 스레드 기동
        self.start(self.selected_mode)

    # =======================================================
    # 5. 프로세스 제어 
    # ========================================================
    def start(self, mode):
        if self.is_running:
            return
        self.is_running = True
        response = self.api_service.request_task_start(mode)
        
        if response and "task_id" in response:
            self.current_task_id = response["task_id"]
            # run_process 스레드 실행 코드 삭제 (필요 없음!)
        else:
            self.is_running = False
            CTkMessagebox(title="통신 오류", message="서버에서 응답이 없습니다.")

    def run_process(self, mode):
        total = len(self.steps)
        for i, step in enumerate(self.steps):
            if self.emergency_stop:
                self._safe_after(0, lambda: self.status.configure(text="시스템 중단됨", text_color="#ff4d6d") if hasattr(self, "status") and self.status.winfo_exists() else None)
                self.is_running = False
                return
            progress = (i + 1) / total
            self._safe_after(0, self.update_ui, i, step, progress)
            time.sleep(2)
        self.is_running = False
        self._safe_after(0, self.handle_process_complete)

    def update_ui(self, current, step, progress):
        if not hasattr(self, "status") or not self.status.winfo_exists():
            return
        self.status.configure(text=step, text_color="#ffffff")
        self.progress.set(progress)

        if self.manager_console:
            self.manager_console.update_user_status(f"이용 중 ({step} 처리 중)")

        for i, label in enumerate(self.step_labels):
            if label.winfo_exists():
                if i < current:
                    label.configure(fg_color="#153e29", text_color="#00fa9a")
                elif i == current:
                    label.configure(fg_color="#1f7ecb", text_color="#ffffff")
                else:
                    label.configure(fg_color="#333b4c", text_color="#a8b3c2")

    def handle_process_complete(self):
        if not hasattr(self, "status") or not self.status.winfo_exists():
            return
        self.status.configure(text="모든 작업 완료", text_color="#00fa9a")
        
        if self.manager_console:
            self.manager_console.update_user_status("대기 중 (작업 완료)")
        if self.error_btn.winfo_exists():
            self.error_btn.pack_forget()
        if self.home_btn.winfo_exists():
            self.home_btn.pack()

    # ========================================================
    # 6. EMERGENCY LOCK SCREEN SYSTEM (비상 정지 전체 오버레이)
    # ========================================================
    def trigger_drop_error(self):
        """센서 단락 혹은 관리자 콘솔 긴급 중단 명령 패킷 수신 시 호출"""
        self.emergency_stop = True
        self.is_running = False
        
        if self.current_task_id:
            self.api_service.send_error_log(self.current_task_id, "DROP001", "수거통 탈락 감지")

        if self.manager_console:
            self.manager_console.update_user_status("❌ 원격 강제 중단됨 (인터록 가동)")
            
        self.emergency_stop = True    
        self.show_fatal_error_screen()

    def show_fatal_error_screen(self):
        """기존 자식 위젯을 해치지 않고 붉은색 강제 인터록 스크린 레이어를 최상단에 완전 오버레이"""
        if hasattr(self, "error_bg_frame") and self.error_bg_frame.winfo_exists():
            return

        # 1. 꽉 채우는 다크 레드 비상 대피용 도화지 생성
        self.error_bg_frame = ctk.CTkFrame(self.root, fg_color="#4c1f24", corner_radius=0)
        self.error_bg_frame.place(relx=0, rely=0, relwidth=1, relheight=1)

        # 2. 비상 타이틀 컴포넌트
        lbl_emoji = ctk.CTkLabel(self.error_bg_frame, text="🚨", font=("맑은 고딕", 80))
        lbl_emoji.pack(pady=(130, 10))

        lbl_title = ctk.CTkLabel(
            self.error_bg_frame, 
            text="EMERGENCY STOP ACTIVATED", 
            font=("Consolas", 28, "bold"), 
            text_color="#ff4d6d"
        )
        lbl_title.pack(pady=10)

        # 3. 비상 상황 조치 내용 안내 본문 패널
        guide_text = (
            "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n"
            "관리자 원격 제어 또는 하드웨어 중단 신호에 의해 비상 정지 되었습니다.\n"
            "현장 안전 요건을 파악 중이오니 관리자의 조치가 완료될 때까지 대기해 주십시오.\n\n"
            "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        )
        
        lbl_guide = ctk.CTkLabel(
            self.error_bg_frame, 
            text=guide_text, 
            font=("맑은 고딕", 15, "bold"), 
            text_color="#ffffff",
            justify="center"
        )
        lbl_guide.pack(pady=35)

        # 4. 하단 고정 인터록 잠금 캡션 바
        lbl_status = ctk.CTkLabel(
            self.error_bg_frame, 
            text="🔒 SYSTEM LOCK / 원격 관리 권한 해제 대기 중...", 
            font=("맑은 고딕", 12, "italic"), 
            text_color="#a8b3c2"
        )
        lbl_status.pack(side="bottom", pady=25)

    # ========================================================
    # 7. RECOVERY SYSTEM (관리자 콘솔 원격 복구 초기화 커넥터)
    # ========================================================
    def reset_system(self):
        """관리자가 인터록 복구 승인 시 호출되는 1차 세이프 리셋 진입점"""
        self.emergency_stop = True  
        self.is_running = False
        
        # 관리자 창의 컴포넌트 파괴 스케줄러와 메모리 레이스가 발생하지 않도록 
        # 유예 마진을 150ms로 늘려서 완전히 격리 후 안전 리셋 진입
        self.root.after(150, self._safe_ui_reset)

    def _safe_ui_reset(self):
        """붉은 장막을 걷어내고 메인 작업 선택 UI 뷰로 안전 귀환"""
        # Tkinter 윈도우 인스턴스가 파괴 중인 상태라면 무시 (안전장치)
        if not self.root.winfo_exists():
            return

        self.emergency_stop = False
        self.is_running = False
        
        # 붉은 비상 장막 프레임이 존재한다면 메인 컨테이너 조작 전에 '먼저' 안전 소멸
        if hasattr(self, "error_bg_frame") and self.error_bg_frame.winfo_exists():
            try:
                self.error_bg_frame.place_forget()
                self.error_bg_frame.destroy()
            except:
                pass
            
        if self.manager_console:
            self.manager_console.update_user_status("대기 중")
            
        # 프레임 교체 도중 사이드바 클릭 등의 충돌을 막기 위해 after 유예를 한번 더 주고 화면 재빌드
        self.root.after(50, self.create_mode_selection_ui)


if __name__ == "__main__":
    FoodWasteGUI()