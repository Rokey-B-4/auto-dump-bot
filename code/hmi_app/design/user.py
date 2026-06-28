import customtkinter as ctk
from CTkMessagebox import CTkMessagebox
import time
import queue

import requests

# 관리자 콘솔 클래스 임포트
from .manager import ManagerGUI
from hmi_app.api.api_user import UserAPI

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class FoodWasteGUI:
    # ROS 상태 문자열 키워드 → (단계 인덱스, steps 텍스트) 매핑
    STEP_KEYWORD_MAP = [
        (0, ["수거통 위치 이동", "파지"]),        # 통 파지
        (1, ["배출 위치 이동"]),                  # 배출 위치 이동
        (2, ["DUMPING", "mode=", "shaking", "배출"]),  # 음식물 배출
        (3, ["WASHING", "세척"]),                 # 세척 중
        (4, ["복귀", "RESET", "home"]),           # 초기 위치 복귀
        (5, ["COMPLETE", "완료", "IDLE"]),        # 작업 완료
    ]
    # ========================================================
    # 1. 초기화 / 설정 
    # ========================================================
    def __init__(self):
        self.root = ctk.CTk()
        self.api_service = UserAPI() # API 서비스 초기화
        self.event_queue = queue.Queue()
        self.api_service.start_websocket_listener(self.handle_ws_message)

        self.current_task_id = None     # 서버에서 받은 Task ID 저장용

        self.root.geometry("1050x700")
        self.root.title("음식물 스마트 처리 시스템 (Auto Dump Bot)")
        self.root.configure(fg_color="#20242f")
        self._last_step_idx = -1

        # 제어 및 상태 변수
        self.is_running = False
        self.emergency_stop = False
        self.in_error_state = False
        self.selected_mode = 1
        self._active_after_ids = []  # 실시간 스케줄러 ID 추적 리스트
        self._process_queue_after_id = None  # 유저 UI 이벤트 큐 루프 after ID
        self._last_handled_safety_event = None  # ROBOT_STATUS 캐시 반복에 의한 중복 팝업 방지
        # [백업 대비 추가] 긴급정지 복구의 이벤트 유효시간, 실제 로봇 체크포인트,
        # 진행 중인 HOME/RESUME 요청을 추적합니다. 과거 안전 이벤트 재실행과 중복 RESET을 막고,
        # 재개 후 기존 진행 화면을 정확한 단계부터 복원하기 위한 상태값
        self._ignore_safety_before = 0.0
        self.recovery_stage = "IDLE"
        self._received_recovery_stage = False
        self._pending_recovery_action = None
        self._resume_session_active = False

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
        self._restart_process_queue()
        self.root.mainloop()

    # ========================================================
    # 2. 큐 기반 이벤트 처리 시스템
    # ========================================================
    def process_queue(self):
        """백엔드 웹소켓 메시지 큐를 안전하게 소모하여 UI 상태 전이를 처리하는 데몬 루프"""
        self._process_queue_after_id = None
        # 윈도우 창 자체가 이미 파괴되었다면 즉시 스케줄러 다운
        if not hasattr(self, "root") or not self.root.winfo_exists():
            return
            
        try:
            while not self.event_queue.empty():
                data = self.event_queue.get_nowait()
                msg_type = data.get("type")

                # ── PROCESS_STATE ──────────────────────────────────────────
                if msg_type == "PROCESS_STATE":
                    if self.emergency_stop or self.in_error_state:
                        continue  # 비상/오류 상태에서는 완료 상태로 덮어쓰지 않음

                    payload = data.get("payload", "")
                    # [백업 대비 추가] 구버전 로봇 노드가 재개 완료를 IDLE 문구로 보내더라도
                    # HMI에서는 완료 단계와 100% 진행률로 정규화해 잘못된 종료 표시를 고침
                    if self._resume_session_active and "분기 복구 후 초기 위치 복귀" in payload:
                        payload = "작업 완료"
                        self._resume_session_active = False
                        self._last_step_idx = len(self.steps) - 1
                        self.update_ui(
                            self._last_step_idx,
                            self.steps[self._last_step_idx],
                            1.0,
                        )
                        self.handle_process_complete()
                    if hasattr(self, "status") and self.status and self.status.winfo_exists():
                        try:
                            self.status.configure(text=payload)
                            # [백업 대비 수정] RecoveryStage를 받기 전까지만 문구 기반 단계를 사용
                            # 이후에는 정확한 체크포인트를 단일 기준으로 사용해 두 매핑의 충돌을 방지
                            if not self._received_recovery_stage:
                                self.update_step_ui_by_server(payload)
                        except Exception as e:
                            print(f"UI 업데이트 오류: {e}")

                # ── SAFETY_EVENT ────────────────────────────────────────────
                elif msg_type == "SAFETY_EVENT":
                    if not self.emergency_stop:   # 중복 트리거 방지
                        error_code = data.get("error_code", "")
                        if not error_code:
                            continue
                        print(f"[WS] SAFETY_EVENT 수신: code={error_code}", flush=True)
                        
                        self._handle_safety_event(error_code, data)

                # ── ROBOT_STATUS ────────────────────────────────────────────
                elif msg_type == "ROBOT_STATUS":
                    # [백업 대비 수정] 누적 상태에서는 진행 체크포인트만 사용
                    # 안전 팝업은 실시간 SAFETY_EVENT에서만 발생시켜 과거 이벤트 재실행을 막음
                    inner = data.get("payload", {})
                    if isinstance(inner, dict):
                        self.recovery_stage = inner.get("recovery_stage", self.recovery_stage)

                # [백업 대비 추가] 체크포인트 이벤트로 진행 바를 갱신하고, HOME 복귀 중 IDLE 도착을
                # 실제 완료 신호로 사용. 화면을 로봇보다 먼저 전환하던 문제를 고침
                elif msg_type == "RECOVERY_STAGE":
                    self.recovery_stage = data.get("stage", self.recovery_stage)
                    self._received_recovery_stage = True
                    if self._pending_recovery_action == "HOME" and self.recovery_stage == "IDLE":
                        self._pending_recovery_action = None
                        self.root.after(0, self._finish_home_recovery)
                    elif not self.emergency_stop and not self.in_error_state:
                        self._update_ui_from_recovery_stage(self.recovery_stage)

        except Exception as e:
            print(f"Queue Processing Error: {e}")
            
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: C-레벨 메모리 세그폴트(Segmentation Fault) 원천 방어선
        ## 비상정지 락업 스크린이 뜬 상태(emergency_stop=True)라면 무한 after 루프를
        ## 완전히 끊어버려 파괴된 구형 위젯 메모리를 참조하다 튕기는 현상을 완벽 차단합니다.
        ## ---------------------------------------------------------------------
        if self.root.winfo_exists() and not self.emergency_stop:
            self._process_queue_after_id = self.root.after(50, self.process_queue)

    # 서버 메시지에 따라 단계 UI를 업데이트하는 함수
    def update_step_ui_by_server(self, current_step_name):
        matched_idx = None

        try:
            matched_idx = self.steps.index(current_step_name)
        except ValueError:
            # ★ 변경: 완료 단계부터 역순으로 검사 (구체적인/뒷단계 키워드 우선순위 높임)
            for idx, keywords in reversed(self.STEP_KEYWORD_MAP):
                if any(kw in current_step_name for kw in keywords):
                    matched_idx = idx
                    break

        if matched_idx is None:
            return

        # [백업 대비 수정] 복구 중 "재파지" 문구가 최초 통 파지 단계로 되돌리는 것을 막음
        if matched_idx < self._last_step_idx:
            return

        self._last_step_idx = matched_idx
        step_label = self.steps[matched_idx]
        progress = (matched_idx + 1) / len(self.steps)
        self.update_ui(matched_idx, step_label, progress)

        if matched_idx == len(self.steps) - 1:
            self.handle_process_complete()

    # [백업 대비 추가] motion_controller의 세부 RecoveryStage를 HMI 6단계로 변환
    # 문구 포함 여부가 아닌 명시적 체크포인트를 사용해 세척 중 재파지 등의 오인식을 해결
    def _step_index_for_recovery_stage(self, stage):
        """로봇 복구 체크포인트를 HMI의 6단계 진행 표시로 변환합니다."""
        if stage in {"IDLE", "READY", "BIN_PICKING", "BIN_PICK_FAILED", "BIN_GRASPED"}:
            return 0
        if stage in {"DUMP_STARTED", "DUMP_APPROACHED"}:
            return 1
        if stage in {"DUMP_TILTING", "DUMP_TILTED", "DUMP_SHAKING", "WASTE_DUMPED",
                     "DUMP_LEVELING", "DUMP_LEVELED"}:
            return 2
        if stage in {"WASH_APPROACHING", "WASH_WAYPOINT_TO_APPROACH", "WASH_APPROACH_REACHED",
                     "WASH_PLACING", "WASH_JIG_PLACED", "WASH_JIG_RELEASED", "WASH_JIG_LEAVING",
                     "WASH_JIG_LEFT", "WASH_TO_WAYPOINT", "WASH_WAYPOINT_REACHED",
                     "WATER_VALVE_APPROACHING", "WATER_VALVE_READY", "WATER_VALVE_GRASPING",
                     "WATER_VALVE_GRASPED", "WATER_VALVE_TURNING", "WATER_VALVE_TURNED",
                     "WATER_VALVE_RETURNING", "WATER_VALVE_RETURNED", "WATER_IN_BIN",
                     "WASH_BIN_GRASPED", "WASH_BIN_LIFTED", "WASH_BIN_AT_APPROACH",
                     "WASH_BIN_LEFT", "WATER_DUMPING", "WATER_DUMPED"}:
            return 3
        if stage in {"RETURNING_BIN", "BIN_RETURNED"}:
            return 4
        if stage == "COMPLETE":
            return 5
        return None

    def _update_ui_from_recovery_stage(self, stage):
        step = self._step_index_for_recovery_stage(stage)
        if step is None or step < self._last_step_idx:
            return
        if hasattr(self, "status") and self.status and self.status.winfo_exists():
            self._last_step_idx = step
            self.update_ui(step, self.steps[step], (step + 1) / len(self.steps))
            if step == len(self.steps) - 1:
                self._resume_session_active = False
                self.handle_process_complete()

    def _handle_safety_event(self, error_code, event_payload):
        """안전 이벤트를 종류별로 UI에 반영하고 ROBOT_STATUS 캐시 중복을 차단합니다."""
        if not error_code:
            return

        # [백업 대비 추가] 재개 요청 이전에 캐시된 안전 이벤트는 새 긴급정지로 처리하지 않음
        # 정상 완료 직후 과거 긴급정지 창이 다시 뜨던 문제를 방지
        event_timestamp = event_payload.get("timestamp", 0) if isinstance(event_payload, dict) else 0
        if isinstance(event_timestamp, (int, float)) and event_timestamp <= self._ignore_safety_before:
            return

        event_key = None
        if isinstance(event_payload, dict):
            event_key = (
                event_payload.get("timestamp"),
                event_payload.get("error_code", error_code),
                event_payload.get("error_msg", ""),
            )
        else:
            event_key = (None, error_code, str(event_payload))

        if event_key == self._last_handled_safety_event:
            return
        self._last_handled_safety_event = event_key
        self.in_error_state = True

        event_text = str(event_payload)
        if error_code == "ERR_COLLISION" or "COLLISION" in str(error_code).upper() or "충돌" in event_text or "충격" in event_text:
            self.show_fatal_error_screen(is_collision=True)
        elif error_code == "ERR_PICK":
            self.show_placement_error()
        else:
            self.show_fatal_error_screen(is_collision=False)

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
        """현재 메인 컨테이너 내부의 모든 유저 인터페이스 위젯 제거 (세그폴트 최전방 핵심 방어선)"""
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX 1: 유령 대기 after 루프들을 선제 박멸하여 파괴된 위젯 접근 원천 봉쇄
        ## ---------------------------------------------------------------------
        for after_id in self._active_after_ids:
            try:
                self.root.after_cancel(after_id)
            except:
                pass
        self._active_after_ids.clear()

        ## ---------------------------------------------------------------------
        ## CRITICAL FIX 2: 기존의 괄호 유실 등 문법 불안 정정 및 다이렉트 위젯 파괴 메커니즘 고도화
        ## ---------------------------------------------------------------------
        if hasattr(self, "main_container") and self.main_container.winfo_exists():
            for widget in self.main_container.winfo_children():
                try:
                    if widget.winfo_exists():
                        widget.pack_forget()
                        widget.place_forget()
                        widget.grid_forget()
                        widget.destroy()
                except Exception as e:
                    print(f"Widget destroy safe catch: {e}")
                    
        # 파괴된 포인터의 잔여 가비지 초기화
        self.status = None
        self.progress = None

    def _restart_process_queue(self):
        """비상정지로 끊긴 유저 UI 이벤트 큐 루프를 중복 없이 재시작합니다."""
        if not hasattr(self, "root") or not self.root.winfo_exists():
            return
        if self.emergency_stop or self._process_queue_after_id is not None:
            return
        self._process_queue_after_id = self.root.after(50, self.process_queue)

    def _safe_after(self, ms, command, *args):
        """메인 윈도우 인스턴스가 실재할 때만 예약을 안전하게 걸고 추적합니다."""
        if not hasattr(self, "root") or not self.root.winfo_exists():
            return None
        if self.emergency_stop and hasattr(command, "__name__") and command.__name__ == "<lambda>":
            return None
        after_id = self.root.after(ms, command, *args)
        self._active_after_ids.append(after_id)
        return after_id

    # ========================================================
    # 4. 화면 뷰 생성 및 공정 시퀀스 UI 시스템
    # VIEW 01: 인트로 시작 화면 (Intro UI)
    # ========================================================
    def create_intro_ui(self):
        self.in_error_state = False
        self.emergency_stop = False
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

        # 모드 선택을 취소하면 작업을 시작하지 않고 인트로 화면으로 돌아갑니다.
        mode_back_btn = ctk.CTkButton(
            self.main_container,
            text="이전으로",
            width=220,
            height=48,
            font=("맑은 고딕", 15, "bold"),
            fg_color="#3e475e",
            hover_color="#566176",
            text_color="#ffffff",
            corner_radius=12,
            command=self.create_intro_ui,
        )
        mode_back_btn.place(relx=0.5, rely=0.92, anchor="center")

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
            command=self.verify_and_start_process
        )
        next_btn.pack(pady=(0, 12))

        # 아직 START API를 호출하기 전이므로 선택한 모드를 다시 고를 수 있게 돌아갑니다.
        placement_back_btn = ctk.CTkButton(
            btn_frame,
            text="이전으로",
            width=240,
            height=48,
            font=("맑은 고딕", 14, "bold"),
            fg_color="#3e475e",
            hover_color="#566176",
            text_color="#ffffff",
            corner_radius=12,
            command=self.create_mode_selection_ui,
        )
        placement_back_btn.pack()

    def show_placement_error(self):
        error_message = (
            "▶ [배치 오류] 통이 감지되지 않았습니다! ◀\n"
            "━━━━━━━━━━━━━━━━━━━━━━\n\n"
            "지정된 위치에 수거통이 없거나 올바르게 밀착되지 않았습니다.\n"
            "수거통의 정렬 상태를 다시 확인한 후 확실하게 밀착시켜 주세요!"
        )
        msg_box = CTkMessagebox(title="⚠️ 수거통 배치 오류 안내", message=error_message, icon="warning", option_1="확인", corner_radius=12, width=500)
        response = msg_box.get()

        if response == "확인":
            self.is_running = False
            if hasattr(self, "home_btn") and self.home_btn and self.home_btn.winfo_exists():
                self.home_btn.pack(pady=0)

    def verify_and_start_process(self):
        """배치 완료 시 먼저 서버 연결을 시도하고, 성공하거나 통신이 유지될 때만 메인 UI를 빌드합니다."""
        # [백업 대비 수정] 이전 작업의 COMPLETE/복구 단계를 새 작업이 물려받지 않도록 초기화
        self.recovery_stage = "IDLE"
        self._received_recovery_stage = False
        self._last_step_idx = -1
        success = self.start(self.selected_mode)
        if success:
            self.create_process_ui()

    # ========================================================
    # VIEW 04: 실시간 공정 진행 화면 (Process Monitoring UI)
    # =======================================================
    def create_process_ui(self):
        self.clear_root()
        self.in_error_state = False
        self.step_labels = []
        self._last_step_idx = -1
        # [백업 대비 수정] START 성공 후 진행 화면 생성 과정에서 실행 상태가 False로 덮이던 문제를 고침
        self.is_running = True
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

        # [백업 대비 추가] 화면 생성 전에 도착한 최신 로봇 체크포인트를 즉시 진행 표시에 반영
        self.root.after(0, lambda: self._update_ui_from_recovery_stage(self.recovery_stage))
        

    # =======================================================
    # 5. 프로세스 제어 
    # ========================================================
    def start(self, mode):
        if self.is_running:
            return False
            
        self.is_running = True
        self.emergency_stop = False
        
        try:
            # api_user 내부에서 requests.post를 호출하여 받은 Response 객체 수집
            response = self.api_service.request_task_start(mode)
            
            # 1. 정상적으로 Response 수집 및 HTTP 200번대 안착 성공 케이스
            if response is not None and (200 <= response.status_code < 300):
                try:
                    res_data = response.json()
                    self.current_task_id = res_data.get("task_id")
                except Exception:
                    # 응답 포맷이 다를 경우 디버깅용 임의 가상 ID 매핑
                    self.current_task_id = f"task_{int(time.time())}"
                return True
                
            # 2. 서버 접속은 되었으나 백엔드 비즈니스 로직 에러 등의 사유인 케이스
            elif response is not None:
                print(f"서버 연결은 확인됨. 시스템 응답코드 이상: {response.status_code}")
                self.current_task_id = f"mock_{int(time.time())}"
                return True
                
            # 3. response가 완벽히 None인 경우 (api_base의 RequestException 캐치 블록 반환값)
            else:
                raise requests.RequestException("서버 응답 없음 (None)")
                
        except Exception as e:
            # ❌ 진짜 네트워크가 연결되지 않았거나 통신 오류인 상황에만 에러 출력 처리
            print(f"서버 물리 연결 실패: {e}")
            self.is_running = False
            
            CTkMessagebox(
                title="시스템 연결 오류", 
                message="❌ 로봇 제어 서버와 물리적으로 연결되지 않았습니다.\n네트워크 상태 및 서버 구동 여부를 다시 확인해 주세요.", 
                icon="cancel"
            )
            return False

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
        if self.emergency_stop or self.in_error_state:
            if hasattr(self, "home_btn") and self.home_btn and self.home_btn.winfo_exists():
                self.home_btn.pack(pady=0)
            return
        if not hasattr(self, "status") or not self.status.winfo_exists():
            return
        self.status.configure(text="모든 작업 완료", text_color="#00fa9a")

        # 관리자 콘솔 상태도 완료로 갱신 (백엔드 WS가 "작업 완료" 단계명을 보내도 동일하게 반영)
        if self.manager_console:
            self.manager_console.update_user_status("대기 중 (작업 완료)")
        self.is_running = False

        if hasattr(self, "error_btn") and self.error_btn and self.error_btn.winfo_exists():
            self.error_btn.pack_forget()

        if hasattr(self, "home_btn") and self.home_btn and self.home_btn.winfo_exists():
            self.home_btn.pack(pady=0)

    # ========================================================
    # 6. EMERGENCY LOCK SCREEN SYSTEM (비상 정지 전체 오버레이)
    # ========================================================

    def trigger_drop_error(self):
        """센서 단락 혹은 관리자 콘솔 긴급 중단 명령 패킷 수신 시 호출"""
        # 1. 즉시 상태 플래그를 변경하여 추가적인 run_process나 백그라운드 UI 업데이트 차단
        self.emergency_stop = True
        self.is_running = False

        # 2. 백엔드 DB에 에러 이력 기록
        #    error_code: 백엔드 tb_error_log 스펙 기준 (ERR_COLLISION)
        #    error_msg:  백엔드 ErrorLogRequest.error_msg 필드에 저장되는 상세 메시지
        if self.current_task_id:
            try:
                self.api_service.send_error_log(
                    self.current_task_id,
                    "ERR_COLLISION",          # ← 수정: DROP001 → 백엔드 스펙 코드
                    "충돌 감지: 비상 정지 인터록 가동"
                )
            except Exception as e:
                print(f"에러 로그 전송 실패: {e}")

        # 3. 관리자 콘솔 실시간 상태 텍스트 갱신
        if self.manager_console:
            try:
                self.manager_console.update_user_status("❌ 충돌 감지 — 비상 정지 인터록 가동")
            except Exception:
                pass

        # 4. 즉시 화면을 띄우지 않고 10ms 유예를 두어 메인 스레드에서 안전하게 렌더링
        self.root.after(10, self.show_fatal_error_screen)

    ## ---------------------------------------------------------------------
    ## CRITICAL FIX: 외력 충돌(is_collision) 상태에 맞춰 UI 문구 분기 및 API 연동
    ## ---------------------------------------------------------------------
    def show_fatal_error_screen(self, is_collision=False):
        """기존 자식 위젯을 해치지 않고 붉은색 강제 인터록 스크린 레이어를 최상단에 완전 오버레이"""
        if hasattr(self, "error_bg_frame") and self.error_bg_frame.winfo_exists():
            return

        # 함수 진입 즉시 플래그를 차단하여 process_queue의 무한 UI 접근 차단
        self.in_error_state = True
        self.emergency_stop = True
        self.is_running = False
        
        # 내부 상태 동기화 메시지 송신
        status_msg = "❌ 외력 충돌로 인한 시스템 셧다운" if is_collision else "🚨 사용자 수동 비상 정지 상태"
        self.handle_ws_message({"type": "PROCESS_STATE", "payload": status_msg})

        # 1. 꽉 채우는 다크 레드 비상 대피용 도화지 생성
        self.error_bg_frame = ctk.CTkFrame(self.root, fg_color="#4c1f24", corner_radius=0)
        self.error_bg_frame.place(relx=0, rely=0, relwidth=1, relheight=1)

        # 2. 비상 타이틀 컴포넌트
        lbl_emoji = ctk.CTkLabel(self.error_bg_frame, text="🛑" if is_collision else "🚨", font=("맑은 고딕", 80))
        lbl_emoji.pack(pady=(80, 10))

        display_title = "HARDWARE INTERLOCK CRITICAL DROP ERROR" if is_collision else "EMERGENCY STOP ACTIVATED"
        lbl_title = ctk.CTkLabel(
            self.error_bg_frame, 
            text=display_title, 
            font=("Consolas", 24, "bold"), 
            text_color="#ff4d6d"
        )
        lbl_title.pack(pady=10)

        # 3. 비상 상황 조치 내용 안내 본문 패널 분기 정의
        if is_collision:
            guide_text = (
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n"
                "안전 레이저 펜스 침범 또는 물리적 외력 충돌이 감지되었습니다.\n"
                "모든 하드웨어 동력이 전면 차단되었습니다 (Safe Mode 락업).\n\n"
                "현장 수거함 주변 안전을 정비하고 관리자 승인을 대기하십시오.\n\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            )
        else:
            guide_text = (
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n"
                "사용자가 현장 비상 중단 스위치를 눌러 즉시 정지시켰습니다.\n"
                "로봇 구동 모터의 전원이 즉각 차단되었으며 공정이 일시 홀딩됩니다.\n\n"
                "위험 요소를 제거한 뒤 관리자 콘솔을 통해 시스템 복구를 진행하십시오.\n\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            )
        
        lbl_guide = ctk.CTkLabel(
            self.error_bg_frame, 
            text=guide_text, 
            font=("맑은 고딕", 14, "bold"), 
            text_color="#ffffff",
            justify="center"
        )
        lbl_guide.pack(pady=20)

        # [백업 대비 수정] WASTE_DUMPED 전에는 HOME, 이후에는 RESUME 버튼으로 자동 분기
        # 기존에는 모든 긴급정지에서 시작 화면으로만 이동해 남은 공정을 이어갈 수 없었음
        can_resume = self._can_resume_after_dump()
        self.error_home_btn = ctk.CTkButton(
            self.error_bg_frame,
            text="동작 재개 (RESUME)" if can_resume else "처음으로 (HOME)",
            width=260,
            height=48,
            font=("맑은 고딕", 15, "bold"),
            fg_color="#1f7ecb",
            hover_color="#145a93",
            text_color="#ffffff",
            corner_radius=12,
            command=self._resume_after_emergency if can_resume else self._return_home_after_emergency
        )
        self.error_home_btn.pack(pady=(0, 15))

        # 4. ★ 수평 이동 버튼: 오직 외력 충돌(is_collision=True)이 발생한 경우만 렌더링되도록 격리
        if is_collision:
            self.lateral_move_btn = ctk.CTkButton(
                self.error_bg_frame,
                text="↔  수평 이동 요청 (Manual Axis Override)  ↔",
                width=340,
                height=52,
                font=("맑은 고딕", 15, "bold"),
                fg_color="#7a3a00",
                hover_color="#a05000",
                text_color="#ffcc66",
                border_color="#ffcc66",
                border_width=1,
                corner_radius=12,
                command=self._on_lateral_move_requested  # 하단 연동 메서드 호출
            )
            self.lateral_move_btn.pack(pady=15)

        # 5. 하단 고정 인터록 잠금 캡션 바
        lbl_status = ctk.CTkLabel(
            self.error_bg_frame, 
            text="🔒 SYSTEM LOCK / 원격 관리 권한 해제 대기 중...", 
            font=("맑은 고딕", 12, "italic"), 
            text_color="#a8b3c2"
        )
        lbl_status.pack(side="bottom", pady=25)

    # [백업 대비 추가: 긴급정지 복구 시스템]
    # 아래 함수들은 사용자 버튼과 관리자 초기화를 동일한 분기로 통합
    # HOME은 실제 IDLE 도착까지 긴급 화면에서 기다리고, RESUME은 이전 단계의 진행 바를 복원
    # 큐 정리와 처리 플래그를 함께 사용해 과거 이벤트 재처리 및 RESET 중복 호출도 방지
    def _can_resume_after_dump(self):
        """음식물 배출 완료 체크포인트부터만 남은 공정 재개를 허용합니다."""
        resumable_stages = {
            "WASTE_DUMPED", "DUMP_LEVELING", "DUMP_LEVELED", "WASH_APPROACHING",
            "WASH_WAYPOINT_TO_APPROACH", "WASH_APPROACH_REACHED", "WASH_PLACING",
            "WASH_JIG_PLACED", "WASH_JIG_RELEASED", "WASH_JIG_LEAVING", "WASH_JIG_LEFT",
            "WASH_TO_WAYPOINT", "WASH_WAYPOINT_REACHED", "WATER_VALVE_APPROACHING",
            "WATER_VALVE_READY", "WATER_VALVE_GRASPING", "WATER_VALVE_GRASPED",
            "WATER_VALVE_TURNING", "WATER_VALVE_TURNED", "WATER_VALVE_RETURNING",
            "WATER_VALVE_RETURNED", "WATER_IN_BIN", "WASH_BIN_GRASPED", "WASH_BIN_LIFTED",
            "WASH_BIN_AT_APPROACH", "WASH_BIN_LEFT", "WATER_DUMPING", "WATER_DUMPED",
            "RETURNING_BIN", "BIN_RETURNED",
        }
        return self.recovery_stage in resumable_stages

    def recover_from_admin(self):
        """관리자 초기화 요청도 사용자 긴급정지 버튼과 동일한 체크포인트 분기로 처리합니다."""
        if self._pending_recovery_action is not None:
            return
        if self._can_resume_after_dump():
            self._resume_after_emergency()
        else:
            self._return_home_after_emergency()

    def _discard_queued_events(self):
        """새 복구 요청 전에 쌓인 과거 상태와 안전 이벤트만 제거합니다."""
        while not self.event_queue.empty():
            try:
                self.event_queue.get_nowait()
            except queue.Empty:
                break

    def _return_home_after_emergency(self):
        """배출 전 긴급정지에서 RESET 복귀를 수행하고 실제 IDLE 도착 후 시작 화면을 표시합니다."""
        try:
            self.error_home_btn.configure(state="disabled", text="원위치 복귀 중...")
            self._discard_queued_events()
            self._ignore_safety_before = time.time()
            self._pending_recovery_action = "HOME"
            self.emergency_stop = False
            response = self.api_service.reset_robot_system()
            if response is None or not (200 <= response.status_code < 300):
                raise RuntimeError("백엔드가 원위치 복귀 요청을 거절했습니다.")
            self._restart_process_queue()
        except Exception as exc:
            self._pending_recovery_action = None
            self.emergency_stop = True
            self.error_home_btn.configure(state="normal", text="처음으로 (HOME)")
            CTkMessagebox(
                title="원위치 복귀 실패",
                message=str(exc),
                icon="cancel",
                option_1="확인",
            )

    def _finish_home_recovery(self):
        """motion_controller가 RESET 완료로 IDLE을 발행한 뒤에만 시작 화면으로 전환합니다."""
        self.in_error_state = False
        self.emergency_stop = False
        self.is_running = False
        self._last_step_idx = -1
        self.recovery_stage = "IDLE"
        self._received_recovery_stage = False
        if hasattr(self, "error_bg_frame") and self.error_bg_frame and self.error_bg_frame.winfo_exists():
            self.error_bg_frame.place_forget()
            self.error_bg_frame.destroy()
        self.create_intro_ui()
        self._restart_process_queue()
        if self.manager_console:
            self.manager_console.update_user_status("대기 중 (원위치 복귀 완료)")

    def _resume_after_emergency(self):
        """관리자 초기화와 같은 RESET 경로로 배출 이후의 남은 공정을 수행합니다."""
        try:
            self.error_home_btn.configure(state="disabled", text="재개 요청 중...")
            self._pending_recovery_action = "RESUME"
            self._resume_session_active = True
            self._discard_queued_events()
            self._ignore_safety_before = time.time()
            self.resume_recovery_stage = self.recovery_stage
            response = self.api_service.reset_robot_system()
            if response is None or not (200 <= response.status_code < 300):
                raise RuntimeError("백엔드가 동작 재개 요청을 거절했습니다.")
            self.resume_step_idx = self._step_index_for_recovery_stage(self.resume_recovery_stage)
            if self.resume_step_idx is None:
                self.resume_step_idx = self._last_step_idx
            self.root.after(100, self._show_resumed_process_ui)
        except Exception as exc:
            self.error_home_btn.configure(state="normal", text="동작 재개 (RESUME)")
            self._pending_recovery_action = None
            self._resume_session_active = False
            CTkMessagebox(
                title="동작 재개 실패",
                message=str(exc),
                icon="cancel",
                option_1="확인",
            )

    def _show_resumed_process_ui(self):
        """긴급정지 오버레이를 닫고 중단됐던 단계의 진행 화면을 복원합니다."""
        if not self.root.winfo_exists():
            return

        self._pending_recovery_action = None

        self.emergency_stop = False
        self.in_error_state = False

        if hasattr(self, "error_bg_frame") and self.error_bg_frame and self.error_bg_frame.winfo_exists():
            self.error_bg_frame.place_forget()
            self.error_bg_frame.destroy()

        step = max(0, min(getattr(self, "resume_step_idx", 0), len(self.steps) - 1))
        self.create_process_ui()
        self.is_running = True
        self._last_step_idx = step
        self.update_ui(step, self.steps[step], (step + 1) / len(self.steps))
        self.status.configure(text=f"{self.steps[step]} - 동작 재개 중", text_color="#ffffff")
        self._restart_process_queue()

    def _return_to_intro_from_error(self):
        """오류/긴급정지 화면에서 사용자 시작 화면으로 안전하게 복귀합니다."""
        self.in_error_state = False
        self.emergency_stop = False
        self.is_running = False
        self._last_step_idx = -1

        if hasattr(self, "error_bg_frame") and self.error_bg_frame and self.error_bg_frame.winfo_exists():
            try:
                self.error_bg_frame.place_forget()
                self.error_bg_frame.destroy()
            except Exception as e:
                print(f"Error overlay destroy failed: {e}")

        while not self.event_queue.empty():
            try:
                self.event_queue.get_nowait()
            except queue.Empty:
                break

        self._restart_process_queue()
        self.create_intro_ui()

    ## ---------------------------------------------------------------------
    ## CRITICAL FIX: 프론트엔드 UserAPI 매핑 규격에 맞춰 로봇 6축 제어 패킷 릴레이
    ## ---------------------------------------------------------------------
    def _on_lateral_move_requested(self):
        """에러 락업 상황에서 강제로 수평축 모터를 구동시키는 오버라이드 API 트랜잭션"""
        # api_user.py의 send_hardware_command가 요구하는 Pydantic 규격 (J1~J6)
        horizontal_joint_packet = {
            "J1": 0.0, 
            "J2": 0.0,
            "J3": 0.0,
            "J4": 0.0,
            "J5": 0.0,
            "J6": 0.0
        }
        
        try:
            # 프론트엔드 api_service 인스턴스를 통해 백엔드(/api/robot/move-joint)로 전송
            response = self.api_service.send_hardware_command(horizontal_joint_packet)
            
            if response and (response.status_code == 200 or response.status_code == 201):
                CTkMessagebox(
                    title="명령 전송 완료",
                    message="수평 이동 오버라이드 제어 명령이 로봇 액추에이터에 도달했습니다.",
                    icon="check",
                    option_1="확인"
                )
            else:
                raise Exception(f"하드웨어 거절 (Status Code: {response.status_code if response else 'No Response'})")
                
        except Exception as e:
            print(f"[FAIL] 수평 이동 제어 인터페이스 통신 에러: {e}")
            CTkMessagebox(
                title="제어 명령 실패",
                message=f"수평이동 명령을 전송하지 못했습니다:\n{str(e)}",
                icon="cancel",
                option_1="확인"
            )

    # ========================================================
    # 7. RECOVERY SYSTEM (관리자 콘솔 원격 복구 초기화 커넥터)
    # ========================================================
    def reset_system(self):
        """관리자 원격 복구"""
        # 현재 상태 백업
        self.was_running_before_reset = self.is_running
        # 현재 단계 저장
        self.resume_step_idx = self._last_step_idx
        self.emergency_stop = True
    
        # 관리자 창의 컴포넌트 파괴 스케줄러와 메모리 레이스가 발생하지 않도록 
        # 유예 마진을 150ms로 늘려서 완전히 격리 후 안전 리셋 진입
        self.root.after(150, self._safe_ui_reset)

    def _safe_ui_reset(self):

        if not self.root.winfo_exists():
            return

        self.emergency_stop = False

        while not self.event_queue.empty():
            try:
                self.event_queue.get_nowait()
            except queue.Empty:
                break

        self._restart_process_queue()

        # 붉은 화면 제거
        if hasattr(self, "error_bg_frame"):
            try:
                self.error_bg_frame.place_forget()
                self.error_bg_frame.destroy()
            except:
                pass

        self.lateral_move_btn = None

        step = getattr(self, "resume_step_idx", 0)

        # ★ 핵심 정책
        # 0~1단계 : 초반 → 홈 복귀
        # 2단계 이후 : 현재 위치 유지 후 재개

        if step <= 1:

            print("[RECOVERY] 초기 위치 복귀 후 재시작")

            # 실제 로봇 복구 모션은 백엔드 RESET -> motion_controller가 담당합니다.
            # HMI에서 별도 MOVE_JOINT를 보내면 안전 경유점 없이 다른 자세로 움직일 수 있습니다.
            self.root.after(
                500,
                self.create_process_ui
            )

        else:

            print(f"[RECOVERY] {self.steps[step]} 단계부터 재개")

            self.create_process_ui()

            # 이전 진행상태 복원
            progress=(step+1)/len(self.steps)

            self.update_ui(
                step,
                self.steps[step],
                progress
            )

            # 실제 재개 진행률은 백엔드 웹소켓 상태를 기준으로 갱신합니다.

    def resume_process(self):
        """중단된 이후 단계부터 재개"""

        self.is_running = True

        remaining_steps=self.steps[self.resume_step_idx:]

        total=len(self.steps)

        for i,step in enumerate(
            remaining_steps,
            start=self.resume_step_idx
        ):

            if self.emergency_stop:
                return

            progress=(i+1)/total

            self.update_ui(
                i,
                step,
                progress
            )

            time.sleep(2)

        self.handle_process_complete()
        
if __name__ == "__main__":
    FoodWasteGUI()