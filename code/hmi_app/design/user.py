import customtkinter as ctk
from CTkMessagebox import CTkMessagebox
import time
import queue

import requests

# 관리자 콘솔 클래스 임포트
from design.manager import ManagerGUI
from api.api_user import UserAPI

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
        """백엔드 웹소켓 메시지 큐를 안전하게 소모하여 UI 상태 전이를 처리하는 데몬 루프"""
        # 윈도우 창 자체가 이미 파괴되었다면 즉시 스케줄러 다운
        if not hasattr(self, "root") or not self.root.winfo_exists():
            return
            
        try:
            while not self.event_queue.empty():
                data = self.event_queue.get_nowait()
                msg_type = data.get("type")

                # ── PROCESS_STATE ──────────────────────────────────────────
                if msg_type == "PROCESS_STATE":
                    if self.emergency_stop:
                        continue  # 비상 정지 상태에서는 이미 위젯 레이아웃이 바뀌었으므로 무시

                    payload = data.get("payload", "")
                    if hasattr(self, "status") and self.status and self.status.winfo_exists():
                        try:
                            self.status.configure(text=payload)
                            self.update_step_ui_by_server(payload)
                        except Exception as e:
                            print(f"UI 업데이트 오류: {e}")

                # ── SAFETY_EVENT ────────────────────────────────────────────
                elif msg_type == "SAFETY_EVENT":
                    if not self.emergency_stop:   # 중복 트리거 방지
                        error_code = data.get("error_code", "")
                        print(f"[WS] SAFETY_EVENT 수신: code={error_code}", flush=True)
                        
                        # ★ 핵심 수정 분기: 오직 외력 충돌(ERR_COLLISION)일 때만 수평이동 버튼을 켬
                        if error_code == "ERR_COLLISION":
                            self.show_fatal_error_screen(is_collision=True)
                        else:
                            # 통 탈락, 파지 실패(ERR_PICK 등) 등의 일반 에러인 경우 수평이동 버튼 비활성화
                            self.show_fatal_error_screen(is_collision=False)

                # ── ROBOT_STATUS ────────────────────────────────────────────
                elif msg_type == "ROBOT_STATUS":
                    if not self.emergency_stop:
                        inner = data.get("payload", {})
                        if isinstance(inner, dict) and "last_safety_event" in inner:
                            last_event = inner.get("last_safety_event", "")
                            print(f"[WS] ROBOT_STATUS 안 last_safety_event 감지: {last_event}", flush=True)
                            
                            # ★ 핵심 수정 분기: 토픽 내부 이벤트 사유가 충돌(COLLISION)을 포함하는지 검사
                            if "COLLISION" in str(last_event).upper() or "충돌" in str(last_event) or "충격" in str(last_event):
                                self.show_fatal_error_screen(is_collision=True)
                            else:
                                self.show_fatal_error_screen(is_collision=False)

        except Exception as e:
            print(f"Queue Processing Error: {e}")
            
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: C-레벨 메모리 세그폴트(Segmentation Fault) 원천 방어선
        ## 비상정지 락업 스크린이 뜬 상태(emergency_stop=True)라면 무한 after 루프를
        ## 완전히 끊어버려 파괴된 구형 위젯 메모리를 참조하다 튕기는 현상을 완벽 차단합니다.
        ## ---------------------------------------------------------------------
        if self.root.winfo_exists() and not self.emergency_stop:
            self.root.after(50, self.process_queue)

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

        if matched_idx < self._last_step_idx and matched_idx != 0:
            return

        self._last_step_idx = matched_idx
        step_label = self.steps[matched_idx]
        progress = (matched_idx + 1) / len(self.steps)
        self.update_ui(matched_idx, step_label, progress)

        if matched_idx == len(self.steps) - 1:
            self.handle_process_complete()

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
        next_btn.pack(side="left", padx=12)

    def show_placement_error(self):
        error_message = (
            "▶ [배치 오류] 통이 감지되지 않았습니다! ◀\n"
            "━━━━━━━━━━━━━━━━━━━━━━\n\n"
            "지정된 위치에 수거통이 없거나 올바르게 밀착되지 않았습니다.\n"
            "수거통의 정렬 상태를 다시 확인한 후 확실하게 밀착시켜 주세요!"
        )
        CTkMessagebox(title="⚠️ 수거통 배치 오류 안내", message=error_message, icon="warning", option_1="확인", corner_radius=12, width=500)

    def verify_and_start_process(self):
        """배치 완료 시 먼저 서버 연결을 시도하고, 성공하거나 통신이 유지될 때만 메인 UI를 빌드합니다."""
        success = self.start(self.selected_mode)
        if success:
            self.create_process_ui()

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
        # self.start(self.selected_mode)

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
        if not hasattr(self, "status") or not self.status.winfo_exists():
            return
        self.status.configure(text="모든 작업 완료", text_color="#00fa9a")

        # 관리자 콘솔 상태도 완료로 갱신 (백엔드 WS가 "작업 완료" 단계명을 보내도 동일하게 반영)
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
        # 수평 이동 버튼 참조 해제 (error_bg_frame 파괴로 이미 소멸되지만 명시적 정리)
        self.lateral_move_btn = None
            
        if self.manager_console:
            self.manager_console.update_user_status("대기 중")
            
        # 프레임 교체 도중 사이드바 클릭 등의 충돌을 막기 위해 after 유예를 한번 더 주고 화면 재빌드
        self.root.after(50, self.create_mode_selection_ui)


if __name__ == "__main__":
    FoodWasteGUI()