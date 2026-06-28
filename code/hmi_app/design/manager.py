import customtkinter as ctk
from CTkMessagebox import CTkMessagebox
from datetime import datetime
import os

import requests
from hmi_app.api.api_manager import AdminAPI
import queue

class ManagerGUI:
    # ★ J3, J5의 초기 각도를 90도로 지정 (스타트 자세)
    DEFAULT_JOINT_ANGLES = {
        "J3 (Elbow)": 90,
        "J5 (Wrist 2)": 90,
    }
    DEFAULT_JOINT_ANGLES_SHORT = {
        "J3": 90.0,
        "J5": 90.0,
    }

    # =========================================================================
    # [SECTION 1] 초기화 및 창 설정
    # =========================================================================
    def __init__(self, reset_callback, main_gui_instance):
        """관리자 모니터링 콘솔 메인 초기화"""
        self.reset_callback = reset_callback
        self.main_gui = main_gui_instance
        self.api = AdminAPI() # API 통신 인스턴스

        self.window = ctk.CTkToplevel(self.main_gui.root)
        self.event_queue = queue.Queue()
        
        self._manager_after_ids = []  # 등록된 비동기 스케줄러(after) ID들을 추적하여 파괴 시 취소하기 위함
        self._ui_lock = False         # 탭 전환(기존 위젯 destroy) 중 백엔드 큐가 위젯을 건드리지 못하게 차단하는 락
        self.window.after(50, self.process_queue)

        # 웹소켓 메시지를 큐에 담아 처리하도록 리스너 시작
        # 아래처럼 유저랑 매니저 두 번 웹소켓 호출하고 있음 
        # self.api.start_websocket_listener(self.handle_ws_message)
    
        self.window.title("관리자 실시간 통합 모니터링 시스템 v1.2")
        
        # 윈도우 레이어 포커스 우선순위 설정
        self.window.lift()
        self.window.attributes("-topmost", True)
        self.window.after(150, lambda: self.window.attributes("-topmost", False))
        
        # 해상도 오프셋 및 화면 배치 계산
        self._setup_window_geometry()

        # 시스템 내부 데이터 및 상태 버퍼 초기화
        self._init_system_variables()

        # 메인 아키텍처 프레임 뼈대 생성
        self._build_layout_skeleton()

        # 좌측 메뉴 바 렌더링 및 첫 탭 자동 로드
        self.build_sidebar_menu()
        self.window.after(60, lambda: self.show_tab("진행 상태 모니터링"))
        
        self.save_error_log("관리자 실시간 제어 콘솔 가동 완료")
        # 관리자 창이 강제로 닫힐 때 안전하게 위젯 자원을 소멸시키는 프로토콜 바인딩
        self.window.protocol("WM_DELETE_WINDOW", self._on_safe_close_window)

    ## ---------------------------------------------------------------------
    ## CRITICAL FIX: 안전한 after 스케줄러 등록 및 일괄 취소 메서드 신설
    ## ---------------------------------------------------------------------
    def _safe_after(self, ms, command, *args):
        """창이 존재할 때만 after를 등록하고, 관리자용 after 리스트에 저장합니다."""
        if not hasattr(self, "window") or not self.window.winfo_exists():
            return None
        after_id = self.window.after(ms, command, *args)
        self._manager_after_ids.append(after_id)
        return after_id

    def _clear_all_after_events(self):
        """창 소멸 시, 남아있는 모든 유령 after 이벤트들을 완전 박멸하여 메모리 참조 오류를 막습니다."""
        for after_id in self._manager_after_ids:
            try:
                self.window.after_cancel(after_id)
            except:
                pass
        self._manager_after_ids.clear()
    ## ---------------------------------------------------------------------

    def _on_safe_close_window(self):
        """윈도우 파괴 시 발생할 수 있는 레이스 컨디션을 완전히 제거하는 안전 종료 디스트럭터"""
        try:
            ## ---------------------------------------------------------------------
            ## CRITICAL FIX: 파괴 즉시 UI 조작 락을 걸고 스케줄러부터 제거 (가장 중요)
            ## ---------------------------------------------------------------------
            self._ui_lock = True
            self._clear_all_after_events() 
            
            while not self.event_queue.empty():
                try: self.event_queue.get_nowait()
                except: break
            
            if self.main_gui and hasattr(self.main_gui, 'manager_console'):
                self.main_gui.manager_console = None
                
            if self.window.winfo_exists():
                self.window.destroy()
        except Exception as e:
            print(f"Manager close safe error: {e}")

    # ===========================================================================
    # [SECTION 2] 이벤트 / 웹소켓 처리 
    # ===========================================================================
    def handle_ws_message(self, data):
        self.event_queue.put(data)

    def process_queue(self):
        """내부 메시지 큐 소모 및 UI 안전 상태 전이 동기화 핸들러 (50ms 주기 반복)"""
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: UI 락 상태이거나 창이 폭파 중일 땐 작업을 수행하지 않고 탈출하여 세그폴트 방지
        ## ---------------------------------------------------------------------
        if self._ui_lock or not hasattr(self, "window") or not self.window.winfo_exists():
            return

        try:
            while not self.event_queue.empty():
                data = self.event_queue.get_nowait()
                if data.get("type") == "PROCESS_STATE":
                    new_status = data.get("payload")
                    if self.window.winfo_exists() and not self._ui_lock:
                        self.update_user_status(new_status)
        except Exception as e:
            print(f"Queue processing soft warning: {e}")
        
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: 스케줄러 재등록 시 기존 after_id 리스트 관리 매개변수 사용
        ## ---------------------------------------------------------------------
        if self.window.winfo_exists() and not self._ui_lock:
            self._safe_after(50, self.process_queue)

    def _setup_window_geometry(self):
        """화면 해상도 분석 및 관리자 창 위치 우측 최적화 배치"""
        win_width, win_height = 1050, 700
        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()
        
        pos_x = int((screen_width * 0.5) + 20)
        pos_y = int((screen_height - win_height) / 2)
        
        if pos_x + win_width > screen_width:
            pos_x = screen_width - win_width - 20

        self.window.geometry(f"{win_width}x{win_height}+{pos_x}+{pos_y}")
        self.window.configure(fg_color="#1a1d24")

    def _init_system_variables(self):
        """파일 경로 및 조작용 글로벌 딕셔너리 버퍼 생성"""
        self.log_file_path = "robot_error_logs.txt"
        self.cycle_count_file = "total_cycles.txt"
        self.total_cycles = 0
        self.load_cycle_count()

        self.current_user_status_text = "대기 중 (사용자 이용 전)"
        self.menu_buttons = {}
        self.joint_sliders = {}
        self.joint_entries = {}

    def _build_layout_skeleton(self):
        """좌측 고정 사이드바와 우측 동적 콘텐츠 컨테이너 영역 분리"""
        self.sidebar_frame = ctk.CTkFrame(self.window, width=230, fg_color="#20242f", corner_radius=0)
        self.sidebar_frame.pack(side="left", fill="y")
        self.sidebar_frame.pack_propagate(False)

        self.main_content_area = ctk.CTkFrame(self.window, fg_color="transparent")
        self.main_content_area.pack(side="right", fill="both", expand=True)

    # =========================================================================
    # [SECTION 3] 화면 라우팅 컨트롤
    # =========================================================================
    def build_sidebar_menu(self):
        """좌측 제어 대시보드 메뉴 버튼 빌드"""
        for widget in self.sidebar_frame.winfo_children():
            widget.destroy()

        sb_title = ctk.CTkLabel(self.sidebar_frame, text="ROBOT CONTROL PANEL", font=("DejaVu Sans Mono", 16, "bold"), text_color="#4fa3e3")
        sb_title.pack(pady=(35, 35))

        menus = [
            ("진행 상태 모니터링", "MONITOR"),
            ("안전 시스템 관리", "SAFETY"),
            ("하드웨어 수동 제어", "ROBOT"),
            ("시스템 로그 관리", "LOG")
        ]

        for menu_name, icon in menus:
            btn = ctk.CTkButton(
                self.sidebar_frame,
                text=f"  {icon}  {menu_name}",
                font=("NanumGothic", 13, "bold"),
                height=48,
                fg_color="transparent",
                text_color="#a8b3c2",
                hover_color="#2d3343",
                anchor="w",
                corner_radius=8,
                command=lambda name=menu_name: self.show_tab(name)
            )
            btn.pack(fill="x", padx=15, pady=4)
            self.menu_buttons[menu_name] = btn

    def show_tab(self, tab_name):
        """가상 프레임 단위의 화면 전환을 담당하는 뷰 스위칭 라우터 (안전성 대폭 강화)"""
        if not self.window.winfo_exists():
            return
            
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: 위젯 파괴 작업 도중 백엔드after나 스레드가 위젯을 건드리지 못하게 차단
        ## ---------------------------------------------------------------------
        self._ui_lock = True 
        
        for name, btn in self.menu_buttons.items():
            if btn.winfo_exists():
                btn.configure(
                    fg_color="#1f7ecb" if name == tab_name else "transparent", 
                    text_color="#ffffff" if name == tab_name else "#a8b3c2"
                )

        # 자식 위젯을 돌며 물리적으로 완전 제거
        for widget in self.main_content_area.winfo_children():
            try:
                if widget.winfo_exists():
                    widget.pack_forget()
                    widget.place_forget()
                    widget.grid_forget()
                    widget.destroy()
            except Exception as e:
                print(f"Tab widget destroy soft catch: {e}")

        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: 완전히 파괴된 UI 컴포넌트의 파이썬 바인딩 포인터를 확실하게 날림
        ## ---------------------------------------------------------------------
        self.lbl_monitor_status = None
        self.remote_stop_btn = None
        self.lbl_counter = None

        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: 정리가 모두 끝났으므로 비동기 갱신 허용(락 해제)
        ## ---------------------------------------------------------------------
        self._ui_lock = False 

        tab_routers = {
            "진행 상태 모니터링": self.view_progress_monitoring,
            "안전 시스템 관리": self.view_safety_management,
            "하드웨어 수동 제어": self.view_hardware_override,
            "시스템 로그 관리": self.view_log_management
        }
        
        if tab_name in tab_routers:
            tab_routers[tab_name]()

    # =========================================================================
    # [SECTION 4] 독립 UI 뷰 렌더, 화면 
    # =========================================================================
    def view_progress_monitoring(self):
        """TAB 1: 로봇 공정 실시간 진행 대시보드"""
        title = ctk.CTkLabel(self.main_content_area, text="MONITOR 로봇 공정 실시간 진행 상태", font=("NanumGothic", 22, "bold"), text_color="#ffffff")
        title.pack(anchor="w", padx=35, pady=(30, 20))

        # 1. 상태 레이블 블록
        card = ctk.CTkFrame(self.main_content_area, fg_color="#2d3343", corner_radius=12)
        card.pack(fill="x", padx=35, pady=10)

        lbl_sub = ctk.CTkLabel(card, text="현재 사용자 단말 구동 상태", font=("NanumGothic", 12), text_color="#4fa3e3")
        lbl_sub.pack(anchor="w", padx=20, pady=(15, 5))

        self.lbl_monitor_status = ctk.CTkLabel(card, text=self.current_user_status_text, font=("NanumGothic", 22, "bold"), text_color="#00fa9a")
        self.lbl_monitor_status.pack(anchor="w", padx=20, pady=(0, 20))
        self.update_user_status(self.current_user_status_text)

        # 2. 원격 긴급 제어 블록
        is_error = any(word in self.current_user_status_text for word in ["오류", "중단", "탈락"])
        emergency_panel = ctk.CTkFrame(self.main_content_area, fg_color="#1a1d24", border_width=1, border_color="#3e475e" if not is_error else "#ff4d6d", corner_radius=12)
        emergency_panel.pack(fill="x", padx=35, pady=15)
        
        lbl_panel_title = ctk.CTkLabel(emergency_panel, text="CONTROL 원격 시스템 제어 및 인터록 권한", font=("NanumGothic", 12, "bold"), text_color="#a8b3c2")
        lbl_panel_title.pack(anchor="w", padx=20, pady=(15, 8))
        
        self.remote_stop_btn = ctk.CTkButton(
            emergency_panel,
            text="EMERGENCY 원격 긴급 비상 정지 (Remote Emergency Stop)", height=50, font=("NanumGothic", 14, "bold"),
            fg_color="#d9534f" if not is_error else "#5c1e24", hover_color="#c9302c", text_color="#ffffff", corner_radius=10,
            state="normal", command=self.execute_remote_emergency_stop
        )
        self.remote_stop_btn.pack(fill="x", padx=20, pady=(0, 20))

        # 3. 장비 통계 블록
        stats_card = ctk.CTkFrame(self.main_content_area, fg_color="#2d3343", corner_radius=12)
        stats_card.pack(fill="both", expand=True, padx=35, pady=(10, 30))

        lbl_stats_title = ctk.CTkLabel(stats_card, text="STATUS 누적 장비 가동 통계", font=("NanumGothic", 13, "bold"), text_color="#ffffff")
        lbl_stats_title.pack(anchor="w", padx=20, pady=(15, 10))

        self.lbl_counter = ctk.CTkLabel(stats_card, text=f"총 누적 배출 공정 횟수 :   {self.total_cycles} 회", font=("DejaVu Sans Mono", 16, "bold"), text_color="#00fa9a")
        self.lbl_counter.pack(anchor="w", padx=25, pady=10)

    def view_safety_management(self):
        """TAB 2: 안전 인터록 대응 및 시스템 조치 복구 스크린"""
        title = ctk.CTkLabel(self.main_content_area, text="SAFETY 안전 제어 및 시스템 복구 조작", font=("NanumGothic", 22, "bold"), text_color="#ffffff")
        title.pack(anchor="w", padx=30, pady=(30, 20))

        is_error = any(word in self.current_user_status_text for word in ["오류", "중단", "탈락"])
        card_bg_col = "#4c1f24" if is_error else "#2d3343"
        border_col = "#ff4d6d" if is_error else "#3e475e"
        title_col = "#ff4d6d" if is_error else "#4fa3e3"
        title_text = "EMERGENCY 긴급 시스템 잠금 활성화 (비상 인터록 가동 중)" if is_error else "SAFETY 시스템 안전 상태 보장됨"

        safety_frame = ctk.CTkFrame(self.main_content_area, fg_color=card_bg_col, corner_radius=12, border_width=2, border_color=border_col)
        safety_frame.pack(fill="both", expand=True, padx=30, pady=(10, 30))

        lbl_sec_title = ctk.CTkLabel(safety_frame, text=title_text, font=("NanumGothic", 16, "bold"), text_color=title_col)
        lbl_sec_title.pack(anchor="w", padx=20, pady=(20, 10))

        status_guide = (
            "WARNING [CRITICAL EMERGENCY INTERLOCK]\n\n"
            "사용자 단말기 측에서 '수거통 탈락' 혹은 '원격 강제 중단' 신호가 수신되었습니다.\n"
            "공정이 강제 중단된 상태입니다.\n\n"
            "원격 제어 조치 사항:\n"
            "1. '하드웨어 수동 제어' 탭으로 이동하여 로봇 관절 상태를 수동 확인\n"
            "2. 현장 안전이 확보되면 하단의 리셋(Recovery) 버튼을 눌러 인터록을 해제하십시오."
        ) if is_error else (
            "[정상 가동 안내]\n\n"
            "OK 현재 메인 시스템 및 로봇 제어 신호가 정상 범주 내에서 유지되고 있습니다.\n"
        )
        info_box = ctk.CTkLabel(
            safety_frame,
            text=status_guide,
            height=160,
            fg_color="#1a1d24",
            corner_radius=6,
            font=("NanumGothic", 13),
            text_color="#ff4d6d" if is_error else "#cbd3dc",
            justify="left",
            anchor="w",
            wraplength=780,
        )
        info_box.pack(padx=20, pady=10, fill="x")

        reset_system_btn = ctk.CTkButton(
            safety_frame, text="RESET 인터록 해제 및 시스템 완전 초기화 (Emergency Recovery)", height=55, font=("NanumGothic", 14, "bold"),
            fg_color="#00fa9a", hover_color="#00c77b", text_color="#14171c", corner_radius=10, command=self.execute_system_recovery
        )
        reset_system_btn.pack(fill="x", padx=20, pady=25, side="bottom")

    def view_hardware_override(self):
        """TAB 3: 협동로봇 무인 6축 수동 조작 및 서보 관절 오버라이드 패널"""
        title = ctk.CTkLabel(self.main_content_area, text="ROBOT M0609 협동로봇 관절별 수동 제어 (Joint Override)", font=("NanumGothic", 22, "bold"), text_color="#ffffff")
        title.pack(anchor="w", padx=30, pady=(25, 15))

        scroll_frame = ctk.CTkScrollableFrame(self.main_content_area, fg_color="#2d3343", corner_radius=12)
        scroll_frame.pack(fill="both", expand=True, padx=30, pady=(0, 20))

        joint_specs = [
            ("J1 (Base)", -180, 180, "정면 기준 좌우 180°"), ("J2 (Shoulder)", -360, 360, "전후 무제한 급 가동"),
            ("J3 (Elbow)", -160, 160, "상하 굴곡 제어"), ("J4 (Wrist 1)", -360, 360, "요골/척골 회전 대체"),
            ("J5 (Wrist 2)", -160, 160, "그리퍼 꺾임 각도"), ("J6 (Wrist 3)", -360, 360, "그리퍼 무한 회전축")
        ]
        self.joint_sliders.clear()
        self.joint_entries.clear()


        for name, min_val, max_val, desc in joint_specs:
            j_frame = ctk.CTkFrame(scroll_frame, fg_color="#1a1d24", corner_radius=8)
            j_frame.pack(fill="x", padx=10, pady=5)
            lbl_title = ctk.CTkLabel(j_frame, text=name, font=("DejaVu Sans Mono", 13, "bold"), text_color="#4fa3e3", width=90, anchor="w")
            lbl_title.pack(side="left", padx=(15, 5), pady=8)
            lbl_desc = ctk.CTkLabel(j_frame, text=f"({desc})", font=("NanumGothic", 11), text_color="#7a889b", width=160, anchor="w")
            lbl_desc.pack(side="left", padx=5)
            slider = ctk.CTkSlider(j_frame, from_=min_val, to=max_val, number_of_steps=max_val-min_val, progress_color="#1f7ecb")
            slider.pack(side="left", fill="x", expand=True, padx=15)

            initial_value = self.DEFAULT_JOINT_ANGLES.get(name, 0)   # ★ J3, J5는 90, 나머지는 0
            slider.set(initial_value)
            print(f"### DEBUG {name}: initial_value={initial_value}, slider.get()={slider.get()}", flush=True)
            self.joint_sliders[name] = slider
            entry = ctk.CTkEntry(j_frame, width=65, font=("DejaVu Sans Mono", 11), justify="center", fg_color="#2d3343", border_color="#3e475e")
            entry.pack(side="left", padx=(5, 15))
            entry.insert(0, f"{initial_value:.1f}")   # ★ entry도 동일한 초기값으로 표시
            self.joint_entries[name] = entry
            slider.configure(command=lambda val, e=entry: self._sync_slider_to_entry(val, e))
            entry.bind("<Return>", lambda event, s=slider, e=entry, mn=min_val, mx=max_val: self._sync_entry_to_slider(s, e, mn, mx))
            # 액추에이터 및 원점 복구 하단 패널 프레임 구성
        self._render_hardware_tool_zone(scroll_frame)

    # ========================================================================
    # [SECTION 5] UI 컴포넌트 및 하드웨어 제어 로직
    # ========================================================================
    def _render_hardware_tool_zone(self, parent_frame):
        """수동 제어 하단 액추에이터 제어 공통 버튼 콤포넌트 렌더러"""
        tool_frame = ctk.CTkFrame(parent_frame, fg_color="#20242f", corner_radius=10, border_width=1, border_color="#3e475e")
        tool_frame.pack(fill="x", padx=10, pady=(15, 10))

        lbl_tool = ctk.CTkLabel(tool_frame, text="CONTROL END EFFECTOR & HARDWARE ACTION", font=("DejaVu Sans Mono", 12, "bold"), text_color="#00fa9a")
        lbl_tool.pack(anchor="w", padx=15, pady=(12, 6))

        btn_zone = ctk.CTkFrame(tool_frame, fg_color="transparent")
        btn_zone.pack(fill="x", padx=15, pady=(0, 15))

        ctk.CTkButton(btn_zone, text="OPEN 그리퍼 개방 (OPEN)", height=42, fg_color="#333b4c", hover_color="#1f7ecb", font=("NanumGothic", 12, "bold"), command=lambda: self.control_hardware_action("그리퍼 OPEN")).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ctk.CTkButton(btn_zone, text="CLOSE 그리퍼 파지 (CLOSE)", height=42, fg_color="#333b4c", hover_color="#ff4d6d", font=("NanumGothic", 12, "bold"), command=lambda: self.control_hardware_action("그리퍼 CLOSE")).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ctk.CTkButton(btn_zone, text="SEND 관절각 일괄 전송 (MoveJ)", height=42, fg_color="#1f7ecb", hover_color="#145a93", font=("NanumGothic", 12, "bold"), command=self.send_all_joints_command).pack(side="left", expand=True, fill="x", padx=(0, 6))
        ctk.CTkButton(btn_zone, text="RESET 로봇 원상복구 (Go Home)", height=42, fg_color="#d9534f", hover_color="#c9302c", font=("NanumGothic", 12, "bold"), command=self.reset_all_joints).pack(side="left", expand=True, fill="x")

    def view_log_management(self):
        """TAB 4: 로컬 디스크 에러 로깅 기록 트래킹 보드"""
        title = ctk.CTkLabel(self.main_content_area, text="LOG 최근 발생 시스템 에러 로그 조회", font=("NanumGothic", 22, "bold"), text_color="#ffffff")
        title.pack(anchor="w", padx=30, pady=(30, 20))

        log_frame = ctk.CTkFrame(self.main_content_area, fg_color="#2d3343", corner_radius=12)
        log_frame.pack(fill="both", expand=True, padx=30, pady=(10, 30))

        self.log_textbox = ctk.CTkTextbox(log_frame, fg_color="#1a1d24", border_color="#3e475e", border_width=1, text_color="#cbd3dc", font=("DejaVu Sans Mono", 11))
        self.log_textbox.pack(fill="both", expand=True, padx=20, pady=20)
        self.refresh_log_display()

    # =========================================================================
    # [SECTION 6] 하드웨어 통신 및 동기화 로직
    # =========================================================================
    def update_user_status(self, status_text):
        """실시간 진행 큐로부터 가공된 상태 문자열을 넘겨받아 안전성 보장 하에 UI 전이"""
        ## ---------------------------------------------------------------------
        ## CRITICAL FIX: UI 소멸 작업(락 상태) 도중 접근 시도 시, 즉시 무시하여 C-레벨 Crash 사전에 방지
        ## ---------------------------------------------------------------------
        if self._ui_lock: 
            return
            
        self.current_user_status_text = status_text
        
        if getattr(self, "lbl_monitor_status", None) is not None:
            try:
                if self.lbl_monitor_status.winfo_exists():
                    if any(word in status_text for word in ["이용 중", "처리 중", "구동 중"]):
                        self.lbl_monitor_status.configure(text=status_text, text_color="#1f7ecb")
                    elif any(word in status_text for word in ["오류", "중단", "탈락", "위험"]):
                        self.lbl_monitor_status.configure(text=status_text, text_color="#ff4d6d")
                    else:
                        self.lbl_monitor_status.configure(text=status_text, text_color="#00fa9a")
            except Exception as e:
                print(f"update_user_status 위젯 접근 실패 무시 (메모리 파괴 레이스): {e}", flush=True)

    def execute_remote_emergency_stop(self):
        """[수정] 관리자 비상 정지 시 백엔드에 즉시 통보"""
        self.save_error_log("[MANUAL_OVERRIDE] 관리자 비상 정지")

        self.api.send_emergency_stop(task_id=getattr(self.main_gui, 'current_task_id', None))
        # 2. 로컬 UI 제어
        if self.main_gui:
            self.main_gui.emergency_stop = True
            self.main_gui.is_running = False
            self.main_gui.root.after(50, lambda: self.main_gui.trigger_drop_error() if hasattr(self.main_gui, "trigger_drop_error") else None)
        self.window.after(100, lambda: self._safe_execute_error_alert())

    def trigger_admin_error_alert(self, error_msg):
        """유저 측 단말 수거통 탈락 감지 신호 바인딩 리다이렉터"""
        self.save_error_log(f"[CRITICAL_ALERT] {error_msg}")
        self.window.after(100, lambda: self._safe_execute_error_alert())

    def _safe_execute_error_alert(self):
        if not self.window.winfo_exists(): return
        self.show_tab("안전 시스템 관리")
        self.window.focus_force()

    def reset_all_joints(self):
        """협동로봇 Home 위치 복귀"""
        try:
            # Home 위치 데이터 (기존 로봇 위치와 상관 없는지 확인 필요)
            joint_angles = {
                "J1": 0.0,
                "J2": 0.0,
                "J3": self.DEFAULT_JOINT_ANGLES_SHORT.get("J3", 0.0),
                "J4": 0.0,
                "J5": self.DEFAULT_JOINT_ANGLES_SHORT.get("J5", 0.0),
                "J6": 0.0
            }
            # 서버 전송
            response = self.api.send_hardware_command(joint_angles)
            # 서버 응답 확인
            # 통신 실패
            if response is None:
                raise Exception(
                    "백엔드 서버와 연결할 수 없습니다."
                )
            # 서버 에러
            if not response.ok:
                raise Exception(
                    f"서버 응답 오류 : {response.status_code}"
                )
            
            # 서버 성공했을 때만 UI 갱신
            for name, slider in self.joint_sliders.items():
                if slider.winfo_exists():
                    slider.set(self.DEFAULT_JOINT_ANGLES.get(name, 0.0))   # ★ name으로 J3/J5만 90 적용
            for name, entry in self.joint_entries.items():
                if entry.winfo_exists():
                    value = self.DEFAULT_JOINT_ANGLES.get(name, 0.0)        # ★ 동일하게 적용
                    entry.delete(0, "end")
                    entry.insert(0, f"{value:.1f}")

            self.save_error_log("[RECOVERY_ACTION] Home Position 이동 성공")
            CTkMessagebox(
                title="원상복구 완료",
                message="OK 로봇이 Home 위치로 복귀했습니다.",
                icon="check",
                corner_radius=12
            )

        except Exception as e:
            self.save_error_log(f"[ERROR] {str(e)}")
            CTkMessagebox(
                title="원상복구 실패",
                message=f"ERROR {str(e)}",
                icon="cancel",
                corner_radius=12
            )
    # ===========================================================================
    # [SECTION 7] 하드웨어 제어 명령 송신
    # ===========================================================================
    def send_all_joints_command(self):
        # 1. 데이터 준비
        joint_angles = {k.split(" ")[0]: round(s.get(), 1) for k, s in self.joint_sliders.items()}
        
        # 2. 서버 전송
        response = self.api.send_hardware_command(joint_angles)
        
        # 3. 서버 응답 확인 
        if response is None or not response.ok:
            CTkMessagebox(title="통신 오류", message="ERROR 백엔드 서버와 연결할 수 없습니다.", icon="cancel")
            return

        # 4. UI 및 로그 피드백 업데이트
        self.save_error_log(f"[MANUAL] MoveJ 관절 명령 전송 완료: {joint_angles}")
       
        def safe_ui_update():
            if not self.window.winfo_exists(): return
            if hasattr(self, "lbl_monitor_status") and self.lbl_monitor_status.winfo_exists():
                self.lbl_monitor_status.configure(text="CONTROL 수동 관절 제어 명령 수행 중", text_color="#f1c40f")
            
            CTkMessagebox(
                title="전송 완료",
                message="OK 관절각 데이터가 로봇 서버로 일괄 전송되었습니다.",
                icon="check",
                option_1="확인",
                corner_radius=12
            )
        
        self.window.after(10, safe_ui_update)

    def control_hardware_action(self, action_name):
        """[수정] 하드코딩된 requests를 제거하고 구조화된 UserAPI를 활용하도록 변경"""
        print(f"### DEBUG control_hardware_action 호출됨: action_name={action_name!r}", flush=True)
        base_angle = round(self.joint_sliders["J1 (Base)"].get(), 1) if "J1 (Base)" in self.joint_sliders else 0.0

        try:
            # 관리자 전용 API 인스턴스에 그리퍼 제어 함수가 없거나 통합을 위해 임시 세팅 시
            # 만약 UserAPI 전용이라면 self.main_gui.api_service를 활용해도 좋습니다.
            response = self.api.send_gripper_command(action_name, base_angle)
            
            if response is None or response.status_code != 200:
                raise Exception("서버 응답 오류 또는 연결 실패")

            self.save_error_log(f"[MANUAL] 제어 성공: {action_name}")
            CTkMessagebox(title="수동 제어 성공", message=f"OK {action_name} 명령 전달 완료", icon="check", corner_radius=12)

        except Exception as e:
            self.save_error_log(f"[ERROR] {str(e)}")
            CTkMessagebox(title="명령 실패", message=f"ERROR {str(e)}", icon="cancel", corner_radius=12)

    def _sync_slider_to_entry(self, value, entry_widget):
        if entry_widget.winfo_exists():
            entry_widget.delete(0, "end"); entry_widget.insert(0, f"{float(value):.1f}")

    def _sync_entry_to_slider(self, slider_widget, entry_widget, min_val, max_val):
        try:
            val = float(entry_widget.get())
            val = max(min_val, min(val, max_val))
            if entry_widget.winfo_exists():
                entry_widget.delete(0, "end"); entry_widget.insert(0, f"{val:.1f}")
            if slider_widget.winfo_exists(): slider_widget.set(val)
        except ValueError:
            if slider_widget.winfo_exists() and entry_widget.winfo_exists():
                entry_widget.delete(0, "end"); entry_widget.insert(0, f"{slider_widget.get():.1f}")

    # =========================================================================
    # [SECTION 8] 비상 제어 / 복구 로직
    # =========================================================================
    # [백업 대비 수정] 관리자가 RESET API를 직접 호출하던 구조 제거
    # 직접 호출 후 사용자 reset_callback까지 실행하면 RESET이 중복 발행되고,
    # 사용자 창의 배출 전 HOME/배출 후 RESUME 분기가 무시되는 문제가 있음
    # 현재는 사용자 HMI의 공용 복구 진입점에 위임해 두 창이 같은 RecoveryStage 기준으로 동작
    def execute_system_recovery(self):
        """관리자 복구도 사용자 창의 RecoveryStage 기반 HOME/RESUME 분기로 요청합니다."""
        try:
            self.save_error_log("[SYSTEM] 복구 승인")
            # [백업 대비 추가] 사용자 창과 복구 인터페이스가 실제로 존재하는지 먼저 확인
            # 긴급정지 상태가 아닌데 초기화가 실행되어 로봇이 불필요하게 움직이는 것도 차단
            if not self.main_gui or not hasattr(self.main_gui, "recover_from_admin"):
                raise Exception("사용자 HMI 복구 인터페이스를 찾을 수 없습니다.")
            if not (self.main_gui.emergency_stop or self.main_gui.in_error_state):
                raise Exception("현재 사용자 HMI가 긴급정지 상태가 아닙니다.")

            # [백업 대비 추가] 현재 RecoveryStage가 WASTE_DUMPED 이후이면 남은 공정을 재개하고,
            # 이전이면 수거통/로봇을 원위치로 복귀시키는 분기를 선택
            can_resume = self.main_gui._can_resume_after_dump()
            recovery_text = "동작 재개 요청 중" if can_resume else "원위치 복귀 중"
            self.current_user_status_text = recovery_text

            # RESET API와 화면 전환은 사용자 HMI의 공용 분기에서 한 번만 수행
            self.main_gui.root.after(0, self.main_gui.recover_from_admin)
            self.window.after(100, lambda: self._show_delegated_recovery_status(recovery_text))

        except requests.ConnectionError:
            self.save_error_log("[ERROR] 백엔드 연결 실패")
            CTkMessagebox(
                title="연결 실패",
                message="ERROR 백엔드 서버와 연결되어 있지 않습니다.",
                icon="cancel"
            )
        except Exception as e:
            self.save_error_log(
                f"[ERROR] {str(e)}"
            )
            CTkMessagebox(
                title="복구 실패",
                message=f"ERROR {str(e)}",
                icon="cancel"
            )
            
    # [백업 대비 추가] 실제 RESET과 사용자 화면 전환은 main_gui가 담당하고,
    # 관리자 창은 선택된 HOME/RESUME 상태를 모니터링 화면에 표시하는 역할만 수행
    # 이렇게 분리해 관리자 창이 사용자 위젯을 직접 파괴하면서 발생하던 화면 경쟁도 방지
    def _show_delegated_recovery_status(self, recovery_text):
        """사용자 HMI에 위임한 복구 분기에 맞춰 관리자 모니터 화면만 갱신합니다."""
        if not self.window.winfo_exists():
            return
        self.show_tab("진행 상태 모니터링")
        if hasattr(self, "lbl_monitor_status") and self.lbl_monitor_status.winfo_exists():
            self.lbl_monitor_status.configure(
                text=f"RESET {recovery_text}",
                text_color="#4fa3e3",
            )

    def _safe_execute_recovery(self):
        """EMERGENCY [CRITICAL PATCH] Segmentation Fault (코어 덤프) 원천 차단 복구 엔진"""
        try:
            # 안전장치: 복구 진입 시점에 관리자 창이 이미 닫혔다면 수행 중단
            if not self.window.winfo_exists():
                return

            self.current_user_status_text = "대기 중"
            
            # 1단계: 유저 단말 메인 GUI 초기화 콜백 실행
            # (이 과정에서 메인 GUI 내부의 기존 스레드나 after 타이머들이 클리어되어야 합니다)
            if self.reset_callback: 
                self.reset_callback()
            
            # 2단계: 유저 창의 UI 드롭 및 재빌드가 '완전히' 끝날 때까지 
            # 관리자 창은 아무것도 건드리지 않고 비동기 대기 (300ms로 안전 마진 확대)
            def finalize_admin_recovery():
                # 락 해제 시점에 관리자 창 무결성 최종 검사
                if not self.window.winfo_exists(): 
                    return
                
                # 3단계: 유저 UI가 완전히 살아난 것을 확인한 후, 안전하게 탭 전환
                self.show_tab("진행 상태 모니터링")
                
                # 4단계: 탭이 바뀐 후 위젯이 완벽히 생성되었는지 '다시 확인'하고 텍스트 주입
                if hasattr(self, "lbl_monitor_status") and self.lbl_monitor_status.winfo_exists():
                    self.lbl_monitor_status.configure(
                        text="OPEN 시스템 복구 완료 (대기 중)", 
                        text_color="#00fa9a"
                    )
            
            # 유저 창의 생성 오버헤드를 고려하여 300ms 시차 부여 (C-코어 충돌 회피)
            self.window.after(300, finalize_admin_recovery)
                
        except Exception as e:
            self.save_error_log(f"[RECOVERY_ERROR] 복구 프로세스 중 예외 발생: {str(e)}")

    # ========================================================================
    # [SECTION 9] 로컬 파일 I/O 및 로그 관리
    # ========================================================================
    def load_cycle_count(self):
        """누적 가동 공정 횟수 카운터 디스크 I/O 트랙"""
        if os.path.exists(self.cycle_count_file):
            with open(self.cycle_count_file, "r") as f:
                try: self.total_cycles = int(f.read().strip())
                except ValueError: self.total_cycles = 142
        else: self.total_cycles = 142
            
        self.total_cycles += 1
        with open(self.cycle_count_file, "w") as f: f.write(str(self.total_cycles))

    def save_error_log(self, reason):
        """로컬 런타임 시스템 에러 로그 어펜딩 입출력"""
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        mode = "a" if os.path.exists(self.log_file_path) else "w"
        with open(self.log_file_path, mode, encoding="utf-8") as f:
            f.write(f"[{now}] {reason}\n")

    def refresh_log_display(self):
        """로그 텍스트 최신순 역순 로드 출력"""
        if not hasattr(self, "log_textbox") or not self.log_textbox.winfo_exists(): return
        self.log_textbox.configure(state="normal")
        self.log_textbox.delete("1.0", "end")
        if os.path.exists(self.log_file_path):
            with open(self.log_file_path, "r", encoding="utf-8") as f:
                self.log_textbox.insert("1.0", "".join(reversed(f.readlines())))
        self.log_textbox.configure(state="disabled")