import sqlite3
import os

# 데이터 저장 경로 지정 (프로젝트 루트의 data 폴더 내)
DB_PATH = os.path.join(os.path.dirname(__file__), "../../data/robot_system.db")

def get_db_connection():
    """DB 연결 객체를 반환하는 함수"""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row  # 딕셔너리 형태로 데이터를 읽어오기 설정
    return conn

def init_db():
    """앱 구동 시 테이블을 생성하고 초기 세팅값을 넣는 함수"""
    # data 폴더가 없으면 생성
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # 1. 배출 모드 마스터 테이블 생성
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tb_dump_modes (
            mode_id INTEGER PRIMARY KEY AUTOINCREMENT,
            mode_name TEXT NOT NULL,
            tilt_angle INTEGER NOT NULL,
            shake_count INTEGER NOT NULL
        );
    """)
    
    # 2. 배출 작업 이력 테이블 생성
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tb_dump_history (
            task_id TEXT PRIMARY KEY,
            mode_id INTEGER,
            start_time TEXT DEFAULT (datetime('now', 'localtime')),
            end_time TEXT,
            status TEXT NOT NULL,
            FOREIGN KEY (mode_id) REFERENCES tb_dump_modes(mode_id)
        );
    """)
    
    # 3. 에러 로그 테이블 생성
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS tb_error_log (
            error_id INTEGER PRIMARY KEY AUTOINCREMENT,
            task_id TEXT,
            error_code TEXT NOT NULL,
            error_msg TEXT NOT NULL,
            error_time TEXT DEFAULT (datetime('now', 'localtime')),
            FOREIGN KEY (task_id) REFERENCES tb_dump_history(task_id)
        );
    """)
    
    # 초기 마스터 데이터 주입 (데이터가 없을 때만 초기화)
    cursor.execute("SELECT COUNT(*) FROM tb_dump_modes")
    if cursor.fetchone()[0] == 0:
        cursor.execute("INSERT INTO tb_dump_modes (mode_name, tilt_angle, shake_count) VALUES ('유형1: 일반 배출 + 세척', 45, 5)")
        cursor.execute("INSERT INTO tb_dump_modes (mode_name, tilt_angle, shake_count) VALUES ('유형2: 강하게 털기 + 세척', 60, 10)")
        print("💡 [DB] 초기 배출 모드 데이터 등록 완료!")
        
    conn.commit()
    conn.close()
    print("✅ [DB] SQLite 데이터베이스 초기화 및 테이블 생성 성공")