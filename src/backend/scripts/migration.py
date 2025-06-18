import sqlite3
import os

# 컨테이너 내부의 데이터 저장 경로
DB_DIR = '../database'
DB_PATH = os.path.join(DB_DIR, 'main.db')

# /data 디렉토리가 없으면 생성
os.makedirs(DB_DIR, exist_ok=True)

# 데이터베이스 연결
conn = sqlite3.connect(DB_PATH)
cursor = conn.cursor()

print(f"성공적으로 {DB_PATH}에 연결되었습니다.")

# 테이블 생성 (없을 경우에만)
cursor.execute('''
CREATE TABLE IF NOT EXISTS users (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    age INTEGER
)
''')

# 데이터 추가
try:
    cursor.execute("INSERT INTO users (name, age) VALUES (?, ?)", ('Alice', 30))
    cursor.execute("INSERT INTO users (name, age) VALUES (?, ?)", ('Bob', 25))
    conn.commit()
    print("데이터를 성공적으로 추가했습니다.")
except sqlite3.IntegrityError:
    print("데이터가 이미 존재하거나 다른 오류가 발생했습니다.")


# 데이터 조회 및 출력
print("\n--- 사용자 목록 ---")
cursor.execute("SELECT * FROM users")
for row in cursor.fetchall():
    print(f"ID: {row[0]}, 이름: {row[1]}, 나이: {row[2]}")

# 연결 종료
conn.close()