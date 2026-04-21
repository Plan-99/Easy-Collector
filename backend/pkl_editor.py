import pickle

# 파일 경로
file_path = 'backend/checkpoints/104/safetensors.pkl'

# 1. 파일 읽기 모드('rb')로 열기
with open(file_path, 'rb') as f:
    data = pickle.load(f)

# 데이터 구조 확인 (출력해서 어떻게 수정할지 파악)
print("수정 전 데이터:", data)

# import pickle
# import numpy as np  # 데이터 내에 numpy array가 있으므로 임포트 필요

# def modify_sensor_keys(input_path, output_path):
#     # 1. pickle 파일 불러오기
#     try:
#         with open(input_path, 'rb') as f:
#             data = pickle.load(f)
#         print(f"✅ 파일을 성공적으로 불러왔습니다: {input_path}")
        
#         # 변경 전 키 확인 (디버깅용)
#         # print("변경 전 키:", data.keys())

#     except FileNotFoundError:
#         print(f"❌ 파일을 찾을 수 없습니다: {input_path}")
#         return

#     # 2. 키 이름 변경 로직
#     # pop('기존키')는 기존 키의 값을 반환하고 딕셔너리에서 해당 키를 삭제합니다.
    
#     if 'observation.images.sensor_1' in data:
#         data['observation.images.sensor_13'] = data.pop('observation.images.sensor_1')
#         print("🔹 Renamed: observation.images.sensor_1 -> observation.images.sensor_13")
#     else:
#         print("⚠️ Warning: 'observation.images.sensor_2' 키가 존재하지 않습니다.")
#     # sensor_2 -> sensor_14
#     if 'observation.images.sensor_2' in data:
#         data['observation.images.sensor_14'] = data.pop('observation.images.sensor_2')
#         print("🔹 Renamed: observation.images.sensor_2 -> observation.images.sensor_14")
#     else:
#         print("⚠️ Warning: 'observation.images.sensor_2' 키가 존재하지 않습니다.")

#     # sensor_3 -> sensor_15
#     if 'observation.images.sensor_3' in data:
#         data['observation.images.sensor_15'] = data.pop('observation.images.sensor_3')
#         print("🔹 Renamed: observation.images.sensor_3 -> observation.images.sensor_15")
#     else:
#         print("⚠️ Warning: 'observation.images.sensor_3' 키가 존재하지 않습니다.")

#     # 3. 변경된 데이터를 다시 pickle 파일로 저장
#     with open(output_path, 'wb') as f:
#         pickle.dump(data, f)
    
#     print(f"✅ 변경된 데이터가 저장되었습니다: {output_path}")
    
#     # 변경 후 키 확인 (디버깅용)
#     # print("변경 후 키:", list(data.keys()))

# # --- 사용 예시 ---
# # 실제 pkl 파일 경로를 입력하세요.
# input_pkl = 'backend/checkpoints/104/dataset_stats.pkl'       # 수정할 원본 파일 이름
# output_pkl = 'backend/checkpoints/104/dataset_stats.pkl' # 저장할 파일 이름

# # 실행 (파일이 실제로 존재해야 오류가 나지 않습니다)
# modify_sensor_keys(input_pkl, output_pkl)