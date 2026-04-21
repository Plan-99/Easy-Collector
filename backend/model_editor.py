import torch
from safetensors.torch import load_file, save_file
import os

# ================= 수정할 경로 설정 =================
# 에러가 나는 그 체크포인트 폴더 경로를 정확히 넣어주세요.
checkpoint_dir = "backend/checkpoints/104"  # 예: outputs/train/.../checkpoints/last/
# ===================================================

model_path = os.path.join(checkpoint_dir, "model.safetensors")

if not os.path.exists(model_path):
    print(f"❌ 파일을 찾을 수 없습니다: {model_path}")
    print("혹시 파일명이 pytorch_model.bin 이라면 코드를 수정해야 합니다.")
    exit()

print(f"🔄 모델 로딩 중: {model_path}")
weights = load_file(model_path)
new_weights = {}

# 변경할 이름 매핑 (구버전 -> 신버전)
# sensor_1 -> sensor_13
# sensor_2 -> sensor_14
# sensor_3 -> sensor_15
key_mapping = {
    "sensor_1": "sensor_13",
    "sensor_2": "sensor_14",
    "sensor_3": "sensor_15"
}

modified_count = 0

for key, value in weights.items():
    new_key = key
    for old_name, new_name in key_mapping.items():
        if old_name in key:
            # 단순히 replace하면 sensor_1이 sensor_13, 14, 15에 모두 매칭될 위험이 있으므로
            # 정확히 이름이 포함된 경우만 교체합니다.
            # 예: "normalize_inputs.mean.observation.images.sensor_1" 
            #  -> "normalize_inputs.mean.observation.images.sensor_13"
            
            # 헷갈림 방지를 위해 구체적인 패턴 확인
            target_str = f"observation.images.{old_name}"
            replace_str = f"observation.images.{new_name}"
            
            if target_str in key:
                new_key = key.replace(target_str, replace_str)
                modified_count += 1
                break
    
    new_weights[new_key] = value

print(f"✅ 총 {modified_count}개의 키 이름을 변경했습니다.")

# 기존 파일 백업
backup_path = model_path + ".bak"
if not os.path.exists(backup_path):
    os.rename(model_path, backup_path)
    print(f"📦 원본 백업 완료: {backup_path}")

# 새 파일 저장
save_file(new_weights, model_path)
print(f"🎉 수정된 모델 저장 완료: {model_path}")
print("이제 다시 실행해 보세요!")