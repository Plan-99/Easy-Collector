#!/bin/bash
# (Bash 4.0 이상 필요)

# --- 스크립트 설정 ---
set -e

pip install --upgrade pip setuptools

# --- 1. 스크립트 기준 경로 ---
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# --- 2. (기본) 설치할 패키지 상대 경로 목록 ---
# 여기에 모든 패키지의 상대 경로를 나열합니다.
RELATIVE_PATHS=(
    "xr_teleoperate/teleop/robot_control/dex-retargeting"
    "xr_teleoperate/teleop/televuer" # 특별한 명령이 필요한 경로
    "unitree_sdk2_python"
)

# --- 3. (신규) 경로별 '설치 후' 실행할 명령어 정의 ---
# Bash 4.0+의 연관 배열(딕셔너리)을 선언합니다.
declare -A POST_INSTALL_COMMANDS

# [Key]는 RELATIVE_PATHS에 있는 '상대 경로' 문자열과 정확히 일치해야 합니다.
# [Value]는 해당 디렉터리에서 실행할 셸 명령어 문자열입니다.

# 예시 1: televuer 경로에서 openssl 명령어 실행
POST_INSTALL_COMMANDS["xr_teleoperate/teleop/televuer"]="openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem"

# 예시 2: 다른 경로에서 다른 명령어 실행 (예: build)
# POST_INSTALL_COMMANDS["pkg_one"]="npm run build"

# --- 4. 스크립트 메인 로직 ---
echo "Script Base Directory: $SCRIPT_DIR"
echo "Found ${#RELATIVE_PATHS[@]} packages to process."
echo "-----------------------------------"

for rel_path in "${RELATIVE_PATHS[@]}"
do
    pkg_path="$SCRIPT_DIR/$rel_path"

    echo "Processing target: $pkg_path"
    
    (
        if [ -d "$pkg_path" ]; then
            # 1. 해당 디렉터리로 이동
            cd "$pkg_path"
            
            # 2. (공통) pip install 실행
            echo "Running 'pip install -e .' in $(pwd)"
            pip install -e .
            echo "✅ Successfully installed $pkg_path"

            # 3. (신규) 이 경로에 대해 정의된 '추가 명령'이 있는지 확인
            if [[ -v POST_INSTALL_COMMANDS["$rel_path"] ]]; then
                echo "⚡ Running post-install command for: $rel_path"
                
                # 정의된 명령어 문자열을 가져옵니다.
                command_to_run=${POST_INSTALL_COMMANDS["$rel_path"]}
                
                echo "Executing: $command_to_run"
                
                # eval을 사용하여 문자열을 셸 명령어로 실행합니다.
                eval "$command_to_run"
                
                echo "✅ Post-install command finished."
            fi
            
        else
            echo "⚠️ Error: Directory not found, skipping: $pkg_path"
        fi
    )
    
    echo "-----------------------------------"
done

echo "🎉 All processing complete."