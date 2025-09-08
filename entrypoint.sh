#!/bin/bash

# tmux가 터미널 유형을 올바르게 인식하도록 TERM 환경 변수 설정
export TERM=xterm

# 'ros_dev'라는 이름으로 새로운 tmux 세션을 백그라운드에서 시작하고 첫 창의 이름을 'ui'로 지정
tmux new-session -d -s ros_dev -n 'ui'

# --- 첫 번째 터미널 (창 0번 'ui'): UI 개발 서버 실행 ---
# tmux 세션의 첫 번째 창에 UI 관련 명령어 전송
# C-m은 Enter 키 입력을 의미합니다.
tmux send-keys -t ros_dev:0 "echo '--- Starting UI dev server ---' && cd /root/src/ui && npm install && npm run dev" C-m

# --- 두 번째 터미널 (창 1번 'ros_ws'): ROS 2 워크스페이스 빌드 ---
# 'ros_ws'라는 이름으로 새 창 생성
tmux new-window -t ros_dev:1 -n 'ros_ws'
# 새로 만든 창에 ROS 2 관련 명령어 전송 후, bash 셸을 실행하여 터미널 유지
tmux send-keys -t ros_dev:1 "echo '--- Building and sourcing ROS2 workspace ---' && cd /root/ros2_ws && colcon build && source install/setup.bash && /bin/bash" C-m

# --- 세 번째 터미널 (창 2번 'backend'): 백엔드 API 실행 ---
# 'backend'라는 이름으로 새 창 생성
tmux new-window -t ros_dev:2 -n 'backend'
# 새로 만든 창에 백엔드 API 실행 명령어 전송
tmux send-keys -t ros_dev:2 "echo '--- Starting backend API ---' && cd /root/src/backend/database \
 && bash migration.sh && cd ../../ && python3 -m backend.api.app" C-m

# tmux 세션의 첫 번째 창을 기본으로 선택
tmux select-window -t ros_dev:0

# 생성된 tmux 세션에 접속하여 사용자가 바로 터미널을 볼 수 있도록 함
# 이 명령어가 포그라운드에서 실행되므로 컨테이너가 종료되지 않습니다.
tmux attach-session -t ros_dev