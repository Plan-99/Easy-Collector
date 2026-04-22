#!/usr/bin/env bash
# 양쪽 그리퍼 열고/닫기
# usage: ./toggle_grippers.sh open | close

set -e

CMD="${1:-open}"

case "$CMD" in
    open)  VAL="1.0" ;;
    close) VAL="0.0" ;;
    *)
        echo "usage: $0 open|close"
        exit 1
        ;;
esac

ros2 topic pub --once /grp/left/${CMD}  std_msgs/msg/Float32 "data: 1.0" &
ros2 topic pub --once /grp/right/${CMD} std_msgs/msg/Float32 "data: 1.0" &
wait
echo "[OK] both grippers -> ${CMD} (${VAL})"
