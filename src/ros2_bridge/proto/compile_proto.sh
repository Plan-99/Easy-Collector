#!/usr/bin/env bash
# Proto 컴파일 스크립트
# 사용: bash compile_proto.sh
# grpcio-tools 필요: pip install grpcio-tools

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="$SCRIPT_DIR/../generated"

mkdir -p "$OUT_DIR"

python3 -m grpc_tools.protoc \
    -I "$SCRIPT_DIR" \
    --python_out="$OUT_DIR" \
    --pyi_out="$OUT_DIR" \
    --grpc_python_out="$OUT_DIR" \
    "$SCRIPT_DIR/robot_bridge.proto"

# Fix imports in generated gRPC file (protoc generates absolute imports)
sed -i 's/^import robot_bridge_pb2/from . import robot_bridge_pb2/' "$OUT_DIR/robot_bridge_pb2_grpc.py"

touch "$OUT_DIR/__init__.py"

echo "Proto compilation complete. Output in $OUT_DIR/"
