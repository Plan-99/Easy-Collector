# ros2/Dockerfile  ── "modules layer"
# ------------------------------------------------------------------
# easytrainer-ros2-base 이미지 위에 설치된 모듈들의 apt/pip deps 를
# baked 한다. 빌드 컨텍스트의 `ros2/_modules_staging/` 디렉터리에
# scripts/rebuild_ros2_image.sh 가 manifest 들을 stage 해두면,
# Docker 의 COPY layer hashing 이 manifest 변경 시 자동으로 캐시
# 무효화한다. (변경 없으면 layer 캐시 hit → instant rebuild)
#
# 모든 apt/pip 다운로드는 BuildKit cache mount 로 영구 캐시되므로
# 외부 미러(특히 robotpkg) 가 죽어있어도 한 번 받은 패키지는 재사용된다.
# 빌드 시 반드시 DOCKER_BUILDKIT=1 (또는 buildx) 사용.
ARG BASE_IMAGE=easytrainer-ros2-base:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# Staged manifests (생성: scripts/rebuild_ros2_image.sh).
# 디렉터리가 비어있어도 빌드는 통과한다 (no-op layer).
COPY ros2/_modules_staging/ /tmp/modules_staging/

# install_modules.sh: manifest 들을 읽고 deps 를 한 번에 설치
COPY ros2/install_modules.sh /usr/local/bin/install_modules.sh
RUN chmod +x /usr/local/bin/install_modules.sh

# BuildKit cache mount 로 apt/pip 다운로드 영구 캐시
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    /usr/local/bin/install_modules.sh /tmp/modules_staging && \
    rm -rf /tmp/modules_staging

# 이 marker 가 있으면 start_ros2_services.sh 가 runtime install 을 skip 한다.
RUN touch /etc/.modules_baked
