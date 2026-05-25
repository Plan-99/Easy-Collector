# backend/Dockerfile  ── "modules layer"
# ------------------------------------------------------------------
# easytrainer-backend-base 위에 extension/backend_* deps 를 baked.
# 빌드 컨텍스트의 `backend/_modules_staging/` 디렉터리에
# scripts/rebuild_images.sh 가 manifest 들을 stage 해두면, COPY
# layer hashing 이 변경 시 자동으로 캐시 무효화한다.
ARG BASE_IMAGE=easytrainer-backend-base:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

COPY backend/_modules_staging/ /tmp/modules_staging/
COPY backend/install_modules.sh /usr/local/bin/install_modules.sh
RUN chmod +x /usr/local/bin/install_modules.sh

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    /usr/local/bin/install_modules.sh /tmp/modules_staging && \
    rm -rf /tmp/modules_staging

RUN touch /etc/.modules_baked
