---
description: deb 파일을 받아 fresh-install 빌드/마이그레이션을 검증하고, 실패 시 알려진 fix 패턴을 안내
argument-hint: "<deb-path-or-url> [--skip-build] [--skip-migrate]"
---

## 목적

이 세션에서 반복적으로 부딪힌 **"deb 빌드는 됐는데 fresh install 한 PC 에서만 깨지는"** 버그(예: 카카오 미러 strict-pin, `ModuleNotFoundError: pyarmor_runtime_000000`)를 사전에 잡기 위한 검증 파이프라인.

릴리즈 직전에 한 번, 또는 사용자가 새 deb 를 올려서 검증 요청할 때 호출.

## 실행

```bash
bash scripts/verify_deb.sh $ARGUMENTS
```

스크립트가 stdout 마지막 줄에 `REASON=<token>` 을 찍는다. 종료 코드와 토큰을 보고 다음 표대로 분류해서 보고할 것.

## 단계

1. **deb 확보** — URL 이면 curl 로 받고, 경로면 그대로 사용. dpkg-deb 로 `/tmp/etverify.XXXX/extract/` 에 풀음.
2. **정적 검증** — payload(`usr/share/easytrainer-project/`) 안에 세션 fix 들이 baked-in 되었는지 grep 으로 확인:
   - `scripts/rebuild_images.sh` 포함 (build.sh whitelist)
   - `docker-compose{,.cpu}.yml` 의 PYTHONPATH 에 `/root/backend` 포함
   - dev tree 의 `release/ui/installer.py` 의 docker run 마이그레이션 PYTHONPATH 에도 `/root/backend` 포함 (PyInstaller 바이너리는 deb 에 들어가지만 소스로는 검증 불가 — dev 와 deb 가 같은 commit 에서 빌드된다는 전제)
   - `ros2/Dockerfile.base` 에 `mirror.kakao.com` 없음
   - `backend/pyarmor_runtime_000000/` 존재
   - `backend/__init__.py` 가 PyArmor obfuscated
3. **이미지 빌드** — `bash scripts/rebuild_images.sh all` 을 payload 위치에서 실행. `easytrainer-{ros2,backend}-base` → `easytrainer-{ros2,backend}` 4개 이미지가 만들어진다.
4. **마이그레이션 smoke test** — installer.py 가 부르는 것과 동일한 `docker run --rm easytrainer-backend:latest ...` 호출을 임시 `/opt/easytrainer` 마운트로 재현. `from backend.database.models import create_tables; create_tables()` 이 성공하면 OK.

## 실패 분류 (REASON 토큰 → 권장 조치)

| 종료코드 | REASON 토큰 | 권장 조치 |
|---|---|---|
| 10 | `deb_download_failed` | URL/네트워크 확인 |
| 10 | `deb_not_found` | 경로 오타 또는 파일 미존재 |
| 11 | `payload_dir_missing` | `release/build.sh` 의 PAYLOAD_DIR 변경 여부 확인 |
| 11 | `missing_rebuild_images_sh` | `release/build.sh` whitelist 깨짐 (commit bd06993 참고) |
| 11 | `compose_pythonpath_unpatched` | `docker-compose*.yml` 의 PYTHONPATH 회귀 (commit d35cc46 참고) |
| 11 | `installer_pythonpath_unpatched` | `release/ui/installer.py` 의 docker run PYTHONPATH 회귀 — installer 와 compose 양쪽 다 패치해야 함 |
| 11 | `kakao_mirror_present` | `ros2/Dockerfile.base` 에 카카오 미러 재유입 (commit a3661b8 참고) |
| 11 | `pyarmor_runtime_missing` | `deploy.yml` 의 PyArmor 단계 산출물 누락 — CI 로그 확인 |
| 11 | `backend_init_not_obfuscated` | PyArmor trial 32KB 한도 초과 또는 obfuscation 스킵 — `deploy.yml` 의 exclude 리스트 확인 (commit cbca95c 참고) |
| 20 | `apt_held_broken` | `Dockerfile.base` 의 미러/`apt-get upgrade` 흐름 검토 |
| 20 | `docker_daemon_unavailable` | 호스트 도커 데몬/권한 |
| 20 | `build_path_missing` | RSYNC_EXCLUDES 가 너무 공격적 — whitelist 추가 |
| 20 | `image_build_failed` | 로그 tail 보고 케이스별로 판단 |
| 30 | `pyarmor_runtime_not_found` | PYTHONPATH 에 `/root/backend` 누락 — compose **그리고** installer 둘 다 확인 |
| 30 | `pyarmor_runtime_abi_mismatch` | CI(`deploy.yml`) 의 `setup-python` 버전이 `backend/Dockerfile.base` 의 베이스 OS Python 과 불일치 — Ubuntu 24.04 면 3.12, 22.04 면 3.10 으로 맞출 것 |
| 30 | `lerobot_not_found` | PYTHONPATH 에 `/root/backend/lerobot/src` 누락 |
| 30 | `import_failure` | 누락된 모듈을 `backend/` 또는 `requirements.txt` 에서 찾기 |
| 30 | `sqlite_error` | 모델 정의 또는 기존 DB 권한 |
| 0  | `ok` / `ok_partial` | 전 단계 통과 (`ok_partial` 은 `--skip-*` 가 있었을 때) |

## 사용 가이드

- **사전 조건:** `docker` CLI 접근 가능, 호스트가 deb 가 만들어진 GPU/CPU 환경과 일치 (예: amd64).
- **권한:** sudo 불필요. /opt 도 건드리지 않음 (임시 디렉토리 사용).
- **기존 이미지 캐시 영향:** rebuild_images.sh 는 `docker image inspect` 로 base 이미지 존재 여부만 체크하기 때문에 이미 빌드된 이미지가 있으면 기존 것을 재사용한다. **순수 fresh 시나리오를 재현하려면** 호출 전에 `docker image rm easytrainer-{ros2,backend}{,-base}:latest` 로 정리할 것.
- **유의:** static check 는 payload 가 dev tree 와 같은 commit 으로 빌드됐다는 전제로 동작한다. CI 로 deb 가 만들어진 경우 일치 보장이 어려우므로, `git diff $(git log --oneline --grep "<deb-version>" -1 | awk '{print $1}')` 로 차이가 없는지 확인하면 더 안전하다.

## 자율 운영에서의 사용

릴리즈 직전 자동 검증 게이트로 활용 가능:

```
git tag X.Y.Z → CI 가 deb 빌드 → asset URL 받아오기
  ↓
/verify-deb <asset-url>
  ↓ REASON=ok
git push --tags  (또는 GitHub Release publish)
```
