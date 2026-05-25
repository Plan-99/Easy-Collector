# Launcher 인증 & 모듈 Entitlement

- **결정일**: 2026-04-28
- **상태**: 적용 중
- **영향 폴더**: [release/ui/](../../release/ui/), [home-next/](../../home-next/)

## 결정

데스크탑 런처(`release/ui/`)의 인증을 **시리얼 키 → Google OAuth Device Flow + Bearer
토큰** 모델로 전환했다. 모듈 설치는 사용자별 entitlement 검사를 거친다.

## 배경 (Why)

시리얼 키 모델은 "1 키 = 1 머신" 가정이라 결제·보유 모델이 사용자 단위인 신규 비즈니스
모델과 어울리지 않았다. Google OAuth + Device Flow는:

- 사용자 계정 단위로 자연스럽게 권한 묶기
- 머신 fingerprint와 묶어 token rotation 가능
- 브라우저 결제 흐름(`/checkout/{module_id}`)과 자연스럽게 연결

## 구조

| 항목 | 값 |
|------|---|
| 삭제된 파일 | `release/ui/license_validator.py` |
| 새 파일 | `release/ui/device_auth.py` |
| 진입점 | `service.py:ensure_signed_in()` → `device_auth.ensure_signed_in_gui()` (Qt 모달) |
| 토큰 저장 위치 | `${EASYTRAINER_DATA_DIR}/auth.json` (기본 `/opt/easytrainer/auth.json`, perms 0600) |
| 서버 URL 설정 키 | `config.json["license_server_url"]` (legacy 키명 호환) |
| 기본 서버 URL | `https://easytrainerhome.vercel.app` |
| 머신 ID 함수 | `device_auth.get_machine_fingerprint()` (sha256 of `/etc/machine-id` 등) |

## 모듈 Entitlement 흐름

1. 설치 전 `modules.is_module_entitled(module_id)` 호출
2. `core` / `feature` 카테고리(`required=True`)는 항상 통과
3. 무료 모듈(`priceKrw=0`)도 서버측 `/api/entitlements`에서 묵시적 보유로 처리됨 — 클라이언트는 신경 쓸 필요 없음
4. 유료 모듈 클릭 → `launcher.py:_open_checkout()` → 브라우저로 `${api}/checkout/{module_id}` 오픈
5. 3초 간격으로 `/api/entitlements` 폴링 (최대 10분)
6. entitlement 감지되면 `_make_action(install)` 자동 트리거 → 설치까지 진행
7. 사용자 취소 시 폴링 중단

## 운영 시 주의

- 토큰 저장 위치가 `${EASYTRAINER_DATA_DIR}/auth.json`임. `~/.config/easytrainer/auth.json`이
  **아니다**. 자주 오해되는 부분.
- 새 API 호출 추가 시 `modules.py:_get_bearer()` + `_http_get_json()` 패턴 재사용
- 머신 fingerprint 알고리즘은 시리얼 검증 시절과 동일하게 유지 (기존 사용자 식별 호환)

## 관련

- [Remote Training Server 분리](2026-04-17_remote-training-server.md)
