---
description: 30s 빠른 검증 게이트 — backend health + 핵심 GET 엔드포인트 + 컨테이너 상태
argument-hint: "[--quick] [--json]"
---

자율 운영 루프의 첫 게이트. **변경을 만들기 전에 baseline으로 한 번**,
**변경 후 검증으로 한 번** 호출하는 게 기본 패턴.

## 실행

[scripts/smoke.sh](../../scripts/smoke.sh)를 인자 `$ARGUMENTS` 그대로 전달해서 실행.

```bash
bash scripts/smoke.sh $ARGUMENTS
```

## 인자

| 인자 | 의미 | 소요 |
|------|------|------|
| (없음) | 컨테이너 + healthz + 11개 핵심 GET + training_server + ros2 daemon | ~10–20s |
| `--quick` | 컨테이너 + healthz만 | ~3s |
| `--json` | machine-readable (overnight 루프에서 사용) | 동일 |

## 종료 코드 해석

- `0` — 통과. 안전하게 다음 단계.
- `1` — 일부 검사 실패. **변경 진행 금지**. 어떤 검사가 실패했는지 출력 확인.
- `2` — 핵심 인프라(backend 컨테이너) 자체가 안 떠있음. `docker compose up -d backend ros2`로 먼저 띄워야 함.

## 자율 루프에서의 위치

```
spec 받음
  ↓
/smoke (baseline) ── exit≠0 → 인프라 문제로 보고하고 중단
  ↓
구현
  ↓
quick_apply.sh
  ↓
/smoke (변경 후) ── exit≠0 → 다시 구현 시도 (최대 2회), 그래도 실패면 revert
  ↓
(선택) /e2e — 무거운 검증
  ↓
commit + PR
```

## 주의

- backend가 hot reload 중이면 `/api/healthz`가 잠시 502/000 나올 수 있음. `--quick` 한 번 더 돌려 안정화 확인.
- 새 라우트 블루프린트를 추가했으면 `scripts/smoke.sh`의 GET 엔드포인트 목록에 한 줄 추가할 것 (아니면 새 라우트가 깨져도 smoke가 못 잡음).
- 이 명령은 어떤 파일도 수정하지 않음. 안전하게 반복 호출 가능.
