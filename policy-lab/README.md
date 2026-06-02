# policy-lab — EasyTrainer 위에서 도는 정책 진화 랩

EasyTrainer 와 **독립된 하네스**로, 로봇 모방학습 정책을 **설계 → 토너먼트(학습+평가) → 분석 →
진화 → 반복**하며 챔피언을 끌어올리는 자율 연구 랩. foundry-harness 의 `/forge` 로 주조됨.

## 독립성과 결합의 경계 (중요)

- **하네스는 독립이다.** 명령(`/policy-*`, `/lab-overnight`, `/lab-smoke`)·에이전트(`policy-researcher`)·
  원칙·채점·진화 로그·야간 루프는 전부 이 폴더 안. EasyTrainer 의 `.claude/`(기능 개발 하네스)와 섞이지 않는다.
- **그러나 EasyTrainer 구조를 읽고, 지정 위치에 쓴다.** 정책 산출물(새 policy 코드, train_worker dispatch,
  evaluator dispatch, DB/UI config)은 학습·평가되려면 **EasyTrainer 트리 안**에 있어야 한다. 그래서
  랩은 EasyTrainer 를 read(구조 맵) + 정해진 곳에 write 한다. EasyTrainer 내부 지도:
  [docs/easytrainer-integration.md](docs/easytrainer-integration.md).

## 작동 방식 (어떻게 "독립 하네스"가 로드되나)

이 폴더를 **cwd 로** Claude Code 를 띄운다:
```bash
cd /home/hjhj/EasyTrainer_v2.3.1/policy-lab
claude            # 이 폴더의 .claude/{commands,agents,settings} 가 로드된다
```
랩은 EasyTrainer 레포 *안*에 있으므로 EasyTrainer 루트는 곧 git toplevel:
```bash
EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"   # = EasyTrainer 루트
```
모든 호스트 측 스크립트/코드 경로는 `$EASYTRAINER_ROOT/...` 로 참조한다 (호스트 무관). 컨테이너
내부 작업은 `docker exec easytrainer_backend ...` 라 경로 영향 없음.

## 명령 (`.claude/commands/`)

| 명령 | 역할 |
|------|------|
| `/lab-smoke` | 랩 readiness 게이트 — EasyTrainer smoke + tutorial sim + eval bridge alive |
| `/policy-design <Type>` | 새 정책 1개 설계·구현·EasyTrainer 등록 (policy-researcher 위임) |
| `/policy-tournament` | 후보 N개를 같은 조건으로 학습·평가, 챔피언 갱신 |
| `/policy-evolve` | 토너먼트→분석→다음 후보 설계→재토너먼트 (루프 1회분) |
| `/lab-overnight` | **야간 무인 진입점** — evolve 루프를 시간/라운드 예산 안에서 자율 반복 (이 랩의 "nightwatch") |

## 에이전트 (`.claude/agents/`)
- `policy-researcher` — 새 정책 변형 연구·구현. `backend/lerobot/` read-only.

## 시스템 오브 레코드
- [AGENTS.md](AGENTS.md) — 랩 진입 목차
- [golden-principles.md](golden-principles.md) — 기계검증 가능한 랩 규칙
- [rubric.md](rubric.md) — 토너먼트/진화 채점 (success_rate strictly-higher)
- [learning.md](learning.md) — 랩 실패·진화 로그
- [docs/evolve/](docs/evolve/) — 라운드별 분석 보고서 (커밋본)
- [docs/easytrainer-integration.md](docs/easytrainer-integration.md) — EasyTrainer 내부 구조 지도 (read-only)

## EasyTrainer 와의 관계 요약
- EasyTrainer 하네스(`../.claude/`, `../docs/HARNESS.md`)는 **기능 개발 + UI nightwatch + overnight-queue(기능 spec)** 담당.
- 이 랩은 **정책 연구/토너먼트/진화 + 야간 자율 진화** 담당. 둘은 서로의 파일을 건드리지 않는다.
- 공유 인프라(컨테이너, `scripts/smoke.sh`, `scripts/db_snapshot.sh`, `scripts/overnight_summary.sh`,
  `backend/tools/model_tester/`)는 EasyTrainer 트리에 있고 랩이 `$EASYTRAINER_ROOT` 로 재사용한다.
