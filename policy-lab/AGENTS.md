# AGENTS.md — policy-lab

> 목차(table of contents). 백과사전 아님. 자세한 건 가리키는 파일에서.
> 이 랩은 EasyTrainer 위에서 도는 **독립 정책 진화 하네스**다. 전체 개요는 [README.md](README.md).

## 이 랩은
EasyTrainer 의 tutorial peg-in-hole 태스크에서 모방학습 정책을 설계→토너먼트→분석→진화하며
챔피언 success_rate 를 끌어올리는 자율 연구 랩. 하네스는 독립, 산출물은 EasyTrainer 트리에 씀.

## 시작점
- 어떻게 운영하나 (cd + claude, EASYTRAINER_ROOT) → [README.md](README.md)
- 반드시 지킬 규칙 → [golden-principles.md](golden-principles.md)
- 채점 기준 → [rubric.md](rubric.md)
- EasyTrainer 내부 구조 (read-only) → [docs/easytrainer-integration.md](docs/easytrainer-integration.md)

## 작업 방식
- 패턴: generate-validate (설계→학습/평가 게이트) + 진화 루프(supervisor 성격). 야간은 `/lab-overnight`.
- 명령: `/lab-smoke` `/policy-design` `/policy-tournament` `/policy-evolve` `/lab-overnight`
- 에이전트: `policy-researcher` (정책 저작 전담)

## EASYTRAINER_ROOT
호스트 경로는 `EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"` 로 도출
(랩이 EasyTrainer 레포 안 → toplevel = EasyTrainer 루트). 컨테이너 작업은 `docker exec easytrainer_backend`.

## 학습
막힌 점/실패/진화 교훈은 [learning.md](learning.md) 에 누적.
