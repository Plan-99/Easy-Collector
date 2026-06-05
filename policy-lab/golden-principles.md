# Golden Principles — policy-lab

> 기계적으로 검증 가능한 규칙만. 각 규칙에 *왜* + *위반 감지*. 규칙은 세금 — 관찰된 실패/정책이
> 있는 것만. 랩 진입: [AGENTS.md](AGENTS.md), 진화 로그: [learning.md](learning.md).

## 규칙

1. **Architecture > Augmentation.** 후보는 vision encoder / temporal head / loss / fusion 중 ≥1곳이
   baseline 대비 구조적으로 달라야 한다. 단순 augmentation·hyperparameter sweep 후보는 기각.
   — 왜: 튜닝은 진화가 아니다(평균 회귀). / 위반 감지: 후보 설계서에 "architecture diff vs baseline" 명시 + arXiv 근거.
   근거: memory `feedback_policy_evolve_creativity`.

2. **`backend/lerobot/` 는 read-only.** 정책은 import + composition 으로만. introspection 은 forward_hook 외부 부착.
   — 왜: 벤더 트리 수정은 다른 정책 기준선을 흔든다. / 위반 감지: `git diff --name-only -- "$EASYTRAINER_ROOT/backend/lerobot/"` 가 비어야 함. 근거: memory `lerobot_readonly`.

3. **성공은 ground-truth scene state 로 판정.** 정책 done 토큰 금지. 같은 체커가 학습 데이터 필터 + 추론 success_rate 양쪽.
   — 왜: self-report 는 0%를 100%로 보고. / 위반 감지: `tutorial_evaluator` / `check_success` 경로 사용. 근거: memory `feedback_success_check`.

4. **vision-only + EE-relative + no-proprio 가정 유지.** `obs_state_keys=[]`, action 은 6-DoF EE delta.
   — 왜: 이 셋업의 일반화 강함이 검증됨(arXiv 2509.18644). / 위반 감지: config 의 obs_state_keys 비어있음 + action_type. 근거: memory `feedback_obs_state_keys_empty`, `feedback_collector_eepos`.

5. **6곳 등록 완결성.** 새 type 은 train_worker dispatch + evaluator dispatch + DB POLICY_CONFIGS + UI modelConfigs(+ utils, policy 코드) 전부.
   — 왜: 하나라도 빠지면 토너먼트가 정책을 못 연다. / 위반 감지: `/policy-design` step 2 의 4-그렙 검증 + 1-epoch import smoke.

6. **챔피언은 strictly-higher 일 때만 갱신.** 동률/하락은 무시.
   — 왜: 노이즈로 챔피언이 흔들리면 진화가 후퇴. / 위반 감지: `tournament declare-champion` 의 strictly-higher 규칙.

7. **호스트 경로는 EASYTRAINER_ROOT 로.** 하드코딩 금지. `EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"`.
   — 왜: 호스트/체크아웃 무관 이식성 (EasyTrainer 하네스 F1 교훈). / 위반 감지: `grep -rn "/home/[a-z]*/" .claude/` 가 비어야 함.

8. **VRAM ≤ 8GB / batch_size 16 기본.** "더 큰 모델로 도망" 금지 — 그 안에서 영리하게.
   — 왜: RTX 3070 Ti 제약. / 위반 감지: 설계서의 메모리 어림셈 + 1-epoch OOM 없음.

9. **라운드 분석 보고서는 `policy-lab/docs/evolve/<run>_round_<N>/analysis.md` 에 커밋.** 한국어, 표+승자분석+실패모드+다음가설.
   — 왜: 진화의 추론 흔적이 다음 라운드 입력. / 위반 감지: 라운드 종료 시 파일 존재. 근거: memory `feedback_evolve_round_reports`.

## 검증
- 등록 완결성(5): `/policy-design` 검증 블록 (`$EASYTRAINER_ROOT` 4-그렙).
- import smoke(5·8): `docker exec easytrainer_backend ... python3 -c 'from training_server.policies.<t>... import *'` + 1-epoch `_run_mini_train`.
- read-only(2)·경로(7): grep 게이트.

## 변경 이력
- 규칙 추가/제거 사유는 [learning.md](learning.md) 에도 남긴다.
