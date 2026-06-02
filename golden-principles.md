# Golden Principles — EasyTrainer

> (foundry-harness/templates/golden-principles.md.tmpl 인스턴스화.)
> 규칙은 **기계적으로 검증 가능한 것만** 적는다. "좋게 짜라" 같은 훈계는 금지.
> 각 규칙에 *왜* 와 *어기면 어떻게 드러나는지(위반 감지)* 를 붙인다. 규칙은 세금이다 —
> 실제 실패 사례(또는 운영 정책)가 있는 것만 남긴다. 자율 운영 지도: [docs/HARNESS.md](docs/HARNESS.md),
> 실패/진화 로그: [learning.md](learning.md).

## 규칙

1. **호스트 절대경로 금지.** 명령·스크립트·훅은 repo-root 상대경로 또는
   `$(git rev-parse --show-toplevel)` / `$CLAUDE_PROJECT_DIR` 로 루트를 도출한다.
   — 왜: 하드코딩된 `/home/<user>/...` 는 다른 체크아웃에서 조용히 깨진다.
   / 위반 감지: `grep -rn "/home/[a-z]*/" .claude/ scripts/` 가 비어야 함 (예외: `git -C "$BASE"`). 근거: [learning.md](learning.md) F1.

2. **i18n 키는 ko-KR / en-US 양쪽에.** 프론트엔드에서 쓰는 `$t('key')` 키는
   `frontend/src/i18n/{ko-KR,en-US}/index.js` 두 곳 모두에 존재해야 한다.
   — 왜: 한쪽만 추가하면 다른 로케일에서 키 노출. / 위반 감지: `.claude/hooks/frontend-i18n-check.sh` (편집 시).

3. **Vue SFC 에 custom CSS 금지.** `<style>` 블록·생 `.css/.scss` 대신 Quasar 유틸리티/컴포넌트.
   동적 위치만 `:style="..."` 인라인 바인딩 허용 (예외: `quasar.variables.scss`, `app.scss`).
   — 왜: 프로젝트 스타일 일관성. / 위반 감지: `.claude/hooks/frontend-no-css.sh`.

4. **UI 기능 변경은 Playwright 검증까지가 "완료".** 렌더 확인(스크린샷 → Read) + 인터랙션 시나리오 + 콘솔/네트워크 에러 관찰.
   — 왜: 안 띄워보면 깨진 채로 머지. / 위반 감지: `.claude/hooks/frontend-ui-test-reminder.sh` (세션당 1회 권고).

5. **기능 표면 변경 시 매뉴얼 동기화.** 기능 표면(아래 trigger glob)이 매뉴얼보다 새로우면 게이트가 빨간불.
   — 왜: 유령 매뉴얼/누락 매뉴얼 방지. / 위반 감지: `bash scripts/manual-check.sh` (exit≠0). 처방: `/manual-update`.

6. **`backend/lerobot/` 는 read-only.** 새 정책은 import + composition 으로만. 활성화/introspection 은 forward_hook 외부 부착.
   — 왜: 벤더 트리 수정은 업스트림/다른 정책의 기준선을 흔든다. / 위반 감지: `git diff --name-only -- backend/lerobot/` 가 정책 작업 PR 에서 비어야 함. 근거: memory `lerobot_readonly.md`.

7. **자동 파이프라인의 성공은 scene state 로 판정 (ground-truth).** 정책 done 토큰이 아니라 ground-truth 체크로 판정하고, 같은 체커가 학습 데이터 필터 + 추론 성공률에 모두 쓰인다.
   — 왜: self-report 성공은 0% 를 100% 로 보고한다. / 위반 감지: model_tester 의 success-check 경로 일치 (memory `feedback_success_check.md`).

8. **새 라우트 블루프린트 추가 시 `scripts/smoke.sh` GET 목록에 한 줄 추가.**
   — 왜: 안 하면 새 라우트가 깨져도 smoke 가 못 잡는다. / 위반 감지: 새 `backend/api/routes/*.py` PR 에서 `scripts/smoke.sh` diff 동반 여부.

9. **검증 테스트는 happy-path 만이 아니라 경계·악성 입력을 최소 1케이스 포함.** (빈/최대길이/0/결측/비ASCII)
   — 왜: 짧은 픽스처는 실데이터 엣지 버그를 통과시킨다. / 위반 감지: 테스트 케이스 리뷰 (근거: coin-tournament m8 ZeroDivision, insta-horror F3/F4).

## 검증 (이 규칙들을 기계로 확인)

- 훅(2·3·4): Write/Edit 시 자동 발화 — `.claude/settings.json` 배선, 루트는 동적 도출.
- 매뉴얼 게이트(5): `bash scripts/manual-check.sh`.
- 인프라(8): `bash scripts/smoke.sh`.
- 경로(1): `grep -rn "/home/[a-z]*/" .claude/ scripts/` (예외만 남아야 함).

## 변경 이력

- 규칙 추가/제거 시 사유를 [learning.md](learning.md) 에도 남긴다 (특히 `[pruned]`).
