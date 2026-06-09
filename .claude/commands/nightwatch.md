---
description: 매뉴얼/시나리오 기반 자율 UI 검증·수선 루프 (device-free 표면). detect → fix → verify를 green까지.
argument-hint: "[--scenarios nightwatch/scenarios] [--max-rounds 3] [--url http://localhost:9000] [--fix on|off]"
---

# /nightwatch — 자율 UI 검증·수선 루프 (device-free)

foundry-harness 의 **nightwatch 패턴**([foundry-harness/patterns/nightwatch.md](../../foundry-harness/patterns/nightwatch.md))을
EasyTrainer 에 맞춰 인스턴스화한 것. 사람이 안 보는 동안 **시나리오를 따라 눌러 깨진 화면·버그를
찾아 고치는** 루프다. 현재 범위: **디바이스-프리 표면만** (튜토리얼 모드 + Dataset/Train/Planner —
실제 로봇·센서가 필요 없는 화면). 실행 모드: **인터랙티브 우선** (메인 에이전트가 detect→fix→verify
직접 수행). 무인(cron) 확장은 맨 아래 참조.

인자: `$ARGUMENTS` (모두 선택)
- `--scenarios DIR`  시나리오 디렉토리 (기본 `nightwatch/scenarios`)
- `--max-rounds N`   detect→fix→verify 최대 반복 (기본 3)
- `--url URL`        프론트엔드 base URL (기본 `http://localhost:9000`, 0단계에서 실측 확인)
- `--fix on|off`     `off` 면 detect/리포트만 (수선 안 함). 기본 `on`.

---

## 불변식 (이 루프가 인코딩 — 어기면 가짜 green)

- **생성≠평가 (제6원칙).** 합격 판정은 *결정적 평가자*(detect 단계의 관찰 가능한 기대결과)만.
  수선자가 "고쳤다"고 자기보고해도 무시하고 **반드시 detect 재실행**.
- **오라클 비침해 (제4원칙).** fix 단계는 **`nightwatch/scenarios/**` 와 `nightwatch/baselines/**`
  와 이 명령 파일을 절대 편집하지 않는다** (= 시험지). 고치는 건 *제품 소스* 뿐. 어기면 매뉴얼/
  baseline 을 고쳐 가짜 green 을 만든다.
- **매뉴얼 staleness 게이트 (제3·4원칙).** 시작 전 `bash scripts/manual-check.sh` — 기능 표면이
  매뉴얼보다 새로우면 먼저 매뉴얼부터 동기화(`/manual-update`)한다 (유령 시나리오 방지).
- **재배포 후 재검 (제4원칙).** 제품 소스를 고쳤으면 `bash scripts/quick_apply.sh ./ /opt/easytrainer/project`
  (또는 dev HMR 반영) 후 **detect 재실행**. 소스만 고치고 안 돌리는 자기기만 차단.
- **의도적 UI 변경은 baseline 재승인.** 시각 baseline 이 깨졌는데 *의도된 변경*이면 회귀가 아니다 →
  사람이 baseline 재승인할 때까지 그 시각 항목은 fail 로 보고만 하고 자동 수선하지 않는다.

---

## 0. 환경 준비 & 도달성 확인

```bash
bash scripts/smoke.sh --quick   # 컨테이너 + backend healthz. exit≠0 면 docker compose up -d 후 재시도.
```

프론트엔드 도달성 — quasar dev(vite) 기본 포트는 9000. 실제 포트를 확인:
```bash
for p in 9000 8080 3000; do
  curl -sf -o /dev/null "http://localhost:$p" && { echo "frontend @ $p"; break; }
done
```
도달 안 되면 `docker compose up -d frontend` 후 HMR 부팅(~수십초) 대기. base URL 을 확정해 `$URL` 로 사용.
앱은 hash 라우팅이라 페이지 주소는 `"$URL/#/datasets"` 형식.

> **device-free 전제.** Dataset/Train/Planner 는 **워크스페이스 선택**이 필요할 수 있다
> (없으면 "Select Workspace First" 빈 상태). 시나리오는 *빈 상태도 유효한 관찰*로 다룬다 —
> 워크스페이스/데이터가 없으면 "빈 상태 힌트가 보인다"가 기대결과다.

## 1. discover — 시나리오 수집 + staleness 게이트

```bash
bash scripts/manual-check.sh || echo "[nightwatch] 매뉴얼 stale — 시나리오 신뢰도 주의 (필요시 /manual-update 먼저)"
ls nightwatch/scenarios/*.md
```

각 시나리오 파일의 `###` 블록 = detect 가 1회 수행하는 단위. 블록 형식:
- **단계**: 누르는/입력하는 순서 (실제 버튼·필드 — i18n 라벨 또는 셀렉터)
- **기대결과**: 관찰 가능한 판정 (텍스트 노출/요소 존재/라우트 변경)
- **엣지**: 빈/에러 상태에서의 올바른 동작

> 시나리오에 `validate: first-run` 표시가 있으면 = 소스에서 유도했지만 **실측 미확인**. 첫
> detect 에서 실제 흐름과 어긋나면 **시나리오를 고치지 말고**(오라클 비침해) `nightwatch/findings/`
> 에 "scenario-mismatch" 로 기록하고 사람 검토 게이트로 넘긴다 (LLM 이 접힌 목록 등 흐름을 틀리게
> 가정할 수 있음 — 패턴의 자동제안 시나리오 함정).

## 2. detect — 결정적 평가자 (Playwright)

`playwright-skill`([../skills/playwright-skill/SKILL.md](../skills/playwright-skill/SKILL.md))로 각 시나리오를 수행.
스크립트는 `/tmp` 에 쓰고, system Chrome 로 `$URL` 을 연다.

> **러너 node 버전.** Playwright 는 node ≥18 필요. 이 호스트 기본 `node` 는 v12 라 그대로 쓰면 깨진다 —
> nvm 의 v20 을 쓴다: `export PATH="$HOME/.nvm/versions/node/v20.20.0/bin:$PATH"` 후
> `cd .claude/skills/playwright-skill && node run.js /tmp/<script>.js`. (근거: learning.md F2.)

각 블록마다:

1. 시나리오 단계대로 네비게이트/클릭/입력.
2. **기능 판정**: 기대결과의 관찰을 assert (예: `expect(page.getByText('데이터셋이 없습니다')).toBeVisible()`).
   콘솔 에러·페이지 크래시·네트워크 5xx 도 실패로 수집.
3. **시각 판정**: 스크린샷 캡처 → `Read` 로 직접 확인. baseline 이 있으면(`nightwatch/baselines/<id>.png`)
   diff, 없으면 이번 캡처를 **후보 baseline** 으로 `nightwatch/baselines/_pending/` 에만 저장
   (자동 승인 금지). 스크린샷 전 **애니메이션/트랜지션/캐럿 freeze** (안티앨리어싱 flaky 회피 —
   임계값 올리기로 때우지 말 것).

발견을 `nightwatch/findings/round_<N>.md` 에 모은다: `[scenario-id] FAIL — <관찰된 증상> @ <route>`.
green(전부 pass)이면 5단계로.

### 2.1 장시간 작업: 보이는 창 + 주기적 스크린샷 (사람이 감시 가능하게)

데이터 수집·학습처럼 **수십 분 이상 도는 흐름**은 backend script 를 직접 찌르지 말고 **사람처럼 UI 를
Playwright 로 조작**한다(센서·워크스페이스 → 수집 REC → 학습 → 추론). 그리고:

- **보이는 창으로 띄운다** — `chromium.launch({ headless: false })` + `DISPLAY=:1`. 사람이 화면에서 직접
  "지금 잘 되고 있는지" 볼 수 있어야 한다. headless 로만 돌리면 사람이 감시 못 하고 렌더 멈춤/깨짐을 놓친다.
- **같은 세션에서 주기적으로 스크린샷 + console-error 스캔** — 폴링 루프(예: 30s)마다
  `page.screenshot({ path: '/tmp/<job>-mon/latest.png' })` + `page.on('console'/'pageerror')` 로 UI 깨짐/에러를
  실측한다. `Read` 로 그 스크린샷을 주기적으로 확인해 "버그·UI 깨짐 없음"을 *결정적으로* 판정(자기보고 금지).
- **새 탭/브라우저로 모니터링하지 말 것 (단일 세션 불변식)** — 많은 SPA 는 onMount 에서 stale 프로세스를
  정리한다. EasyTrainer `App.vue` 의 `cleanup()` 은 `/stop_process` 로 실행 중인 `record_episode` 를 죽인다.
  즉 **수집/학습 중 앱을 새로 열면 그 작업이 중단된다.** 모니터링·스크린샷은 *작업을 시작한 바로 그
  페이지*에서만 하고, 별도 detect 가 필요하면 작업이 끝난 뒤에 연다.
- **부하 주의** — 단일 스레드 sim 은 host 부하에 민감하다. 수집/학습 중엔 rebuild·다중 브라우저 등
  무거운 작업을 같이 돌리지 말 것(planner/추론 성공률이 떨어진다).

## 3. fix — 수선자 (제품 소스만)

`--fix on` 일 때만. 각 FAIL 에 대해:
- 원인을 제품 소스에서 찾아 고친다 (`frontend/src/...`). **`nightwatch/**` 는 절대 안 건드림.**
- 한 라운드에 한 결함씩 좁게. 사이드 변경 금지.
- 고친 뒤 `bash scripts/quick_apply.sh ./ /opt/easytrainer/project` (dev HMR 이면 생략 가능하나
  반영 확인). golden-principles 게이트(i18n/CSS) 위반 안 만들었는지 확인 ([golden-principles.md](../../golden-principles.md)).

`scenario-mismatch`(시나리오가 틀림) / 시각 baseline 미승인 항목은 **수선 대상 아님** → 사람 검토로.

## 4. verify — detect 재실행

3단계에서 무엇이든 고쳤으면 2단계를 **그 시나리오에 대해 다시** 수행. 종료 조건은 오직 평가자 green.
`--max-rounds` 초과면 미해결 FAIL 을 남긴 채 5단계로 (무한 루프 방지).

## 5. 보고

```
nightwatch round_summary:
  url:        <확정된 base URL>
  scenarios:  <수행 수>
  passed:     <K>/<N>
  fixed:      <고친 결함 수> (PR/커밋은 만들지 않음 — 사용자 검토 후)
  unresolved: <남은 FAIL — scenario-mismatch / baseline-pending / max-rounds 초과 분류>
  baselines:  _pending 에 <M>개 (사람 승인 필요)
```

발견·수선 diff 를 사용자에게 보여주고 커밋은 사용자가. (자율 커밋은 cron 모드에서만.)

---

## 무인(cron/headless) 확장 — 아직 기본 아님

인터랙티브가 안정되면 무인 야간 모드로 승격:
- fix 단계를 headless 에이전트(`claude -p`)로, 생존(rate-limit)은 대기 후 재개.
- 오라클 비침해를 **환경으로** 강제: sparse git worktree 로 `nightwatch/{scenarios,baselines}` 를
  수선자 워크트리에서 제외(읽기 전용) — 프롬프트 부탁이 아니라 물리적 차단 (제4원칙).
- `/overnight-queue` 와 같은 결로 `scripts/overnight_summary.sh` + `db_snapshot.sh` 재사용,
  아침 `summary.md` 에 라운드 결과 누적.
- 승격 전 반드시: 인터랙티브에서 1회 이상 green 도달 + scenario-mismatch 0 (시나리오 실측 확정).

## 관련
- 패턴: [foundry-harness/patterns/nightwatch.md](../../foundry-harness/patterns/nightwatch.md)
- 시나리오: [nightwatch/README.md](../../nightwatch/README.md), `nightwatch/scenarios/*.md`
- 매뉴얼 게이트: [scripts/manual-check.sh](../../scripts/manual-check.sh)
- 규칙/채점: [golden-principles.md](../../golden-principles.md), [rubric.md](../../rubric.md)
- 하네스 지도: [docs/HARNESS.md](../../docs/HARNESS.md)
