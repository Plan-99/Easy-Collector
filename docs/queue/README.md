# docs/queue/

자율 운영(`/overnight-queue`)이 야간에 처리할 **작업 큐**.

각 항목은 한 개의 markdown 파일 (`NN_<slug>.md`)로, [_TEMPLATE.md](_TEMPLATE.md)
포맷을 따릅니다. 우선순위는 파일명 prefix 숫자 오름차순 (`01_xxx.md` → `02_xxx.md`).

## 디렉토리 컨벤션

```
docs/queue/
├── README.md              # 이 파일
├── _TEMPLATE.md           # 새 spec 작성 시 복사 시작점
├── 01_<slug>.md           # 처리 대기
├── 02_<slug>.md           # 처리 대기
├── _in-progress/          # /overnight-queue가 작업 시작 시 이동
│   └── 03_<slug>.md
├── _done/                 # PR 머지 또는 통과 후 이동
│   └── 00_<old-slug>.md
└── _failed/               # 검증 실패한 spec — 일어나서 검토
    └── 04_<slug>.md       (실패 사유는 야간 summary에)
```

**상태는 위치로 표현**. frontmatter `status:` 필드를 따로 두지 않는다 (둘 다
관리하면 어긋남).

## 새 spec 작성 절차

```bash
cp docs/queue/_TEMPLATE.md docs/queue/05_my_feature.md
$EDITOR docs/queue/05_my_feature.md
```

작성 후 `git status`로 확인만 하고 commit은 불필요 (untracked 상태로 야간 큐에서 픽업됨).

## 처리 정책

- **순서**: 파일명 prefix 숫자 오름차순. 같은 prefix면 사전식.
- **격리**: `/feature-from-spec`이 spec 1개당 worktree 1개 생성.
- **매뉴얼 자동 갱신**: 검증 통과 후, 변경 파일이 사용자 매뉴얼 영향 패턴(`frontend/src/pages/v2/`, `frontend/src/components/v2/`, `release/ui/`, `modules/*/module.json`, `backend/api/routes/`, `README.md`)과 매칭되면 `manual-writer` 서브에이전트가 같은 PR에 매뉴얼 변경을 추가. spec frontmatter `manual:` 로 제어 (`auto` 기본 / `skip` / `already`).
- **실패 시**: 해당 spec은 `_failed/`로 이동. worktree는 보존하고 다음 spec으로.
- **연속 실패 2회**: `/overnight-queue`가 자체 중단 (인프라 문제 의심).
- **빈 큐**: 즉시 종료 후 summary에 "queue empty" 기록.

> 매뉴얼 갱신은 **논블로커** — 매뉴얼 서브에이전트가 실패해도 코드 PR 은 정상 진행됩니다. summary 의 note 컬럼에 `manual: skipped (<사유>)` 가 남습니다.

## 운영 시 주의

- 미완성 spec은 prefix를 `99_`로 두거나 `_drafts/` 디렉토리에. 자율 루프가 건드리지 않는다.
- spec 파일 안에 *비밀(API 키, 토큰)* 적지 말 것. PR 본문에 통째로 인용될 수 있음.
- "이거 너무 큼" 싶으면 잘라서 두 개 spec으로. 한 사이클이 ~1시간 안에 끝나야 6시간에 4–6개 처리.
