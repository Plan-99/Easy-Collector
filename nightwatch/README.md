# nightwatch/ — 자율 UI 검증·수선 루프 자산

[`/nightwatch`](../.claude/commands/nightwatch.md) 명령이 쓰는 시나리오·baseline·발견 기록.
foundry-harness 의 nightwatch 패턴 인스턴스화. **현재 범위: device-free 표면**(튜토리얼 모드 +
Dataset/Train/Planner — 실제 로봇·센서 불필요).

## 디렉토리

```
nightwatch/
├── README.md            # 이 파일
├── scenarios/*.md       # detect 가 따라 누르는 시나리오 (### 블록 = 1 시나리오)
├── baselines/           # 승인된 시각 baseline PNG
│   └── _pending/        # detect 가 새로 캡처한 후보 (사람 승인 전까지 비교에 안 씀)
└── findings/round_<N>.md  # 라운드별 FAIL / scenario-mismatch 기록
```

## 시나리오 블록 형식

각 `###` 블록 = detect 1회 단위.

```
### <scenario-id> — <한 줄 목적>
<!-- validate: first-run | confirmed -->   ← first-run = 소스에서 유도, 실측 미확인
- route: /#/<hash-route>
- **단계**: 1. … 2. …  (실제 버튼/필드 — i18n 라벨 또는 셀렉터)
- **기대결과**: <관찰 가능한 판정> (텍스트 노출 / 요소 존재 / 라우트 변경)
- **엣지**: <빈/에러 상태에서 올바른 동작>
```

## 오라클 비침해 (절대 규칙)

`/nightwatch` 의 **fix 단계는 이 폴더(`scenarios/`·`baselines/`)를 절대 편집하지 않는다.** 여기는
"시험지"다. 수선은 제품 소스(`frontend/src/...`)만. 시나리오가 실제 흐름과 다르면(=scenario-mismatch)
고치지 말고 `findings/` 에 기록하고 **사람 검토 게이트**로 넘긴다. (LLM 이 접힌 목록·조건부 렌더를
틀리게 가정하는 패턴 함정 방지.)

## 현재 시나리오 신뢰도

전부 `validate: first-run` — `frontend/src/pages/v2/*.vue` 소스에서 유도했고 라이브 실행으로는
아직 확정 안 됨. 첫 detect 에서 흐름이 어긋나면 confirmed 로 승격하기 전 사람이 검토한다.
("Settings" 화면은 전역 페이지로 **존재하지 않아** 시나리오에서 제외 — 지어내지 않음.)
