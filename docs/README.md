# docs/

EasyTrainer의 **단일 정보원(Single Source of Truth)**. Slack·메모리에 흩어진 결정이 아니라
저장소에 박혀 있는 결정만 공식 기록으로 간주한다.

작업이 들어오면:
1. [CLAUDE.md](../CLAUDE.md)에서 전반적 컨벤션 확인
2. [FOLDERS.md](../FOLDERS.md)에서 어느 폴더를 건드릴지 결정
3. 이 디렉토리에서 **왜 지금 코드가 이렇게 생겼는지** 확인

## 구조

| 디렉토리 | 용도 | 수명 |
|---------|------|------|
| [design-docs/](design-docs/) | 굳어진 설계 결정. "왜 이렇게 했나" | 영구 (사실이 바뀌면 새 결정으로 덮어쓰기) |
| (`exec-plans/`) | 진행 중 작업 계획. 끝나면 `design-docs/`로 졸업 | 임시 |
| (`product-specs/`) | 페이지·기능 사양 (TrainPage, PlannerPage 등) | 페이지가 살아있는 동안 |

`(괄호)` 디렉토리는 필요해질 때 만든다 — 미리 만들지 않는다.

## design-docs/ 인덱스

| 날짜 | 문서 | 한 줄 요약 |
|------|------|----------|
| 2026-04-17 | [Remote Training Server 분리](design-docs/2026-04-17_remote-training-server.md) | 학습을 독립 Docker 서비스(포트 5100)로 분리 |
| 2026-04-28 | [Launcher 인증 & 모듈 Entitlement](design-docs/2026-04-28_launcher-auth.md) | 시리얼 키 → Google OAuth Device Flow + Bearer 토큰 |
| 2026-05-08 | [정적 / 동적 카메라 구분](design-docs/2026-05-08_camera-static-vs-dynamic.md) | crop은 정적 카메라 전용, segmentation은 둘 다 OK |
| 2026-05-29 | [Curriculum 자가 학습](design-docs/2026-05-29_curriculum-self-training.md) | Planner 위 Curriculum 계층 — 자가 롤아웃·수집·재학습 루프 |

## 그 외 문서

- [openpi_vs_easytrainer_pi05.md](openpi_vs_easytrainer_pi05.md) — pi0.5 정책 비교 분석

## 새 design-doc 작성 규칙

파일명: `YYYY-MM-DD_<slug>.md` (날짜는 결정이 굳어진 날)

본문 구조:
```markdown
# 제목

- **결정일**: YYYY-MM-DD
- **상태**: 적용 중 | 정책 | 폐기됨
- **영향 폴더**: [foo/](../../foo/), ...

## 결정
한 단락으로 핵심.

## 배경 (Why)
이 결정이 왜 필요했는지. 대안과 trade-off.

## 구조 / 적용
구체적 파일·함수·플래그.

## 운영 시 주의
자주 헷갈리는 부분. 다음 사람을 위한 경고.
```

결정이 폐기되면 파일을 지우지 말고 **상태를 "폐기됨"으로 바꾸고 후속 design-doc 링크를 맨 위에** 적는다.
