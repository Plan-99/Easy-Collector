# `.claude/agents/`

이 폴더의 마크다운 파일은 Claude Code의 **프로젝트 레벨 서브에이전트** 정의입니다.
Claude Code가 시작될 때 자동으로 로드되고, 메인 에이전트가 `Agent` 도구로
`subagent_type: <name>` 인자를 주어 호출할 수 있습니다.

## 등록된 에이전트

### manual-writer
**파일:** [manual-writer.md](manual-writer.md)
**용도:** Easy Trainer 사용자 매뉴얼(`home-next/src/app/docs/_content/ko/`) 전담.
**호출 시점:**
- 사용자가 `/manual-update` 슬래시 명령을 실행할 때 ([../commands/manual-update.md](../commands/manual-update.md))
- `frontend/src/pages/v2/`, `frontend/src/components/v2/`, `release/ui/`,
  `modules/*/module.json`, `README.md` 등이 변경된 직후 (proactive)

매뉴얼 외 파일은 절대 편집하지 않으며, 스크린샷 PNG도 만들지 않습니다.
누락 스크린샷은 `<DocsImage status="missing" />`로 표시만 합니다.
