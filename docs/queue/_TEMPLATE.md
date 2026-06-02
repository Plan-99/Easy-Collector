---
title: <한 줄 제목 — "데이터셋 페이지에 검색 필터 추가" 같은 동사형>
size: small | medium | large       # small=15–30분 / medium=30–90분 / large=2시간+
verify: smoke | e2e | none         # 검증 강도. e2e는 비싸니 정말 필요할 때만
manual: auto | skip | already      # auto=변경 파일이 매뉴얼 영향 패턴이면 자동 갱신 / skip=하지 않음 (내부 리팩터) / already=이 spec 안에 매뉴얼 변경 포함
affected:                          # 영향 폴더 (FOLDERS.md 키워드)
  - backend
  - frontend
prerequisite_specs: []             # 이 spec 전에 통과해야 할 다른 spec 파일명 (없으면 [])
---

# <title>

<!--
이 spec은 사용자가 자는 동안 Claude가 자율적으로 처리합니다.
명확하지 않은 결정은 "보수적인 선택"으로 처리되며, 그게 의도와 다르면
PR 리뷰에서 코멘트로 수정됩니다. **모호하면 spec 자체를 더 좁히세요.**
-->

## 목표 (Why)

이 변경이 왜 필요한가. 1–3 문장. 사용자/시스템의 어떤 문제가 풀리는가.

## 변경 내용 (What)

구체적으로 어떤 동작이 추가/변경/제거되는가. 가능하면 글머리표로.

- 변경 1
- 변경 2

## 영향 파일 / 폴더 (Where)

직접 손댈 것으로 예상되는 파일이나 폴더. 추측이어도 됨 — Claude가 실제로
손대는 파일이 다를 수 있다.

- `backend/api/routes/<X>.py`
- `frontend/src/pages/v2/<Y>Page.vue`

## Acceptance Criteria

**모든 항목이 통과돼야 spec이 완료된 것으로 간주됨.** 자동 검증 가능하게 적을 것.

- [ ] `bash scripts/smoke.sh` 통과
- [ ] 새 endpoint면: `curl http://localhost:5000/api/<route>` 200 반환 + 응답에 `<field>` 포함
- [ ] 새 UI라면: `frontend/` 빌드 통과 (`cd frontend && npm run build`)
- [ ] (선택) `python3 -m backend.tests.test_e2e_pipeline --skip-train` 통과

## 비-목표 (Out of Scope)

이 spec에서 *하지 않을* 것. 모호함의 가장 큰 원인을 차단.

- 리팩토링 X (이번 변경의 함수만)
- 테스트 추가 X (acceptance criteria로 충분)

## 검증 hooks (자동 실행됨)

`/feature-from-spec`이 구현 후 자동 호출. **이 블록의 명령들이 전부 exit 0이면 통과.**

```bash
# baseline은 자동으로 한 번 더 호출됨 — 여기 적지 말 것
bash scripts/smoke.sh
# 추가로 hit해야 할 endpoint가 있으면:
# curl -sf http://localhost:5000/api/<route> >/dev/null
```

## PR 제목 / 본문 (선택)

비워두면 자동 생성. 명시하고 싶으면:

- **제목**: `<제목 그대로>`
- **본문 추가**: <리뷰어가 알아야 할 컨텍스트>
