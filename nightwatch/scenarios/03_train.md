# 03 — Train 페이지 (device-free)

학습 설정 UI. 실제 학습은 돌리지 않는다 (폼/셀렉터/도움말까지만). 디바이스 불필요.

### tr-stepper — 3-step stepper 렌더
<!-- validate: first-run -->
- route: `/#/train`
- **단계**
  1. Train 페이지로 이동. 워크스페이스가 있으면 선택.
- **기대결과**: Step 1 "Select Datasets"(i18n `trainStep1Title`) 가 활성. 데이터셋 그리드가 보이거나, 데이터셋 0개면 빈 그리드. **Continue**(i18n `continueBtn`) 버튼은 데이터셋 미선택 시 비활성.
- **엣지**: 워크스페이스 미선택 → "Select Workspace First" 힌트 (FAIL 아님).

### tr-policy-types — 정책 유형 셀렉터
<!-- validate: first-run -->
- route: `/#/train`
- **단계**
  1. 데이터셋을 하나 선택하고 **Continue** → Step 2.
  2. 정책 셀렉터(i18n `trainPolicySelect`)에서 "Create New Policy +"(i18n `trainPolicyCreateNew`) 선택.
  3. 정책 유형 셀렉터(i18n `trainPolicyType`) 펼치기.
- **기대결과**: 유형 옵션에 `ACT`, `Diffusion`, `DPDino`, `VATDino`, `PI05` 가 보인다 (modelConfigs.js POLICY_CONFIGS 와 일치). 유형 선택 시 아래에 해당 정책의 설정 폼 필드가 렌더된다.
- **엣지**: Step 1 에서 데이터셋을 안 골랐으면 Continue 가 비활성이라 여기 도달 불가 → 그 경우 tr-stepper 만 검증하고 skip.

### tr-hyperparam-help — 하이퍼파라미터 도움말 다이얼로그
<!-- validate: first-run -->
- route: `/#/train`
- **단계**
  1. Step 2 정책 설정 폼에서 아무 필드 라벨 옆 도움말 아이콘(`help_outline`, amber) 클릭.
- **기대결과**: HyperparamHelp 다이얼로그가 열린다 — 파라미터 이름 + 정책 유형 배지 + "Plain English"/"Technical Details" 섹션. 닫기(i18n `close`)로 닫힌다.
- **엣지**: 도움말 메타가 없는 필드는 아이콘이 없을 수 있음 (FAIL 아님).
