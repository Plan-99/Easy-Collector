# 07 — Train 페이지 학습 마법사 (device-free)

데이터셋 → 정책 → 모델 학습(파라미터) 3-step 마법사. 실제 학습 시작(GPU 점유)은
하지 않는다 — step3 진입 + GPU 예상/여유 카드 + 큐 패널 렌더까지만. **tutorial_env
워크스페이스 기준.**

> 참고: 좌하단 플로팅 큐 패널(`.q-page-sticky`)이 우하단 "계속" 버튼과 겹쳐 클릭을
> 가로채므로, 자동화 시 패널을 잠시 숨기거나(`display:none`) "계속" 버튼을
> `button:visible >> text=계속` 으로 직접 집어 클릭한다.

### tr-wizard-step3 — 데이터셋→정책→모델 학습 진입
<!-- validate: first-run -->
- route: `/#/train`
- **단계**
  1. 워크스페이스 셀렉터(i18n `workspaceSelectLabel`)에서 `tutorial_env` 선택.
  2. step1(데이터셋 선택)에서 데이터셋 카드 하나 체크 → "계속"(i18n `continueBtn`).
  3. step2(정책 선택)에서 기존 정책 하나 선택 → "계속".
- **기대결과**: step3 헤더 "모델 학습"(i18n `trainStep3Title`)이 active 가 되고
  "학습 환경 설정"(i18n `trainStep3Heading`) + 학습 서버주소 입력이 보인다.
- **엣지**: 데이터셋 미선택으로 "계속" 시 경고 noti(크래시 아님). 정책 미선택도 동일.

### tr-gpu-estimate — 예상 GPU 사용량 / 남은 GPU 카드
<!-- validate: first-run -->
- route: `/#/train` (step3)
- **단계**
  1. step3 에 진입한 상태에서 학습 서버주소 카드 바로 아래 GPU 카드 확인.
- **기대결과**: "예상 GPU 사용량"(i18n `trainGpuEstimateTitle`) 카드가 보이고,
  "이 설정의 예상 사용량"(i18n `trainGpuEstimated`)에 `~NNNN MiB` 가 표시된다.
  "학습 서버 남은 GPU"(i18n `trainGpuFree`)는 서버가 GPU 메모리를 보고하면
  `free / total MiB`, 아니면 "측정 불가"(i18n `trainGpuUnknown`). 예상>여유면
  "GPU 부족 — 대기열에서 순차 실행"(i18n `trainGpuWillQueue`) 칩, 충분하면
  "여유 충분"(i18n `trainGpuFits`) 칩.
- **엣지**: batch_size / image_resolution / LoRA / mixed precision 토글 변경 시
  예상 사용량이 디바운스(0.4s) 후 갱신된다. 서버 미응답이어도 예상값은 항상 보이고
  남은 GPU 만 "측정 불가".

### tr-queue-panel — 학습 큐 플로팅 패널
<!-- validate: first-run -->
- route: `/#/train`
- **단계**
  1. 우하단 플로팅 큐 패널(`.q-page-sticky`) 확인.
- **기대결과**: 새로고침 버튼 + 실행 중(deep-orange `model_training`) / 대기(`schedule`)
  버튼이 보인다. GPU 여유 시 **동시 학습**이 가능하므로 실행 중 버튼이 여러 개
  나타날 수 있다(`running_list`). 진행 중 학습이 없으면 새로고침 버튼만.
- **엣지**: 백엔드 `/train/queue` 가 `running_list` 를 안 주는 구버전이어도 단일
  `running` 으로 폴백 렌더(크래시 아님).
