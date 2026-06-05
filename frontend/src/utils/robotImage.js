// 로봇 카드/조립 화면의 이미지 경로 결정.
//
// 기본은 company 단위(`/images/<company>.png`)지만, 같은 company 아래 여러 모델이 있으면
// (예: ROBOTIS 의 omx / omy / ai_worker) company 로는 구분이 안 된다. 그래서 robot.type
// → 이미지 베이스 매핑을 우선 적용하고, 없으면 company 로 폴백한다.
//
// 이미지 파일은 frontend/public/images/<base>.png. home-next 모듈 카탈로그
// (home-next/public/modules/<base>.png, ModuleCatalog.tsx 의 MODULE_IMAGE)와는 배포가
// 분리돼 런타임 공유가 안 되므로 각 public 에 수동으로 둔다(중복 허용).
const TYPE_IMAGE = {
  omx_f: 'omx',
  omy_3m: 'omy',
  omy_f3m: 'omy',
  ffw_bg2: 'aiworker',
  ffw_sg2: 'aiworker',
}

export function robotImage(robot) {
  const base = TYPE_IMAGE[robot?.type] || robot?.company
  return '/images/' + base + '.png'
}
