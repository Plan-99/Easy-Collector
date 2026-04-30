// 키보드 텔레옵 기본 키맵.
// 키 형식:
//   - 일반 키: 소문자 한 글자(영문) 또는 구두점 (예: 'w', '.', '[')
//   - Shift 조합: 'shift+w', 'shift+.' 등 (영문 대문자 대신 명시적 shift 플래그 사용 → CapsLock 영향 X)
//
// side: 'left' | 'right' — single arm assembly에서는 left만, dual arm에서는 양쪽 모두 활성화.
// axis: x | y | z | ax | ay | az | tool
// sign: +1 | -1
// scale: stepSize 곱 (회전축 기본 20, gripper 30)

export const DEFAULT_KEYBOARD_AXIS_MAP = {
  // ── Left arm (or single arm) ──
  w: { side: 'left', axis: 'x', sign: +1, scale: 1 },
  s: { side: 'left', axis: 'x', sign: -1, scale: 1 },
  a: { side: 'left', axis: 'y', sign: +1, scale: 1 },
  d: { side: 'left', axis: 'y', sign: -1, scale: 1 },
  e: { side: 'left', axis: 'z', sign: +1, scale: 1 },
  z: { side: 'left', axis: 'z', sign: -1, scale: 1 },
  'shift+w': { side: 'left', axis: 'ax', sign: +1, scale: 20 },
  'shift+s': { side: 'left', axis: 'ax', sign: -1, scale: 20 },
  'shift+a': { side: 'left', axis: 'ay', sign: +1, scale: 20 },
  'shift+d': { side: 'left', axis: 'ay', sign: -1, scale: 20 },
  'shift+e': { side: 'left', axis: 'az', sign: +1, scale: 20 },
  'shift+z': { side: 'left', axis: 'az', sign: -1, scale: 20 },
  c: { side: 'left', axis: 'tool', sign: +1, scale: 30 },
  v: { side: 'left', axis: 'tool', sign: -1, scale: 30 },

  // ── Right arm (dual arm assembly에서만 노출) ──
  '.': { side: 'right', axis: 'x', sign: +1, scale: 1 },
  '[': { side: 'right', axis: 'x', sign: -1, scale: 1 },
  p: { side: 'right', axis: 'y', sign: +1, scale: 1 },
  ';': { side: 'right', axis: 'y', sign: -1, scale: 1 },
  l: { side: 'right', axis: 'z', sign: +1, scale: 1 },
  "'": { side: 'right', axis: 'z', sign: -1, scale: 1 },
  'shift+.': { side: 'right', axis: 'ax', sign: +1, scale: 20 },
  'shift+[': { side: 'right', axis: 'ax', sign: -1, scale: 20 },
  'shift+p': { side: 'right', axis: 'ay', sign: +1, scale: 20 },
  'shift+;': { side: 'right', axis: 'ay', sign: -1, scale: 20 },
  'shift+l': { side: 'right', axis: 'az', sign: +1, scale: 20 },
  "shift+'": { side: 'right', axis: 'az', sign: -1, scale: 20 },
  m: { side: 'right', axis: 'tool', sign: +1, scale: 30 },
  n: { side: 'right', axis: 'tool', sign: -1, scale: 30 },
}

export const DEFAULT_KEYBOARD_SETTINGS = {
  step_size: 0.0005,
  axis_map: { ...DEFAULT_KEYBOARD_AXIS_MAP },
}

export const TELEOP_AXES = ['x', 'y', 'z', 'ax', 'ay', 'az', 'tool']

// axis index in eeDelta array sent via moveRobotEEDelta
export const AXIS_TO_EE_INDEX = {
  x: 0,
  y: 1,
  z: 2,
  ax: 3,
  ay: 4,
  az: 5,
  tool: 6,
}

// Browser KeyboardEvent → 키맵 lookup용 정규화 키.
// - 영문은 항상 lowercase (CapsLock 영향 제거)
// - shiftKey가 true이면 'shift+' 접두
export function normalizeEventKey(event) {
  let k = event.key
  if (k.length === 1 && /[a-zA-Z]/.test(k)) k = k.toLowerCase()
  return event.shiftKey ? `shift+${k}` : k
}

// 매핑 테이블 표시용
export function displayKey(k) {
  if (k.startsWith('shift+')) return `Shift + ${k.slice(6)}`
  return k
}
