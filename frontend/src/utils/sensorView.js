// utils/sensorView.js
// ─────────────────────────────────────────────────────────────────────────
// Per-view sensor key helpers (mirrors backend/utils/sensor_view.py).
//
// 한 카메라(=물리 sensor)를 워크스페이스에서 여러 번 등록할 수 있고, 각
// 등록은 서로 다른 crop/rotate/resize 를 가진다 → 한 frame 의 여러 ROI 를
// 별도 채널처럼 다루기 위함. ``sensor_ids`` 배열에 같은 ID 가 여러 번
// 등장할 수 있고, 등장 순서로 view_key 가 결정된다.
//
// 키 규칙
//   N === 0 → view_key = String(sensor_id)         예: "5"
//   N >= 1  → view_key = `${sensor_id}_${N+1}`     예: "5_2", "5_3"
//
// 첫 view 가 suffix 없는 키인 이유: single-view 환경의 기존 task/dataset/
// checkpoint 와 bit-for-bit 같은 키를 유지 → 마이그레이션 불필요.
// ─────────────────────────────────────────────────────────────────────────

/**
 * sensor_id + occurrence index 를 view_key 문자열로.
 *
 * @param {number|string} sensorId  물리 sensor ID
 * @param {number} occurrenceIndex  같은 sensor_id 가 sensor_ids 에서 몇 번째 (0-based)
 * @returns {string}
 *
 * viewKey(5, 0) → "5"
 * viewKey(5, 1) → "5_2"
 * viewKey(7, 2) → "7_3"
 */
export function viewKey(sensorId, occurrenceIndex) {
  if (occurrenceIndex < 0) {
    throw new Error(`occurrenceIndex must be >= 0, got ${occurrenceIndex}`)
  }
  const sid = String(sensorId)
  if (occurrenceIndex === 0) return sid
  return `${sid}_${occurrenceIndex + 1}`
}

/**
 * sensor_ids 리스트를 [{sensorId, viewKey, occurrence}, ...] 로 펼침.
 *
 * Occurrence 는 입력 순서로 계산 (어떤 view 가 occurrence 0 인지는 "먼저
 * 들어온 것" 이 기준). 결과는 (sensorId, occurrence) 오름차순으로 정렬해서
 * 같은 sensor 의 view 들이 인접하게 묶인다. MonitoringWindow viewport /
 * 데이터셋 feature 순서 / 학습 dataloader / 추론 image_features 가 모두
 * 같은 ordering 을 따라 일관성 유지.
 *
 * @param {Array<number|string>} sensorIds
 * @returns {Array<{sensorId: number, viewKey: string, occurrence: number}>}
 *
 * enumerateViews([5, 7])       → [{5,"5",0}, {7,"7",0}]
 * enumerateViews([5, 7, 5])    → [{5,"5",0}, {5,"5_2",1}, {7,"7",0}]
 * enumerateViews([7, 5, 7, 5]) → [{5,"5",0}, {5,"5_2",1}, {7,"7",0}, {7,"7_2",1}]
 */
export function enumerateViews(sensorIds) {
  const seen = new Map()
  const items = []
  for (const sid of sensorIds || []) {
    const sidNum = Number(sid)
    const idx = seen.get(sidNum) || 0
    items.push({ sensorId: sidNum, viewKey: viewKey(sidNum, idx), occurrence: idx })
    seen.set(sidNum, idx + 1)
  }
  items.sort((a, b) => a.sensorId - b.sensorId || a.occurrence - b.occurrence)
  return items
}

/**
 * view_key 를 {sensorId, occurrence} 로 역변환.
 *
 * parseViewKey("5")   → {sensorId: 5, occurrence: 0}
 * parseViewKey("5_2") → {sensorId: 5, occurrence: 1}
 * parseViewKey("7_3") → {sensorId: 7, occurrence: 2}
 */
export function parseViewKey(vkey) {
  const s = String(vkey)
  const idx = s.lastIndexOf('_')
  if (idx < 0) return { sensorId: Number(s), occurrence: 0 }
  const head = s.slice(0, idx)
  const tail = s.slice(idx + 1)
  const n = Number(tail)
  if (!Number.isInteger(n) || n < 2) {
    // suffix 가 1 이거나 비정수면 single-view 로 간주 (방어적)
    return { sensorId: Number(s), occurrence: 0 }
  }
  return { sensorId: Number(head), occurrence: n - 1 }
}

/**
 * 두 view_key 가 같은 물리 센서인지.
 *
 * isSameSensor("5", "5_2") → true
 * isSameSensor("5", "7")   → false
 */
export function isSameSensor(a, b) {
  try {
    return parseViewKey(a).sensorId === parseViewKey(b).sensorId
  } catch {
    return false
  }
}
