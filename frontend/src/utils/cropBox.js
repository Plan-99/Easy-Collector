// utils/cropBox.js
// ─────────────────────────────────────────────────────────────────────────
// Reusable drag-to-draw bounding box logic. Extracted from
// WorkspacePage.vue's sensor crop UI so both pages (workspace, datasets
// batch-edit) can share the same coordinate conversion + clamping.
//
// Usage (Composition API):
//   const crop = useCropBox(() => ({
//       resolution: focused.resolution,            // [width, height] in pixels
//       containerEl: containerRef.value,           // bounding ancestor (mouse coords relative to this)
//       innerEl: () => containerRef.value?.querySelector('img'),  // actual image element
//   }))
//   crop.onMouseDown(e); crop.onMouseMove(e); crop.onMouseUp() ...
//   crop.box                                       // ref<[x1,y1,x2,y2] | null>
//   crop.style                                     // computed CSS for the overlay rect
//   crop.reset()
// ─────────────────────────────────────────────────────────────────────────
import { ref, computed } from 'vue'

export function useCropBox(getCtx) {
  // Context provider so resolution / DOM refs can update over time without
  // re-creating the composable. Returns:
  //   { resolution: [w,h], containerEl: HTMLElement, innerEl: () => HTMLElement | null }
  const _ctx = () => (typeof getCtx === 'function' ? getCtx() : getCtx) || {}

  const isDragging = ref(false)
  const start = ref({ x: 0, y: 0 })
  const end = ref({ x: 0, y: 0 })
  // Persisted box in *source pixel* coordinates: [x1, y1, x2, y2]
  const box = ref(null)

  function _containerRect() {
    const el = _ctx().containerEl
    return el ? el.getBoundingClientRect() : null
  }

  function _innerRect() {
    const fn = _ctx().innerEl
    const el = typeof fn === 'function' ? fn() : fn
    return el ? el.getBoundingClientRect() : null
  }

  function onMouseDown(event) {
    const rect = _containerRect()
    if (!rect) return
    isDragging.value = true
    start.value = { x: event.clientX - rect.left, y: event.clientY - rect.top }
    end.value = { ...start.value }
  }

  function onMouseMove(event) {
    if (!isDragging.value) return
    const rect = _containerRect()
    if (!rect) return
    end.value = { x: event.clientX - rect.left, y: event.clientY - rect.top }
  }

  function onMouseUp() {
    if (!isDragging.value) return
    isDragging.value = false
    const persisted = currentBoxInSourcePx()
    if (persisted) box.value = persisted
  }

  function cancel() {
    isDragging.value = false
    start.value = { x: 0, y: 0 }
    end.value = { x: 0, y: 0 }
  }

  function reset() {
    cancel()
    box.value = null
  }

  // Set the box from existing source-px coords (e.g. loading from manifest).
  function setBox(srcBox) {
    if (!srcBox || srcBox.length !== 4) {
      box.value = null
      return
    }
    box.value = srcBox.map((v) => Math.round(v))
  }

  // CSS overlay style — only valid while dragging or for the persisted box.
  const dragStyle = computed(() => {
    if (!isDragging.value) return null
    const left = Math.min(start.value.x, end.value.x)
    const top = Math.min(start.value.y, end.value.y)
    const width = Math.abs(start.value.x - end.value.x)
    const height = Math.abs(start.value.y - end.value.y)
    return {
      position: 'absolute',
      left: `${left}px`,
      top: `${top}px`,
      width: `${width}px`,
      height: `${height}px`,
    }
  })

  // Style for the *persisted* (committed) box — overlays the inner image.
  const persistedStyle = computed(() => {
    if (!box.value || !box.value.length) return null
    const innerRect = _innerRect()
    const containerRect = _containerRect()
    const res = _ctx().resolution || []
    if (!innerRect || !containerRect || !res[0] || !res[1]) return null

    const scaleX = innerRect.width / res[0]
    const scaleY = innerRect.height / res[1]
    const offsetX = innerRect.left - containerRect.left
    const offsetY = innerRect.top - containerRect.top

    const [x1, y1, x2, y2] = box.value
    return {
      position: 'absolute',
      left: `${offsetX + x1 * scaleX}px`,
      top: `${offsetY + y1 * scaleY}px`,
      width: `${(x2 - x1) * scaleX}px`,
      height: `${(y2 - y1) * scaleY}px`,
    }
  })

  // Compute the current (dragged) box in source-pixel coordinates.
  function currentBoxInSourcePx() {
    const innerRect = _innerRect()
    const containerRect = _containerRect()
    const res = _ctx().resolution || []
    if (!innerRect || !containerRect || !res[0] || !res[1]) return null

    const scaleX = res[0] / innerRect.width
    const scaleY = res[1] / innerRect.height
    const offsetX = innerRect.left - containerRect.left
    const offsetY = innerRect.top - containerRect.top

    // Drag rect in container coords → inner coords → source pixels.
    const x1c = Math.min(start.value.x, end.value.x) - offsetX
    const y1c = Math.min(start.value.y, end.value.y) - offsetY
    const x2c = Math.max(start.value.x, end.value.x) - offsetX
    const y2c = Math.max(start.value.y, end.value.y) - offsetY

    let x1 = Math.round(x1c * scaleX)
    let y1 = Math.round(y1c * scaleY)
    let x2 = Math.round(x2c * scaleX)
    let y2 = Math.round(y2c * scaleY)

    x1 = Math.max(0, Math.min(res[0], x1))
    y1 = Math.max(0, Math.min(res[1], y1))
    x2 = Math.max(0, Math.min(res[0], x2))
    y2 = Math.max(0, Math.min(res[1], y2))
    // Degenerate boxes get rejected so we don't store noise from a stray click.
    if (x2 - x1 < 2 || y2 - y1 < 2) return null
    return [x1, y1, x2, y2]
  }

  return {
    // state
    isDragging,
    box,
    // event handlers
    onMouseDown,
    onMouseMove,
    onMouseUp,
    cancel,
    reset,
    setBox,
    // styles
    dragStyle,
    persistedStyle,
  }
}
