import { ref } from 'vue'
import { AXIS_TO_EE_INDEX } from 'src/configs/teleopDefaults'

// 키보드 텔레옵 공용 로직 (MonitoringWindow=record, TeleopConsole=텔레옵 페이지 공유).
//
// 동작:
//  - 눌린 키 집합(heldKeys)을 추적하고 일정 주기(tick)마다 합산 델타를 전송 →
//    w+a 동시 입력이면 한 번에 대각선(합산) 이동.
//  - 같은 로봇으로 가는 여러 EE(dual_arm 의 L_ee/R_ee)는 **하나의 delta_pos** 로
//    묶어 보낸다 → backend 가 두 EE 를 동시에 IK 로 푼다(따로 보내면 한쪽을 현재
//    포즈로 고정해 두 팔이 같이 안 움직임).
//  - tool 축은 arm 내장 그리퍼(tool_inner)면 EE delta[6], 아니면 외부 그리퍼
//    agent 의 joint delta 로 라우팅.
//  - keyup/window blur 로 키 제거(멈춤), Space 로 전체 정지.
//
// 옵션:
//   getAxisMap()   → 키맵 (axis_map)
//   getStepSize()  → step size (number)
//   robotForSide(side) / toolForSide(side)
//   canHandle?()        → false 면 입력 무시 (예: home pose 이동 중)
//   shouldCapture?(e)   → false 면 이 이벤트 무시 (예: input 편집 중). 부수효과(blur) 허용
//   onKeyDownLog?(entry, normKey)  → 새 키 눌림 1회 로그
//   onStop?()           → Space 정지 시
//   onError?(name, err)
//   tickMs?             → 기본 50 (20Hz)
const TELEOP_TICK_MS = 50

function baseFromEvent(event) {
  const k = event.key
  return k.length === 1 && /[a-zA-Z]/.test(k) ? k.toLowerCase() : k
}

// dual_arm 단일 로봇은 한 agent 가 두 EE 를 들고 있어 side→EE 이름 매핑 필요.
function eeKeyFor(robot, side) {
  if (robot.role === 'dual_arm') return side === 'right' ? 'R_ee' : 'L_ee'
  return 'ee'
}

export function useKeyboardTeleop(opts) {
  const {
    getAxisMap,
    getStepSize,
    robotForSide,
    toolForSide = () => null,
    canHandle = () => true,
    shouldCapture = () => true,
    onKeyDownLog = () => {},
    onStop = () => {},
    onError = () => {},
    tickMs = TELEOP_TICK_MS,
  } = opts

  const heldKeys = new Set()
  let shiftHeld = false
  let timer = null
  const active = ref(false)

  function tick() {
    if (!canHandle()) return
    const map = getAxisMap() || {}
    const stepSize = Number(getStepSize()) || 0.003
    const armAcc = new Map() // robot.id -> { robot, deltaPos: {eeName: number[]} }
    const toolAcc = new Map() // tool.id  -> { tool, delta }
    for (const base of heldKeys) {
      const entry = map[shiftHeld ? `shift+${base}` : base]
      if (!entry) continue
      const side = entry.side || 'left'
      const robot = robotForSide(side)
      if (!robot) continue
      const idx = AXIS_TO_EE_INDEX[entry.axis]
      if (idx === undefined) continue
      const d = stepSize * (Number(entry.scale) || 1) * (Number(entry.sign) || 1)
      if (entry.axis === 'tool' && !robot.tool_inner) {
        const tool = toolForSide(side)
        if (tool?.handler?.moveRobotJointDelta) {
          const cur = toolAcc.get(tool.id) || { tool, delta: 0 }
          cur.delta += d
          toolAcc.set(tool.id, cur)
        }
        continue
      }
      const len = robot.tool_inner ? 7 : 6
      if (idx >= len) continue
      const eeName = eeKeyFor(robot, side)
      let a = armAcc.get(robot.id)
      if (!a) {
        a = { robot, deltaPos: {} }
        armAcc.set(robot.id, a)
      }
      if (!a.deltaPos[eeName]) a.deltaPos[eeName] = Array(len).fill(0)
      a.deltaPos[eeName][idx] += d
    }
    armAcc.forEach((a) => {
      const dp = {}
      let any = false
      for (const k of Object.keys(a.deltaPos)) {
        if (a.deltaPos[k].some((v) => Math.abs(v) > 1e-12)) {
          dp[k] = a.deltaPos[k]
          any = true
        }
      }
      if (any && a.robot.handler?.moveRobotEEDelta) {
        try {
          a.robot.handler.moveRobotEEDelta(dp)
        } catch (e) {
          onError(a.robot.name, e)
        }
      }
    })
    toolAcc.forEach((tc) => {
      if (Math.abs(tc.delta) < 1e-12) return
      const tlen = (Array.isArray(tc.tool.joint_names) && tc.tool.joint_names.length) || 1
      const jd = Array(tlen).fill(0)
      jd[0] = tc.delta
      try {
        tc.tool.handler.moveRobotJointDelta(jd)
      } catch (e) {
        onError(tc.tool.name, e)
      }
    })
  }

  function startTimer() {
    if (!timer) timer = setInterval(tick, tickMs)
  }
  function stopTimer() {
    if (timer) {
      clearInterval(timer)
      timer = null
    }
  }
  function clearHeld() {
    heldKeys.clear()
    stopTimer()
  }

  // 전체 정지: 로봇별로 양쪽 EE 를 0 으로 한 번에 보낸다.
  function stopAllArms() {
    const byRobot = new Map()
    ;['left', 'right'].forEach((side) => {
      const robot = robotForSide(side)
      if (!robot?.handler?.moveRobotEEDelta) return
      const len = robot.tool_inner ? 7 : 6
      let a = byRobot.get(robot.id)
      if (!a) {
        a = { robot, dp: {} }
        byRobot.set(robot.id, a)
      }
      a.dp[eeKeyFor(robot, side)] = Array(len).fill(0)
    })
    byRobot.forEach((a) => {
      try {
        a.robot.handler.moveRobotEEDelta(a.dp)
      } catch (e) {
        onError(a.robot.name, e)
      }
    })
  }

  function onKeyDown(event) {
    if (!canHandle()) return
    if (!shouldCapture(event)) return
    shiftHeld = event.shiftKey
    if (event.key === ' ' || event.key === 'Space') {
      clearHeld()
      stopAllArms()
      onStop()
      event.preventDefault()
      return
    }
    const base = baseFromEvent(event)
    if (['Shift', 'Control', 'Alt', 'Meta'].includes(base)) return
    const map = getAxisMap() || {}
    const normKey = shiftHeld ? `shift+${base}` : base
    const entry = map[normKey]
    if (!entry && !map[base]) return // 제어키만 추적
    if (!heldKeys.has(base) && entry) onKeyDownLog(entry, normKey)
    heldKeys.add(base)
    startTimer()
    tick() // 즉시 1회 반응 (탭에도 움직이도록)
    event.preventDefault()
  }

  function onKeyUp(event) {
    shiftHeld = event.shiftKey
    if (event.key === 'Shift') shiftHeld = false
    heldKeys.delete(baseFromEvent(event))
    if (heldKeys.size === 0) stopTimer()
  }

  function start() {
    stop()
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', clearHeld)
    active.value = true
  }

  function stop() {
    window.removeEventListener('keydown', onKeyDown)
    window.removeEventListener('keyup', onKeyUp)
    window.removeEventListener('blur', clearHeld)
    clearHeld()
    active.value = false
  }

  return { start, stop, active }
}
