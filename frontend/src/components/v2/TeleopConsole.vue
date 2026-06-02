<template>
    <div class="row q-col-gutter-md teleop-console">
        <div class="col-auto column q-gutter-y-sm" style="min-width: 260px;">
            <!-- Robot ON buttons (per-robot) -->
            <TutorialHint step="2" :text="$t('tutorialTeleopRobotOn')" />
            <div class="row no-wrap robot-buttons-row">
                <q-btn
                    v-for="robot in assemblyRobots"
                    :key="robot.id"
                    class="col"
                    outline
                    no-caps
                    icon="power_settings_new"
                    :color="robot.status === 'on' ? 'positive' : 'grey-6'"
                    :label="robot.name"
                    :loading="robot.status === 'loading'"
                    @click="toggleRobot(robot)"
                />
            </div>

            <!-- Method select -->
            <TutorialHint step="3" :text="$t('tutorialTeleopMethod')" />
            <q-select
                v-model="teleopMethod"
                :options="methodOptions"
                emit-value
                map-options
                outlined
                dark
                dense
                :label="$t('teleopMethod')"
            />

            <!-- Start / Stop -->
            <TutorialHint step="4" :text="$t('tutorialTeleopStart')" />
            <q-btn
                v-if="!teleopActive"
                color="primary"
                icon="play_arrow"
                :label="$t('teleopStartTeleoperation')"
                :disable="!allRobotsOn"
                @click="startTeleop"
                outline
            />
            <q-btn
                v-else
                color="orange-8"
                icon="stop"
                :label="$t('teleopStopTeleoperation')"
                @click="stopTeleop"
                outline
            />

            <div class="text-caption text-grey-5 q-mt-sm" v-if="teleopActive && teleopMethod === 'keyboard'">
                <template v-if="isDualArm">
                    <div>{{ $t('teleopLeftLabel') }}: <span class="text-primary">{{ leftArm.name }}</span></div>
                    <div>{{ $t('teleopRightLabel') }}: <span class="text-primary">{{ rightArm.name }}</span></div>
                </template>
                <template v-else>
                    <div>{{ $t('teleopArmLabel') }}: <span class="text-primary">{{ (leftArm || rightArm)?.name }}</span></div>
                </template>
            </div>
        </div>

        <div class="col">
            <q-scroll-area
                ref="logScroll"
                class="bg-dark border-rounded q-pa-sm"
                style="height: 280px; width: 100%;"
            >
                <div
                    v-for="(line, i) in logs"
                    :key="i"
                    class="text-caption"
                    :class="logClass(line)"
                >{{ line.text }}</div>
            </q-scroll-area>
        </div>
    </div>
</template>

<script setup>
import { ref, computed, defineProps, onUnmounted, onMounted, nextTick, watch } from 'vue'
import { useI18n } from 'vue-i18n'
import { useLeaderTeleoperation } from 'src/composables/useLeaderTeleoperation'
import { useProcessStore } from 'src/stores/processStore'
import { DEFAULT_KEYBOARD_SETTINGS, AXIS_TO_EE_INDEX, normalizeEventKey, displayKey } from 'src/configs/teleopDefaults'
import TutorialHint from 'src/components/v2/TutorialHint.vue'

const props = defineProps({
    assembly: { type: Object, required: true },
})

const { t } = useI18n()
const { leaderTeleStarted, startLeaderTele, stopLeaderTele } = useLeaderTeleoperation()
const processStore = useProcessStore()

// ───── Robot resolution ─────
const leftArm = computed(() => props.assembly.left_arm || null)
const rightArm = computed(() => props.assembly.right_arm || null)
const isDualArm = computed(() => !!(leftArm.value && rightArm.value))

const assemblyRobots = computed(() => {
    const list = []
    const seen = new Set()
    ;['left_arm', 'left_tool', 'right_arm', 'right_tool'].forEach((part) => {
        const r = props.assembly[part]
        if (!r || seen.has(r.id)) return
        seen.add(r.id)
        list.push(r)
    })
    return list
})

const allRobotsOn = computed(() =>
    assemblyRobots.value.length > 0 && assemblyRobots.value.every((r) => r.status === 'on')
)

function robotForSide(side) {
    if (isDualArm.value) {
        return side === 'right' ? rightArm.value : leftArm.value
    }
    if (side === 'right') return null
    return leftArm.value || rightArm.value
}

// 별도 그리퍼(role=tool) agent. arm이 tool_inner=false 인데 그리퍼를 함께 텔레옵하고
// 싶을 때(예: fairino arm + robotiq) 키보드의 tool 축 입력을 이 agent로 라우팅한다.
function toolForSide(side) {
    const left = props.assembly.left_tool || null
    const right = props.assembly.right_tool || null
    if (isDualArm.value) {
        return side === 'right' ? right : left
    }
    // 단일 팔: 어느 쪽 키 binding 이든 존재하는 tool 로 fallback.
    return left || right
}

function toggleRobot(robot) {
    if (!robot.handler) return
    if (robot.status === 'on') {
        if (robot.handler.stopRobot) robot.handler.stopRobot()
    } else if (robot.status === 'loading') {
        // 로딩 중 토글 = 진행 중인 시작 취소.
        if (robot.handler.stopRobot) robot.handler.stopRobot()
    } else {
        if (robot.handler.startRobot) robot.handler.startRobot()
    }
}

// ───── Method selection ─────
const hasLeaderSetting = computed(() =>
    !!props.assembly.teleoperators?.find((tt) => tt.type === 'leader')
)
const methodOptions = computed(() => {
    const opts = [{ label: t('teleopMethodKeyboard'), value: 'keyboard' }]
    if (hasLeaderSetting.value) {
        opts.push({ label: t('teleopMethodLeader'), value: 'leader' })
    }
    return opts
})
const teleopMethod = ref('keyboard')

// 양팔 모드에 따른 자동 보정 (만약 leader 설정이 없는 상태에서 leader 선택돼있으면 keyboard로 fallback)
watch(hasLeaderSetting, (v) => {
    if (!v && teleopMethod.value === 'leader') teleopMethod.value = 'keyboard'
})

// ───── Unified log buffer ─────
const logs = ref([])
const logScroll = ref(null)

function pushLog(text, type = 'info') {
    logs.value.push({ text: `[${new Date().toLocaleTimeString()}] ${text}`, type })
    if (logs.value.length > 500) logs.value.splice(0, logs.value.length - 500)
    nextTick(() => {
        if (logScroll.value) {
            const el = logScroll.value.getScrollTarget()
            if (el) el.scrollTop = el.scrollHeight
        }
    })
}

function logClass(line) {
    if (line.type === 'error' || line.type === 'stderr') return 'text-red-5'
    if (line.type === 'leader') return 'text-cyan-3'
    return 'text-white'
}

let _leaderSeen = 0
watch(
    () => processStore.taskLogs['leader_teleoperation']?.length || 0,
    (newLen) => {
        const arr = processStore.taskLogs['leader_teleoperation'] || []
        for (let i = _leaderSeen; i < newLen; i++) {
            const entry = arr[i]
            const isErr = entry.type === 'stderr'
            logs.value.push({
                text: `[LEADER] ${entry.message}`,
                type: isErr ? 'error' : 'leader',
            })
        }
        _leaderSeen = newLen
        if (logs.value.length > 500) logs.value.splice(0, logs.value.length - 500)
        nextTick(() => {
            if (logScroll.value) {
                const el = logScroll.value.getScrollTarget()
                if (el) el.scrollTop = el.scrollHeight
            }
        })
    }
)

onMounted(() => {
    _leaderSeen = processStore.taskLogs['leader_teleoperation']?.length || 0
})

// ───── Keyboard teleop runtime ─────
const keyboardSetting = computed(() => {
    const s = props.assembly.teleoperators?.find((tt) => tt.type === 'keyboard')?.settings
    return s && s.axis_map ? s : DEFAULT_KEYBOARD_SETTINGS
})

const keyboardActive = ref(false)
const activeArms = computed(() => [leftArm.value, rightArm.value].filter(Boolean))

function keyboardHandler(event) {
    const tag = (event.target?.tagName || '').toLowerCase()
    if (tag === 'input' || tag === 'textarea') return

    const map = keyboardSetting.value.axis_map || {}
    const stepSize = Number(keyboardSetting.value.step_size) || 0.003

    if (event.key === ' ' || event.key === 'Space') {
        activeArms.value.forEach((robot) => {
            const len = robot.tool_inner ? 7 : 6
            sendDelta(robot, Array(len).fill(0))
        })
        pushLog(t('teleopLogStopAllArms'))
        event.preventDefault()
        return
    }

    const normKey = normalizeEventKey(event)
    const entry = map[normKey]
    if (!entry) return

    const side = entry.side || 'left'
    const robot = robotForSide(side)

    const idx = AXIS_TO_EE_INDEX[entry.axis]
    if (idx === undefined) return

    // tool 축: arm 내장 그리퍼면 EE delta[6] 으로 packing, 별도 gripper agent 가
    // 있으면 그 agent 의 joint delta 로 라우팅. 둘 다 없으면 무시.
    if (entry.axis === 'tool') {
        const delta = stepSize * (Number(entry.scale) || 1) * (Number(entry.sign) || 1)
        if (robot && robot.tool_inner) {
            const eeDelta = Array(7).fill(0)
            eeDelta[idx] = delta
            sendDelta(robot, eeDelta)
            pushLog(t('teleopLogKeyDelta', {
                side, name: robot.name,
                key: displayKey(normKey), axis: entry.axis,
                sign: entry.sign > 0 ? '+' : '-',
                delta: eeDelta[idx].toFixed(5),
            }))
            event.preventDefault()
            return
        }
        const tool = toolForSide(side)
        if (!tool || !tool.handler || !tool.handler.moveRobotJointDelta) return
        const len = Array.isArray(tool.joint_names) && tool.joint_names.length > 0
            ? tool.joint_names.length : 1
        const jointDelta = Array(len).fill(0)
        jointDelta[0] = delta  // 단일 knuckle / 단일 finger joint 가정
        try {
            tool.handler.moveRobotJointDelta(jointDelta)
        } catch (e) {
            pushLog(t('teleopLogSendError', { name: tool.name, error: e.message }), 'error')
            return
        }
        pushLog(t('teleopLogKeyDelta', {
            side, name: tool.name,
            key: displayKey(normKey), axis: entry.axis,
            sign: entry.sign > 0 ? '+' : '-',
            delta: delta.toFixed(5),
        }))
        event.preventDefault()
        return
    }

    // 비-tool 축: 기존 EE delta 경로
    if (!robot) return
    const len = robot.tool_inner ? 7 : 6
    if (idx >= len) return
    const eeDelta = Array(len).fill(0)
    eeDelta[idx] = stepSize * (Number(entry.scale) || 1) * (Number(entry.sign) || 1)
    sendDelta(robot, eeDelta)
    pushLog(t('teleopLogKeyDelta', {
        side,
        name: robot.name,
        key: displayKey(normKey),
        axis: entry.axis,
        sign: entry.sign > 0 ? '+' : '-',
        delta: eeDelta[idx].toFixed(5),
    }))
    event.preventDefault()
}

function sendDelta(robot, eeDelta) {
    if (!robot.handler || !robot.handler.moveRobotEEDelta) {
        pushLog(t('teleopLogHandlerMissing', { name: robot.name }), 'error')
        return
    }
    try {
        robot.handler.moveRobotEEDelta({ ee: eeDelta })
    } catch (e) {
        pushLog(t('teleopLogSendError', { name: robot.name, error: e.message }), 'error')
    }
}

function startKeyboardTeleop() {
    if (keyboardActive.value) return
    if (activeArms.value.length === 0) {
        pushLog(t('teleopLogNoArms'), 'error')
        return
    }
    keyboardActive.value = true
    pushLog(t('teleopLogKeyboardStarted', { arms: activeArms.value.map((r) => r.name).join(' + ') }))
    window.addEventListener('keydown', keyboardHandler)
}

function stopKeyboardTeleop() {
    if (!keyboardActive.value) return
    keyboardActive.value = false
    window.removeEventListener('keydown', keyboardHandler)
    pushLog(t('teleopLogKeyboardStopped'))
}

function startLeaderTeleop() {
    pushLog(t('teleopLogStartingLeader'))
    startLeaderTele(props.assembly.id, 'leader_teleoperation')
}

// ───── Unified active state / start-stop ─────
const teleopActive = computed(() => {
    if (teleopMethod.value === 'keyboard') return keyboardActive.value
    if (teleopMethod.value === 'leader') return leaderTeleStarted.value
    return false
})

function startTeleop() {
    if (!allRobotsOn.value) return
    if (teleopMethod.value === 'keyboard') startKeyboardTeleop()
    else if (teleopMethod.value === 'leader') startLeaderTeleop()
}

function stopTeleop() {
    if (teleopMethod.value === 'keyboard') stopKeyboardTeleop()
    else if (teleopMethod.value === 'leader') stopLeaderTele()
}

onUnmounted(() => {
    stopKeyboardTeleop()
})
</script>

<style scoped>
.teleop-console {
    padding-bottom: 16px;
}
.robot-buttons-row {
    gap: 8px;
}
</style>
