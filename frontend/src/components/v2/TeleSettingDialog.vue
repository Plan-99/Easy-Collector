<template>
    <q-dialog persistent>
        <q-card style="min-width: 80%" dark class="bg-dark border-rounded border-white">
            <q-card-section class="q-pt-none">
                <q-card-section class="row">
                    <div class="text-h6 text-center">{{ $t('teleopSettingTitle') }}</div>
                    <q-space></q-space>
                    <q-btn size="md" icon="close" flat round @click="$emit('hide')" v-close-popup></q-btn>
                </q-card-section>
                <q-separator color="white"></q-separator>
                <q-tabs
                    v-model="teleSettingTab"
                    class="text-grey"
                    active-color="primary"
                    indicator-color="primary"
                    align="justify"
                    narrow-indicator
                >
                    <q-tab name="general" :label="$t('teleopTabGeneral')" />
                    <q-tab name="keyboard" :label="$t('teleopTabKeyboard')" />
                    <q-tab name="leader" :label="$t('teleopTabLeader')" />
                </q-tabs>
                <q-tab-panels v-model="teleSettingTab" animated>
                    <q-tab-panel name="general" class="bg-secondary">
                        <div class="text-body text-grey-4 q-mb-md">{{ $t('teleopGeneralDescription') }}</div>
                        <div v-for="robot in generalRobots" :key="robot.id" class="border-white border-rounded q-pa-md q-mb-md">
                            <div class="row items-center q-mb-sm">
                                <div class="text-bold text-primary">{{ robot.name }}</div>
                                <div class="text-caption text-grey-5 q-ml-md">{{ robot.type }}</div>
                                <q-space></q-space>
                                <q-btn flat dense size="sm" color="grey-5" :label="$t('teleopGeneralResetToDefault')" @click="resetEeOffset(robot)" />
                            </div>
                            <div v-for="(eeDef, idx) in (robot.default_ee_definitions || [])" :key="idx" class="row q-col-gutter-sm q-mb-sm">
                                <div class="col-2 q-pt-sm text-caption">
                                    <span class="text-primary">{{ eeDef.name }}</span>
                                    <div class="text-grey-5">parent: {{ eeDef.parent }}</div>
                                </div>
                                <div class="col-3" v-for="axis in ['x', 'y', 'z']" :key="axis">
                                    <q-input
                                        v-model.number="generalForm[robot.id][eeDef.name][axis]"
                                        :label="axis + ' (m)'"
                                        type="number"
                                        step="0.001"
                                        dense
                                        outlined
                                        dark
                                        @update:model-value="saveGeneralSetting"
                                    />
                                </div>
                            </div>
                            <div v-if="!(robot.default_ee_definitions && robot.default_ee_definitions.length)" class="text-caption text-grey-6">
                                {{ $t('teleopGeneralNoIk') }}
                            </div>
                        </div>
                    </q-tab-panel>

                    <q-tab-panel name="keyboard" class="bg-secondary">
                        <div class="text-body text-grey-4 q-mb-md">{{ $t('teleopKeyboardDescription') }}</div>
                        <div class="row q-col-gutter-sm q-mb-md">
                            <div class="col-3">
                                <q-input
                                    v-model.number="keyboardForm.step_size"
                                    :label="$t('teleopKeyboardStepSize')"
                                    type="number"
                                    step="0.0001"
                                    dense
                                    outlined
                                    dark
                                    @update:model-value="saveKeyboardSetting"
                                />
                            </div>
                            <div class="col-9 q-pt-sm text-caption text-grey-5">{{ $t('teleopKeyboardDeltaHint') }}</div>
                        </div>
                        <div v-if="isDualArm" class="text-bold text-primary q-mb-xs">{{ $t('teleopKeyboardLeftArm') }}</div>
                        <q-markup-table dark dense flat class="teleop-keymap-table q-mb-md">
                            <thead>
                                <tr>
                                    <th class="text-left">{{ $t('teleopKeyboardKey') }}</th>
                                    <th class="text-left">{{ $t('teleopKeyboardAxis') }}</th>
                                    <th class="text-left">{{ $t('teleopKeyboardSign') }}</th>
                                    <th class="text-left">{{ $t('teleopKeyboardScale') }}</th>
                                </tr>
                            </thead>
                            <tbody>
                                <tr v-for="row in leftAxisRows" :key="`L-${row.entry.axis}-${row.entry.sign}`">
                                    <td class="text-bold">{{ displayKey(row.key) }}</td>
                                    <td class="text-primary">{{ row.entry.axis }}</td>
                                    <td class="text-primary">{{ row.entry.sign > 0 ? '+' : '-' }}</td>
                                    <td>
                                        <q-input
                                            v-model.number="row.entry.scale"
                                            type="number"
                                            step="1"
                                            dense outlined dark borderless
                                            input-class="text-right"
                                            style="max-width: 70px;"
                                            @update:model-value="saveKeyboardSetting"
                                        />
                                    </td>
                                </tr>
                            </tbody>
                        </q-markup-table>

                        <template v-if="isDualArm">
                            <div class="text-bold text-primary q-mb-xs">{{ $t('teleopKeyboardRightArm') }}</div>
                            <q-markup-table dark dense flat class="teleop-keymap-table">
                                <thead>
                                    <tr>
                                        <th class="text-left">Key</th>
                                        <th class="text-left">Axis</th>
                                        <th class="text-left">Sign</th>
                                        <th class="text-left">Scale</th>
                                    </tr>
                                </thead>
                                <tbody>
                                    <tr v-for="row in rightAxisRows" :key="`R-${row.entry.axis}-${row.entry.sign}`">
                                        <td class="text-bold">{{ displayKey(row.key) }}</td>
                                        <td class="text-primary">{{ row.entry.axis }}</td>
                                        <td class="text-primary">{{ row.entry.sign > 0 ? '+' : '-' }}</td>
                                        <td>
                                            <q-input
                                                v-model.number="row.entry.scale"
                                                type="number"
                                                step="1"
                                                dense outlined dark borderless
                                                input-class="text-right"
                                                style="max-width: 70px;"
                                                @update:model-value="saveKeyboardSetting"
                                            />
                                        </td>
                                    </tr>
                                </tbody>
                            </q-markup-table>
                        </template>
                        <div class="row q-mt-md justify-end">
                            <q-btn flat color="grey-5" :label="$t('teleopResetToDefaults')" @click="resetKeyboardSetting" />
                        </div>
                    </q-tab-panel>

                    <q-tab-panel name="leader" class="row q-col-gutter-x-md bg-secondary" style="min-height: 600px;">
                        <div class="col-5 column">
                            <div class="row q-col-gutter-sm q-mb-sm">
                                <div class="col" v-for="robot in generalRobots" :key="robot.id">
                                    <q-btn
                                        class="full-width"
                                        outline
                                        no-caps
                                        icon="power_settings_new"
                                        :color="robot.status === 'on' ? 'positive' : 'grey-6'"
                                        :label="robot.name"
                                        :loading="robot.status === 'loading'"
                                        @click="toggleRobot(robot)"
                                    />
                                </div>
                            </div>
                            <q-btn class="full-width" outline color="primary" @click="startLeaderRobot" v-if="!leaderRobotStarted">{{ $t('teleopStartLeaderRobot') }}</q-btn>
                            <q-btn class="full-width" outline color="orange-8" @click="stopLeaderRobot" v-else>{{ $t('teleopStopLeaderRobot') }}</q-btn>
                            <process-console
                                process="start_leader_robot"
                                class="col q-mt-md border-white"
                            />
                        </div>
                        <div class="col">
                            <div
                                v-if="!allRobotsOn"
                                class="border-white border-rounded q-pa-lg text-center text-grey-5 column items-center justify-center"
                                style="height: 100%; min-height: 400px;"
                            >
                                <q-icon name="power_off" size="xl" class="q-mb-md" />
                                <div class="text-body1 q-mb-sm">{{ $t('teleopRobotOff') }}</div>
                                <div class="text-caption">{{ $t('teleopRobotOffHint') }}</div>
                            </div>
                            <q-stepper
                                v-else
                                v-model="leaderSettingStep"
                                header-nav
                                ref="stepper"
                                color="primary"
                                animated
                                vertical
                                dark
                                class="border-white border-rounded"
                            >
                                <q-step
                                    :name="1"
                                    title="Origin Setting"
                                    :done="leaderSettingStep > 1"
                                    :header-nav="leaderSettingStep > 1"
                                >
                                    <div>
                                        The origin of the leader robot's motor must be calibrated. 
                                        Manually adjust the leader robot so its pose matches the worker robot's pose, then press the save button to store the position.
                                    </div>
                                    <div>
                                        <!-- <q-btn
                                            color="green"
                                            label="Go to Origin Position"
                                            icon="home"
                                            @click="robot.handler.goOriginPos"
                                            class="full-width q-mt-md"
                                            outline
                                        /> -->
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md" v-if="leaderSettingForm.joint_map.length">
                                        <div class="col-3" v-for="(j, i) in leaderSettingForm.joint_map" :key="i">
                                            <div class="border-white border-rounded drop-target overflow-hidden q-px-sm column" style="min-height: 100px;"
                                                @dragenter="onDragEnter"
                                                @dragleave="onDragLeave"
                                                @dragover="onDragOver"
                                                @drop="() => onDrop(i)"
                                            >
                                                <div class="text-center q-pa-sm col text-caption" style="max-width: 100%;">
                                                    <span class="text-primary">{{ j.robot_name }}</span> {{ j.joint_name }}
                                                </div>
                                                <q-input
                                                    v-model="j.origin"
                                                    :label="`ID ${j.dxl_id}`"
                                                    type="number"
                                                    dense
                                                    outlined
                                                    dark
                                                    class="bg-dark"
                                                    v-if="j.dxl_id !== null && j.port !== null"
                                                >
                                                    <template v-slot:append>
                                                        <q-btn
                                                            flat
                                                            color="primary"
                                                            :outline="false"
                                                            size="sm"
                                                            dense
                                                            @click="j.port = '/dev/ttyUSB' + (Number(j.port[j.port.length-1]) + 1) % 5"
                                                        >ttyUSB{{ j.port[j.port.length-1] }}</q-btn>
                                                        <q-badge color="grey-7" floating class="cursor-pointer" @click="removeDxlFromJointMap(i)">x</q-badge>
                                                    </template>
                                                </q-input>
                                                <q-checkbox size="xs" dark v-model="j.is_gripper" val="xs" label="Tool Joint" />
                                            </div>
                                        </div>
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md" v-if="dxlArray.length">
                                        <div class="col-3" v-for="(j, i) in dxlArray" :key="i">
                                            <q-input
                                                v-model="j.origin"
                                                :label="`ID ${j.dxl_id} (${j.port})`"
                                                type="number"
                                                dense
                                                outlined
                                                dark
                                                class="bg-dark"
                                                draggable="true"
                                                @dragstart="(e) => onDragStart(e, i)"
                                                :id="`dxl-${j.port}-${j.dxl_id}`"
                                                style=" cursor: move;"
                                            >
                                            </q-input>
                                            <q-btn size="xs" dense label="Set as Gripper" flat icon="add" color="primary" @click="dxlSetAsGripper(j)" />
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="() => { leaderSettingStep = 2 }" color="primary" outline label="Save & Go Next" :disable="leaderSettingForm.joint_map.filter((e) => !e.dxs_id && !e.port).length > 0" />
                                    </q-stepper-navigation>
                                </q-step>

                                <q-step
                                    :name="2"
                                    title="Gripper Setting"
                                    :done="leaderSettingStep > 2"
                                    :header-nav="leaderSettingStep > 2"
                                >
                                    Open and close your leader robot's gripper and press the button below to save the gripper position.
                                    <div class="row q-col-gutter-sm q-mt-md" v-for="(joint, id) in leaderSettingForm.joint_map.filter((e) => e.is_gripper)" :key="id">
                                        <div class="col-6" v-for="i in [0, 1]" :key="i">
                                            <q-input
                                                v-model.number="joint.gripper_dxl_range[i]"
                                                :label="`${i === 0 ? 'Open' : 'Close'} Position`"
                                                type="number"
                                                dense
                                                outlined
                                                dark
                                                v-if="i === 0 || joint.gripper_dxl_range_saved[0]"
                                            >
                                                <template v-slot:append>
                                                    <q-btn
                                                        color="primary"
                                                        :outline="!joint.gripper_dxl_range_saved[i]"
                                                        size="sm"
                                                        @click="() => { joint.gripper_dxl_range_saved[i] = !joint.gripper_dxl_range_saved[i]; }"
                                                    >Save</q-btn>
                                                </template>
                                            </q-input>
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="() => { leaderSettingStep = 3; saveLeaderSetting() }" color="primary" outline label="Go Next" :disable="!leaderSettingForm.joint_map.filter((e) => e.is_gripper && e.gripper_dxl_range_saved[0] && e.gripper_dxl_range_saved[1]).length > 0" />
                                        <q-btn flat @click="leaderSettingStep = 1" color="primary" label="Back" class="q-ml-sm" />
                                    </q-stepper-navigation>
                                </q-step>

                                <q-step
                                    :name="3"
                                    title="Joint Direction Setting"
                                    :header-nav="leaderSettingStep > 3"
                                    :done="leaderSettingStep > 3"
                                >
                                    You can set the joint direction of the leader robot's motors here.
                                    Please start teleoperation and map the direction of leader robot's motors to the worker robot's motors.
                                    <div>
                                        <q-btn
                                            v-if="!leaderTeleStarted"
                                            color="positive"
                                            label="Start Teleoperation"
                                            @click="startLeaderTele(assembly.id, 'start_leader_robot')"
                                            class="full-width q-mt-md"
                                            outline
                                        />
                                        <q-btn
                                            v-else
                                            color="orange-8"
                                            label="Stop Teleoperation"
                                            @click="stopLeaderTele()"
                                            class="full-width q-mt-md"
                                            outline
                                        />
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md">
                                        <div class="col-4" v-for="(joint, i) in leaderSettingForm.joint_map" :key="i">
                                            <q-toggle
                                                v-model="joint.sign"
                                                :label="`${joint.robot_name} ${joint.joint_name}`"
                                                outlined
                                                :true-value="-1"
                                                :false-value="1"
                                                @update:model-value="saveLeaderSetting"
                                            />
                                        </div>
                                        <div class="col-12 q-mt-md border-white border-rounded q-pa-md">
                                            <div>
                                                Check the joint of the robot which moves in the opposite direction of the leader's joint.
                                            </div>
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn outline @click="() => { leaderSettingStep = 4 }" color="primary" label="Go Next" />
                                        <q-btn flat @click="leaderSettingStep = 2" color="primary" label="Back" class="q-ml-sm" />
                                    </q-stepper-navigation>
                                </q-step>
                                <q-step
                                    :name="4"
                                    title="Motion Filter Settings"
                                    :header-nav="leaderSettingStep > 4"
                                >
                                    Tune teleoperation responsiveness. Higher EMA = smoother but laggier.
                                    Larger Max step / Publish rate = faster response but rougher motion.
                                    <div>
                                        <q-btn
                                            v-if="!leaderTeleStarted"
                                            color="primary"
                                            label="Start Teleoperation"
                                            @click="startLeaderTele(assembly.id, 'start_leader_robot')"
                                            class="full-width q-mt-md"
                                            outline
                                        />
                                        <q-btn
                                            v-else
                                            color="orange-8"
                                            label="Stop Teleoperation"
                                            @click="stopLeaderTele()"
                                            class="full-width q-mt-md"
                                            outline
                                        />
                                    </div>
                                    <div class="row q-col-gutter-md q-mt-md">
                                        <div class="col-4">
                                            <q-input
                                                v-model.number="leaderSettingForm.ema"
                                                @update:model-value="saveLeaderSetting"
                                                label="EMA filter (0~1)"
                                                type="number"
                                                step="0.05"
                                                dense
                                                outlined
                                                dark
                                            />
                                        </div>
                                        <div class="col-4">
                                            <q-input
                                                v-model.number="leaderSettingForm.max_step_rad"
                                                @update:model-value="saveLeaderSetting"
                                                label="Max step (rad/tick)"
                                                type="number"
                                                step="0.001"
                                                dense
                                                outlined
                                                dark
                                            />
                                        </div>
                                        <div class="col-4">
                                            <q-input
                                                v-model.number="leaderSettingForm.publish_rate"
                                                @update:model-value="saveLeaderSetting"
                                                label="Publish rate (Hz)"
                                                type="number"
                                                step="10"
                                                dense
                                                outlined
                                                dark
                                            />
                                        </div>
                                    </div>
                                    <q-stepper-navigation align="right">
                                        <q-btn @click="$emit('hide', leaderSettingForm)" color="primary" outline label="Finish" />
                                    </q-stepper-navigation>
                                </q-step>
                                

                            </q-stepper>
                        </div>
                    </q-tab-panel>

                    <q-tab-panel name="keyboard">
                        <div class="text-h6">Keyboard Teleoperation will be supported soon.</div>
                    </q-tab-panel>
                </q-tab-panels>
            </q-card-section>
        </q-card>
    </q-dialog>
</template>

<script setup>
import { ref, reactive, onMounted, onUnmounted, defineProps, computed } from 'vue';
import { useSocket } from 'src/composables/useSocket';
import ProcessConsole from './ProcessConsole.vue';
import { useROS } from 'src/composables/useROS';
import { useLeaderTeleoperation } from 'src/composables/useLeaderTeleoperation';
import { api } from 'src/boot/axios';
import { useProcessStore } from 'src/stores/processStore';
import { DEFAULT_KEYBOARD_SETTINGS, TELEOP_AXES, displayKey } from 'src/configs/teleopDefaults';

const processStore = useProcessStore();

// import { Notify } from 'quasar';

const props = defineProps({
    assembly: {
        type: Object,
        default: () => ({})
    },
})

const { socket } = useSocket();
const { connectROS } = useROS(); 
const { leaderTeleStarted, startLeaderTele, stopLeaderTele } = useLeaderTeleoperation();


const teleSettingTab = ref('general');
const leaderSettingStep = ref(1);

// ───── General tab: per-robot ee_offset 편집 ─────
const generalRobots = computed(() => {
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

// generalForm[robotId][eeName] = { x, y, z }
const generalForm = reactive({})

function _initGeneralFormForRobot(robot) {
    const defs = robot.default_ee_definitions || []
    const userOffset = (robot.settings && robot.settings.ee_offset) || {}
    if (!generalForm[robot.id]) generalForm[robot.id] = {}
    defs.forEach((d) => {
        const fallback = Array.isArray(d.offset) ? d.offset : [0, 0, 0]
        const saved = userOffset[d.name]
        const src = Array.isArray(saved) ? saved : fallback
        generalForm[robot.id][d.name] = {
            x: Number(src[0] ?? 0),
            y: Number(src[1] ?? 0),
            z: Number(src[2] ?? 0),
        }
    })
}

function saveGeneralSetting() {
    generalRobots.value.forEach((robot) => {
        const eeOffset = {}
        Object.entries(generalForm[robot.id] || {}).forEach(([eeName, v]) => {
            eeOffset[eeName] = [Number(v.x) || 0, Number(v.y) || 0, Number(v.z) || 0]
        })
        api.put(`/robot/${robot.id}`, {
            name: robot.name,
            type: robot.type,
            ee_offset: eeOffset,
        }).catch((err) => console.error('Failed to save ee_offset for robot', robot.id, err))
    })
}

function resetEeOffset(robot) {
    const defs = robot.default_ee_definitions || []
    defs.forEach((d) => {
        const fb = Array.isArray(d.offset) ? d.offset : [0, 0, 0]
        generalForm[robot.id][d.name] = {
            x: Number(fb[0] ?? 0),
            y: Number(fb[1] ?? 0),
            z: Number(fb[2] ?? 0),
        }
    })
    // Save with empty ee_offset to clear DB override
    api.put(`/robot/${robot.id}`, {
        name: robot.name,
        type: robot.type,
        ee_offset: {},
    }).catch((err) => console.error('Failed to reset ee_offset', err))
}

// ───── Keyboard tab: 키 매핑 편집 ─────
const existingKeyboardSetting = props.assembly.teleoperators?.find((e) => e.type === 'keyboard')?.settings
const keyboardForm = reactive(
    existingKeyboardSetting && existingKeyboardSetting.axis_map
        ? JSON.parse(JSON.stringify(existingKeyboardSetting))
        : JSON.parse(JSON.stringify(DEFAULT_KEYBOARD_SETTINGS))
)

// 양팔 여부: left_arm AND right_arm 둘 다 존재해야 dual
const isDualArm = computed(() => {
    const hasLeft = !!(props.assembly.left_arm || props.assembly.left_arm_id)
    const hasRight = !!(props.assembly.right_arm || props.assembly.right_arm_id)
    return hasLeft && hasRight
})

// 표시 순서: x, y, z, ax, ay, az, tool — 같은 axis 내에서는 + 먼저, - 나중
const _axisOrder = Object.fromEntries(TELEOP_AXES.map((a, i) => [a, i]))
function _sortAxisRows(rows) {
    return [...rows].sort((a, b) => {
        const ai = _axisOrder[a.entry.axis] ?? 999
        const bi = _axisOrder[b.entry.axis] ?? 999
        if (ai !== bi) return ai - bi
        return (b.entry.sign || 0) - (a.entry.sign || 0)
    })
}

const leftAxisRows = computed(() => {
    const rows = Object.entries(keyboardForm.axis_map || {})
        .filter(([, entry]) => (entry.side || 'left') === 'left')
        .map(([key, entry]) => ({ key, entry }))
    return _sortAxisRows(rows)
})

const rightAxisRows = computed(() => {
    const rows = Object.entries(keyboardForm.axis_map || {})
        .filter(([, entry]) => entry.side === 'right')
        .map(([key, entry]) => ({ key, entry }))
    return _sortAxisRows(rows)
})

function saveKeyboardSetting() {
    api.post('/teleoperator', {
        assembly_id: props.assembly.id,
        type: 'keyboard',
        settings: keyboardForm,
    }).catch((err) => console.error('Failed to save keyboard setting', err))
}

function resetKeyboardSetting() {
    const fresh = JSON.parse(JSON.stringify(DEFAULT_KEYBOARD_SETTINGS))
    Object.keys(keyboardForm).forEach((k) => delete keyboardForm[k])
    Object.assign(keyboardForm, fresh)
    saveKeyboardSetting()
}

// 초기 generalForm 채우기
generalRobots.value.forEach(_initGeneralFormForRobot)

// ───── Leader tab: 로봇 on/off 상태 ─────
const allRobotsOn = computed(() =>
    generalRobots.value.length > 0 && generalRobots.value.every((r) => r.status === 'on')
)

function toggleRobot(robot) {
    if (!robot.handler) return
    if (robot.status === 'on') {
        if (robot.handler.stopRobot) robot.handler.stopRobot()
    } else if (robot.status !== 'loading') {
        if (robot.handler.startRobot) robot.handler.startRobot()
    }
}


const leaderRobotStarted = computed(() => {
    return processStore.isRunning('subscribe_dxl');
}); 
const _existingLeader = props.assembly.teleoperators.find(e => e.type === 'leader')
const leaderSettingForm = ref(_existingLeader ? _existingLeader.settings : {
    joint_map: [],
    ema: 0.5,
    max_step_rad: 0.005,
    publish_rate: 50,
});
// 기존 preset에 새 필드가 없으면 기본값으로 채움 (마이그레이션 보조)
if (leaderSettingForm.value.max_step_rad === undefined) leaderSettingForm.value.max_step_rad = 0.005
if (leaderSettingForm.value.publish_rate === undefined) leaderSettingForm.value.publish_rate = 50


const duplicatedIds = [];
if (!props.assembly.teleoperators.find(e => e.type === 'leader')) {
    ['left_arm', 'left_tool', 'right_arm', 'right_tool'].forEach((e) => {
        if (!props.assembly[e]) {
            return;
        }
        if (duplicatedIds.includes(props.assembly[e].id)) {
            return;
        }
        duplicatedIds.push(props.assembly[e].id);
        props.assembly[e].joint_names.forEach((joint_name, i) => {
            let isGripper = false;
            if (e.includes('tool')) isGripper = true;
            if (props.assembly[e].tool_inner && i === props.assembly[e].joint_names.length - 1) isGripper = true;
            leaderSettingForm.value.joint_map.push({
                part_name: e,
                robot_id: props.assembly[e].id,
                robot_name: props.assembly[e].name,
                joint_name: joint_name,
                joint_upper_bound: props.assembly[e].joint_upper_bounds[i],
                joint_lower_bound: props.assembly[e].joint_lower_bounds[i],
                port: null,
                dxl_id: null,
                origin: 0,
                is_gripper: isGripper,
                sign: 1,
                gripper_dxl_range: [0, 0],
                gripper_dxl_range_saved: [false, false],
            });
        });
    })
}

function dxlSetAsGripper(dxl) {
    console.log(dxl)
    leaderSettingForm.value.joint_map.push({
        joint_name: 'Gripper (no robot)',
        port: dxl.port,
        dxl_id: dxl.dxl_id,
        origin: dxl.origin,
        is_gripper: true,
        is_dummy_gripper: true,
        sign: 1,
        gripper_dxl_range: [0, 0],
        gripper_dxl_range_saved: [false, false],
    });
    dxlArray.value.splice(dxlArray.value.indexOf(dxl), 1);

}

// const toolEditable = computed(() => {
//     let result = false;
//     ['left_arm', 'left_tool', 'right_arm', 'right_tool'].forEach((e) => {
//         if (!props.assembly[e]) {
//             return false;
//         }
//         if (props.assembly[e].tool_inner) {
//             result = true;
//         }
//     })
//     return result;
// })

const dxlArray = ref([]);


function startLeaderRobot() {
    api.post('/teleoperator:dxl_read')
}

function stopLeaderRobot() {
    if (!leaderRobotStarted.value) {
        return;
    }
    api.post('/teleoperator:stop_dxl_read')
}


function saveLeaderSetting() {
    api.post('/teleoperator', {
        assembly_id: props.assembly.id,
        type: 'leader',
        settings: leaderSettingForm.value,
    })
}

// function goOriginPos() {
//     const robot = props.robot;
//     api.post(`/robot/${robot.id}/:move_to`, {
//         goal_pos: [0, 0, 0, 0, 0, 0] // Default to zero if not set
//     }).then(() => {
//         Notify.create({
//             color: 'positive',
//             message: 'Robot moved to origin position'
//         })
//     }).catch((error) => {
//         console.error('Error moving robot to origin:', error);
//         Notify.create({
//             color: 'negative',
//             message: 'Failed to move robot to origin'
//         })
//     });
// }


const draggingDxl = ref(null);
// store the id of the draggable element
function onDragStart (e, targetDxlIndex) {
    const dxl = dxlArray.value[targetDxlIndex]
    draggingDxl.value = dxl
    e.dataTransfer.dropEffect = 'move'
}

function onDragEnter (e) {
// don't drop on other draggables
    if (e.target.draggable !== true) {
        e.target.classList.add('drag-enter')
    }
}

function onDragLeave (e) {
    e.target.classList.remove('drag-enter')
}


function onDragOver (e) {
    e.preventDefault()
}

function onDrop (targetJointIndex) {
    if (!draggingDxl.value) {
        return;
    }
    dxlArray.value.splice(dxlArray.value.indexOf(draggingDxl.value), 1);

    leaderSettingForm.value.joint_map[targetJointIndex].port = draggingDxl.value.port;
    leaderSettingForm.value.joint_map[targetJointIndex].dxl_id = draggingDxl.value.dxl_id;
    leaderSettingForm.value.joint_map[targetJointIndex].origin = draggingDxl.value.origin;
    draggingDxl.value = null;
}

function removeDxlFromJointMap(jointIndex) {
    const joint = leaderSettingForm.value.joint_map[jointIndex];
    if (joint.dxl_id === null || joint.port === null) {
        return;
    }
    if (joint.is_dummy_gripper) {
        leaderSettingForm.value.joint_map.splice(jointIndex, 1);
        return;
    }
    dxlArray.value.push({
        port: joint.port,
        dxl_id: joint.dxl_id,
        origin: joint.origin,
    });
    joint.port = null;
    joint.dxl_id = null;
    joint.origin = 0;
}

onMounted(() => {
    connectROS();

    socket.on('dynamixel_data', (data) => {
        const port = data.port;
        data.values.forEach((value) => {
            const dxl_id = value.id;
            const position = value.position;
            if (leaderSettingStep.value === 1) {
                const dxlFound = dxlArray.value.find(e => e.port === port && e.dxl_id === dxl_id);
                const dxlInJointMap = leaderSettingForm.value.joint_map.find(e => e.port === port && e.dxl_id === dxl_id);
                if (dxlFound) {
                    dxlFound.origin = position;
                } else if (dxlInJointMap) {
                    dxlInJointMap.origin = position;
                } else {
                    dxlArray.value.push({
                        port: port,
                        dxl_id: dxl_id,
                        origin: position,
                    });
                }
                return;
            }

            if (leaderSettingStep.value === 2) {
                const dxlInJointMap = leaderSettingForm.value.joint_map.find(e => e.port === port && e.dxl_id === dxl_id);
                if (!dxlInJointMap.is_gripper) {
                    return;
                }

                const currentValue = Number(position);


                // i번째 그리퍼의 최소값이 아직 저장되지 않았다면 현재 값으로 업데이트
                if (!dxlInJointMap.gripper_dxl_range_saved[0]) {
                    dxlInJointMap.gripper_dxl_range[0] = currentValue;
                }
                // i번째 그리퍼의 최대값이 아직 저장되지 않았다면 현재 값으로 업데이트
                else if (!dxlInJointMap.gripper_dxl_range_saved[1]) {
                    dxlInJointMap.gripper_dxl_range[1] = currentValue;
                }
            }

        });
    })

    socket.on('stop_process', (data) => {
        if (data.id === 'start_leader_robot') {
            leaderRobotStarted.value = false;
        }
        if (data.id === 'leader_teleoperation') {
            leaderTeleStarted.value = false;
        }
    });
});

onUnmounted(() => {
    stopLeaderRobot();
    stopLeaderTele();
    socket.off('leader_robot_started');
    socket.off('leader_robot_stopped');
    socket.off('dynamixel_data');
});

</script>

<style scoped>
.teleop-keymap-table :deep(td),
.teleop-keymap-table :deep(th) {
    padding: 2px 8px !important;
    height: 28px !important;
}
.teleop-keymap-table :deep(.q-field__control) {
    min-height: 24px;
    height: 24px;
}
.teleop-keymap-table :deep(.q-field__native) {
    min-height: 24px;
    padding: 0;
}
.teleop-keymap-table :deep(.q-field__inner) {
    min-height: 24px;
}
</style>