<template>
    <div class="row q-mb-md items-center q-gutter-sm">
        <!-- 이동 중에는 모든 Pose 버튼이 단일 정지 버튼으로 토글된다.
             cancelMove() 가 즉시 백엔드 cancel_move_to + 로컬 isMoving false. -->
        <template v-if="!isMoving">
            <!-- Saved Pose Buttons -->
            <q-btn
                v-for="pose in poses" :key="pose.id"
                outline
                :label="pose.name"
                @click="goToPose(pose)"
                :style="pose.color ? { borderColor: pose.color, color: pose.color } : {}"
            >
                <q-menu context-menu>
                    <q-list dense dark>
                        <q-item clickable v-close-popup @click="deletePose(pose)" class="text-negative">
                            <q-item-section avatar><q-icon name="delete" size="xs" /></q-item-section>
                            <q-item-section>{{ $t('robotPoseDelete') }}</q-item-section>
                        </q-item>
                    </q-list>
                </q-menu>
            </q-btn>

            <!-- Add Pose Button -->
            <q-btn outline color="white" @click="openAddPoseDialog">
                {{ $t('robotPoseAddBtn') }}
            </q-btn>
        </template>
        <template v-else>
            <q-btn
                unelevated color="negative" icon="stop"
                :label="$t('robotPoseStop')"
                @click="cancelMove"
            />
            <div class="text-caption text-grey-5 q-ml-sm">
                {{ $t('robotPoseMovingTo', { name: movingPoseName || '' }) }}
            </div>
        </template>

        <q-space></q-space>

        <q-input
            dense outlined dark bg-color="dark"
            v-model.number="robotStepSize"
            type="number"
            :label="$t('robotPoseStepSize')"
            class="col-2"
        ></q-input>
    </div>
    <div class="row q-gutter-x-md">
        <div class="col">
            <div v-for="(j, i) in props.robot.joint_names" :key="i" class="q-gutter-x-md text-white q-mb-sm">
                <div class="border-white q-px-md q-py-xs text-center row flex flex-center">
                    <q-btn
                        dense
                        flat
                        icon="remove"
                        size="xs"
                        class="text-white col-1"
                        @click="moveOneJoint(i, -robotStepSize)"
                    ></q-btn>
                    <div class="col text-center text-caption"> {{ j }} </div>
                    <div class="col text-center text-caption text-primary">
                        {{ jointReading(i) }}
                    </div>
                    <q-btn
                        dense
                        flat
                        icon="add"
                        size="xs"
                        class="text-white col-1"
                        @click="moveOneJoint(i, robotStepSize)"
                    ></q-btn>
                </div>
            </div>
        </div>
        <div class="col" v-if="props.robot.ik_available">
            <div v-for="(ee_name, i) in Object.keys(props.robot.eePos || {})" :key="i">
                <div v-for="(p_label, j) in ['x', 'y', 'z', 'ax', 'ay', 'az', ...(props.robot.tool_inner ? ['tool'] : [])]" :key="j" class="q-gutter-x-md text-white q-mb-sm">
                    <div class="border-white q-px-md q-py-xs text-center row flex flex-center">
                        <q-btn
                            dense flat icon="remove" size="xs" class="text-white col-1"
                            @click="moveOneEE(i, j, -robotStepSize)"
                        ></q-btn>
                        <div class="col text-center text-caption">{{ eeAxisLabel(ee_name, p_label) }}</div>
                        <div class="col text-center text-caption text-primary">
                            {{ props.robot.eePos && props.robot.eePos[ee_name] ? props.robot.eePos[ee_name][j]?.toFixed(4) : '0' }}
                        </div>
                        <q-btn
                            dense flat icon="add" size="xs" class="text-white col-1"
                            @click="moveOneEE(i, j, +robotStepSize)"
                        ></q-btn>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Add Pose Dialog -->
    <q-dialog v-model="showAddPoseDialog" persistent>
        <q-card style="min-width: 500px;" dark>
            <q-card-section>
                <div class="row items-center">
                    <div class="text-h6">{{ $t('robotPoseAddTitle') }}</div>
                    <q-space></q-space>
                    <q-btn flat dense icon="sync" color="pink-3" size="sm" @click="scanCurrentPose">
                        <q-tooltip>{{ $t('robotPoseScanTooltip') }}</q-tooltip>
                    </q-btn>
                </div>
            </q-card-section>
            <q-card-section>
                <q-input v-model="newPoseName" dense outlined dark bg-color="dark" :label="$t('robotPoseAddName')" class="q-mb-md" />
                <div class="row q-col-gutter-sm">
                    <div class="col" v-for="(joint, i) in props.robot.joint_names" :key="joint">
                        <div class="text-caption">{{ joint }}</div>
                        <q-input v-model.number="newPoseValues[i]" type="number" dense dark bg-color="dark" outlined />
                    </div>
                </div>
            </q-card-section>
            <q-card-actions align="center">
                <q-btn flat :label="$t('robotPoseAddSave')" color="primary" @click="addPose" />
                <q-btn flat :label="$t('robotPoseAddCancel')" color="grey-7" v-close-popup />
            </q-card-actions>
        </q-card>
    </q-dialog>
</template>
<script setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { api } from 'src/boot/axios'
import { Notify } from 'quasar'
import { useI18n } from 'vue-i18n'

const { t } = useI18n()

const props = defineProps({
    robot: {
        type: Object,
        required: true
    }
})

// dual_arm 일 때는 축 라벨에 팔 구분을 붙인다 (예: left_x, right_ax).
// L_ee → left_, R_ee → right_. 그 외(single_arm 'ee')는 라벨 그대로.
function eeAxisLabel(ee_name, p_label) {
    if (props.robot.role === 'dual_arm') {
        const side = ee_name === 'L_ee' ? 'left_' : ee_name === 'R_ee' ? 'right_' : ''
        return side + p_label
    }
    return p_label
}

function jointReading(i) {
    const src = props.robot.joint_pos || props.robot.jointState
    const v = src && src[i]
    return typeof v === 'number' ? v.toFixed(4) : '0'
}

function moveOneJoint(index, delta) {
    const deltaAction = new Array(props.robot.joint_names.length).fill(0)
    deltaAction[index] = delta
    props.robot.handler.moveRobotJointDelta(deltaAction)
}

function moveOneEE(tool_index, p_index, delta) {
    if (p_index < 3) {
        delta = delta * 1
    } else if (p_index < 6) {
        delta = delta * 4
    }
    const deltaPos = {}
    const toolName = props.robot.role === 'dual_arm' ? (tool_index === 0 ? 'L_ee' : 'R_ee') : 'ee'
    deltaPos[toolName] = Array(props.robot.tool_inner ? 7 : 6).fill(0)
    deltaPos[toolName][p_index] = delta
    props.robot.handler.moveRobotEEDelta(deltaPos)
}

const robotStepSize = ref(0.003)

// --- Poses ---
const poses = ref([])
const showAddPoseDialog = ref(false)
const newPoseName = ref('')
const newPoseValues = ref([])

function loadPoses() {
    if (!props.robot || !props.robot.id) return
    api.get(`/robot/${props.robot.id}/poses`).then(res => {
        poses.value = res.data.poses || []
    })
}

function openAddPoseDialog() {
    newPoseName.value = 'pose_' + (poses.value.length + 1)
    newPoseValues.value = Array(props.robot.joint_names.length).fill(0)
    showAddPoseDialog.value = true
}

function scanCurrentPose() {
    if (props.robot.jointState) {
        newPoseValues.value = [...props.robot.jointState]
    }
}

function addPose() {
    api.post(`/robot/${props.robot.id}/pose`, {
        name: newPoseName.value,
        pose: [...newPoseValues.value],
        is_default: poses.value.length === 0,
    }).then(() => {
        Notify.create({ color: 'positive', message: t('robotPoseSavedNotify') })
        showAddPoseDialog.value = false
        loadPoses()
    })
}

// 이동 중 토글 — 백엔드 is_moving 이 push 되지 않으므로 로컬에서 duration 만큼
// 타이머로 자동 해제. cancelMove 호출 시 즉시 reset.
const POSE_MOVE_DURATION = 5.0
const isMoving = ref(false)
const movingPoseName = ref(null)
let moveTimer = null

function clearMoveTimer() {
    if (moveTimer) {
        clearTimeout(moveTimer)
        moveTimer = null
    }
}

function goToPose(pose) {
    if (!pose.pose || pose.pose.length !== props.robot.joint_names.length) return
    // moveRobotJoint(socketio 단발) 가 아니라 move_to API 로 보내야 보간이 적용된다.
    // 단발 publish 는 interp_node 에서 default cmd_interval 만에 jump 해서
    // duration 이 무시되고 한 번에 가는 것처럼 보임.
    clearMoveTimer()
    isMoving.value = true
    movingPoseName.value = pose.name
    props.robot.handler.moveToPose(pose.pose, { duration: POSE_MOVE_DURATION })
    moveTimer = setTimeout(() => {
        isMoving.value = false
        movingPoseName.value = null
        moveTimer = null
    }, POSE_MOVE_DURATION * 1000)
}

function cancelMove() {
    clearMoveTimer()
    isMoving.value = false
    movingPoseName.value = null
    props.robot.handler.stopMove()
}

function deletePose(pose) {
    api.delete(`/robot/pose/${pose.id}`).then(() => {
        loadPoses()
    })
}

onMounted(() => {
    loadPoses()
})

onBeforeUnmount(() => {
    clearMoveTimer()
})

</script>
