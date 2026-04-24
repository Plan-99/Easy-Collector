<template>
    <div class="row q-mb-md items-center q-gutter-sm">
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
                        <q-item-section>삭제</q-item-section>
                    </q-item>
                </q-list>
            </q-menu>
        </q-btn>

        <!-- Add Pose Button -->
        <q-btn outline color="white" @click="openAddPoseDialog">
            ADD POSE
        </q-btn>

        <q-space></q-space>

        <q-input
            dense outlined dark bg-color="dark"
            v-model.number="robotStepSize"
            type="number"
            label="Step Size"
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
                        {{ props.robot.joint_pos[i] ? props.robot.joint_pos[i].toFixed(4) : '0' }}
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
                <div v-for="(p_label, j) in ['x', 'y', 'z', 'roll', 'pitch', 'yaw', ...(props.robot.tool_inner ? ['tool'] : [])]" :key="j" class="q-gutter-x-md text-white q-mb-sm">
                    <div class="border-white q-px-md q-py-xs text-center row flex flex-center">
                        <q-btn
                            dense flat icon="remove" size="xs" class="text-white col-1"
                            @click="moveOneEE(i, j, -robotStepSize)"
                        ></q-btn>
                        <div class="col text-center text-caption">{{ p_label }}</div>
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
                    <div class="text-h6">포즈 추가</div>
                    <q-space></q-space>
                    <q-btn flat dense icon="sync" color="pink-3" size="sm" @click="scanCurrentPose">
                        <q-tooltip>현재 로봇 포즈 읽기</q-tooltip>
                    </q-btn>
                </div>
            </q-card-section>
            <q-card-section>
                <q-input v-model="newPoseName" dense outlined dark bg-color="dark" label="포즈 이름" class="q-mb-md" />
                <div class="row q-col-gutter-sm">
                    <div class="col" v-for="(joint, i) in props.robot.joint_names" :key="joint">
                        <div class="text-caption">{{ joint }}</div>
                        <q-input v-model.number="newPoseValues[i]" type="number" dense dark bg-color="dark" outlined />
                    </div>
                </div>
            </q-card-section>
            <q-card-actions align="center">
                <q-btn flat label="저장" color="primary" @click="addPose" />
                <q-btn flat label="취소" color="grey-7" v-close-popup />
            </q-card-actions>
        </q-card>
    </q-dialog>
</template>
<script setup>
import { ref, onMounted } from 'vue'
import { api } from 'src/boot/axios'
import { Notify } from 'quasar'

const props = defineProps({
    robot: {
        type: Object,
        required: true
    }
})

function moveOneJoint(index, delta) {
    if (props.robot.role === 'tool') {
        delta = 0.12 * (delta > 0 ? 1 : -1)
    }
    const deltaAction = new Array(props.robot.joint_names.length).fill(0)
    deltaAction[index] = delta
    props.robot.handler.moveRobotJointDelta(deltaAction)
}

function moveOneEE(tool_index, p_index, delta) {
    if (p_index < 3) {
        delta = delta * 1
    } else if (p_index < 6) {
        delta = delta * 4
    } else {
        delta = 0.12 * (delta > 0 ? 1 : -1)
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
        Notify.create({ color: 'positive', message: 'Pose saved.' })
        showAddPoseDialog.value = false
        loadPoses()
    })
}

function goToPose(pose) {
    if (pose.pose && pose.pose.length === props.robot.joint_names.length) {
        props.robot.handler.moveRobotJoint(pose.pose)
    }
}

function deletePose(pose) {
    api.delete(`/robot/pose/${pose.id}`).then(() => {
        loadPoses()
    })
}

onMounted(() => {
    loadPoses()
})

</script>
