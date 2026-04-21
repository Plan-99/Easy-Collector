<template>
    <q-dialog persistent>
        <q-card style="min-width: 80%" dark class="bg-dark border-rounded border-white">
            <q-card-section class="q-pt-none">
                <q-card-section class="row">
                    <div class="text-h6 text-center">Teleoperation Setting</div>
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
                    <q-tab name="leader" label="Leader Robot" />
                    <q-tab name="keyboard" label="Keyboard" />
                </q-tabs>
                <q-tab-panels v-model="teleSettingTab" animated>
                    <q-tab-panel name="leader" class="row q-col-gutter-x-md bg-secondary">
                        <div class="col-5">
                            <div class="row q-col-gutter-x-md">
                                <div  class="col">
                                    <q-btn class="full-width full-height" outline color="primary" @click="startLeaderRobot" v-if="!leaderRobotStarted">Start Leader Robot</q-btn>
                                    <q-btn class="full-width full-height" outline color="orange-8" @click="stopLeaderRobot" v-else>Stop Leader Robot</q-btn>
                                </div>
                            </div>
                            <process-console 
                                process="start_leader_robot"
                                class="q-mt-md border-white"
                                style="height: 800px;"
                            />
                        </div>
                        <div class="col">
                            <q-stepper
                                v-model="leaderSettingStep"
                                header-nav
                                ref="stepper"
                                color="primary"
                                animated
                                vertical
                                style="height: 100%;"
                                dark
                                class="border-white border-rounded"
                            >
                                <q-step
                                    :name="1"
                                    title="Origin Setting"
                                    :done="leaderSettingStep > 1"
                                    :header-nav="leaderSettingStep > 1"
                                    style="height: 100%;"
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
                                            color="positive"
                                            label="Start Teleoperation"
                                            @click="startLeaderTele(assembly.id, 'start_leader_robot')"
                                            class="full-width q-mt-md"
                                            outline
                                            v-if="!leaderTeleStarted"
                                        />
                                        <q-btn
                                            color="orange-8"
                                            label="Stop Teleoperation"
                                            @click="stopLeaderTele()"
                                            class="full-width q-mt-md"
                                            outline
                                            v-else
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
                                    title="EMA Factor Setting"
                                    :header-nav="leaderSettingStep > 4"
                                >
                                    Set the EMA factor for the leader robot's teleoperation here.
                                    Bigger values result in smoother but more delayed movements.
                                    <div>
                                        <q-btn
                                            color="primary"
                                            label="Start Teleoperation"
                                            @click="startLeaderTele(assembly.id, 'start_leader_robot')"
                                            class="full-width q-mt-md"
                                            outline
                                            v-if="!leaderTeleStarted"
                                        />
                                        <q-btn
                                            color="orange-8"
                                            label="Stop Teleoperation"
                                            @click="stopLeaderTele()"
                                            class="full-width q-mt-md"
                                            outline
                                            v-else
                                        />
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md">
                                        <q-input
                                            v-model="leaderSettingForm.ema"
                                            @update:model-value="saveLeaderSetting"
                                            label="EMA filter value"
                                            type="number"
                                            dense
                                            outlined
                                            class="full-width"
                                            dark
                                        
                                        />
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
import { ref, onMounted, onUnmounted, defineProps, computed } from 'vue';
import { useSocket } from 'src/composables/useSocket';
import ProcessConsole from './ProcessConsole.vue';
import { useROS } from 'src/composables/useROS';
import { useLeaderTeleoperation } from 'src/composables/useLeaderTeleoperation';
import { api } from 'src/boot/axios';
import { useProcessStore } from 'src/stores/processStore';

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


const teleSettingTab = ref('leader');
const leaderSettingStep = ref(1);
const leaderRobotStarted = computed(() => {
    return processStore.isRunning('subscribe_dxl');
}); 
const leaderSettingForm = ref(props.assembly.teleoperators.find(e => e.type === 'leader') ?  props.assembly.teleoperators.find(e => e.type === 'leader').settings : {
    joint_map: [],
    ema: 0.5,
});


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