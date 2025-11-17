<template>
    <q-dialog persistent>
        <q-card style="min-width: 80%" dark class="bg-dark border-rounded border-white">
            <q-card-section class="q-pt-none">
                <q-btn class="absolute-top-right" size="md" icon="close" flat round style="z-index: 3;" @click="$emit('hide')"></q-btn>
                <q-card-section>
                    <div class="text-h6 text-center">Teleoperation Setting</div>
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
                        <div class="col ">
                            <div class="row q-col-gutter-x-md q-mt-md">
                                <div class="col">
                                    <q-select
                                        label="Serial Port"
                                        :options="['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3', '/dev/ttyUSB4', '/dev/ttyUSB5']"
                                        v-model="leaderSettingForm.port_name"
                                        dense
                                        dark
                                        bg-color="dark"
                                        outlined
                                    ></q-select>
                                </div>
                                <div  class="col">
                                    <q-btn class="full-width full-height" outline color="primary" @click="startLeaderRobot" v-if="!leaderRobotStarted">Start Leader Robot</q-btn>
                                    <q-btn class="full-width full-height" outline color="orange-8" @click="stopLeaderRobot" v-else>Stop Leader Robot</q-btn>
                                </div>
                            </div>
                            <process-console 
                                process="start_leader_robot"
                                class="q-mt-md border-white"
                                style="height: 600px;"
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
                                        <q-btn
                                            color="green"
                                            label="Go to Origin Position"
                                            icon="home"
                                            @click="robot.handler.goOriginPos"
                                            class="full-width q-mt-md"
                                            outline
                                        />
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md" v-if="leaderSettingForm.origin">
                                        <div class="col-3" v-for="(o, i) in leaderSettingForm.origin" :key="i">
                                            <q-input
                                                v-model="leaderSettingForm.origin[i]"
                                                :label="`Dynamixel ${leaderSettingForm.dxl_ids[i]} Origin`"
                                                type="number"
                                                dense
                                                outlined
                                                dark
                                                class="bg-dark"
                                            />
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="() => { leaderSettingStep = 2 }" color="primary" outline label="Save & Go Next" :disable="!leaderSettingForm.origin" />
                                    </q-stepper-navigation>
                                </q-step>

                                <q-step
                                    :name="2"
                                    title="Gripper Setting"
                                    :done="leaderSettingStep > 2"
                                    :header-nav="leaderSettingStep > 2"
                                >
                                    Open and close your leader robot's gripper and press the button below to save the gripper position.
                                    <div class="row q-col-gutter-sm q-mt-md">
                                        <q-select
                                            v-model="leaderSettingForm.gripper_dxl_ids"
                                            dense
                                            :options="
                                                leaderSettingForm.dxl_ids.map(id => ({
                                                    label: `Dynamixel ${id}`,
                                                    value: id
                                                }))"
                                            label="Select Gripper Dynamixel IDs"
                                            multiple
                                            @update:model-value="onGripperDxlIdsChange"
                                            map-options
                                            emit-value
                                            dark
                                            outlined
                                            class="q-mb-md full-width bg-dark"
                                        />
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md" v-for="(dxl_id, id) in leaderSettingForm.gripper_dxl_ids" :key="id">
                                        <div class="col-6" v-for="i in [0, 1]" :key="i">
                                            <q-input
                                                v-model.number="leaderSettingForm.gripper_dxl_range[id][i]"
                                                :label="`${i === 0 ? 'Open' : 'Close'} Position`"
                                                type="number"
                                                dense
                                                outlined
                                                dark
                                                v-if="i === 0 || gripperDxlRangeSaved[id][0]"
                                            >
                                                <template v-slot:append>
                                                    <q-btn
                                                        color="primary"
                                                        :outline="!gripperDxlRangeSaved[id][i]"
                                                        size="sm"
                                                        @click="() => { gripperDxlRangeSaved[id][i] = !gripperDxlRangeSaved[id][i]; }"
                                                    >Save</q-btn>
                                                </template>
                                            </q-input>
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="() => { leaderSettingStep = 3; saveLeaderSetting() }" color="primary" outline label="Go Next" :disable="!gripperDxlRangeSaved.every((saved) => saved[0] && saved[1])" />
                                        <q-btn flat @click="leaderSettingStep = 1" color="primary" label="Back" class="q-ml-sm" />
                                    </q-stepper-navigation>
                                </q-step>

                                <q-step
                                    :name="3"
                                    title="Joint Direction Setting"
                                    :header-nav="leaderSettingStep > 3"
                                >
                                    You can set the joint direction of the leader robot's motors here.
                                    Please start teleoperation and map the direction of leader robot's motors to the worker robot's motors.
                                    <div>
                                        <q-btn
                                            color="positive"
                                            label="Start Teleoperation"
                                            @click="startLeaderTele(robot, leaderSettingForm, 'log_start_leader_robot')"
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
                                        <div class="col-6" v-for="(sign, i) in leaderSettingForm.sign_corrector" :key="i">
                                            <q-toggle
                                                v-model="leaderSettingForm.sign_corrector[i]"
                                                :label="robot.joint_names[i]"
                                                outlined
                                                :true-value="-1"
                                                :false-value="1"
                                                @update:model-value="saveLeaderSetting"
                                            />
                                        </div>
                                        <div class="col-12 q-mt-md border-white border-rounded q-pa-md ">
                                            <div>
                                                Check the joint of the robot which moves in the opposite direction of the leader's joint.
                                            </div>
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn flat @click="leaderSettingStep = 2" color="primary" label="Back" class="q-ml-sm" />
                                        <q-btn @click="() => { leaderSettingStep = 4 }" color="primary" label="Go Next" />
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
                                            @click="startLeaderTele(robot, leaderSettingForm, 'log_start_leader_robot')"
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
import { ref, onMounted, onUnmounted, defineProps } from 'vue';
import { useSocket } from 'src/composables/useSocket';
import ProcessConsole from './ProcessConsole.vue';
import { useROS } from 'src/composables/useROS';
import { useLeaderTeleoperation } from 'src/composables/useLeaderTeleoperation';
import { api } from 'src/boot/axios';
// import { Notify } from 'quasar';

const props = defineProps({
    robot: {
        type: Object,
        default: () => ({})
    },
})

const { socket } = useSocket();
const { connectROS, createSubscriber } = useROS(); 
const { leaderTeleStarted, startLeaderTele, stopLeaderTele } = useLeaderTeleoperation();

const teleSettingTab = ref('leader');
const leaderSettingStep = ref(1);
const leaderRobotStarted = ref(false);
// const robotId = ref(props.robot.id);
const leaderSettingForm = ref(props.robot.leader_robot_preset || {
    gripper_dxl_ids: [],
    gripper_dxl_range: [],
    dxl_ids: [],
    origin: [],
    sign_corrector: Array(props.robot.joint_names.length).fill(1),
    port_name: '/dev/ttyUSB0',
    ema: 0.5,
});
const gripperDxlRangeSaved = ref([[]]);
gripperDxlRangeSaved.value = leaderSettingForm.value.gripper_dxl_ids.map(() => [false, false]);

function startLeaderRobot() {
    api.post('/leader_robot:start', {
        serial_port: leaderSettingForm.value.port_name
    })
}

function stopLeaderRobot() {
    if (!leaderRobotStarted.value) {
        return;
    }
    api.post('/leader_robot:stop')
}

function saveLeaderSetting() {
    api.post('/leader_robot', {
        robot_id: props.robot.id,
        preset: leaderSettingForm.value
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

function onGripperDxlIdsChange(val) {
    for (let i = 0; i < val.length; i++) {
        if (!leaderSettingForm.value.gripper_dxl_range[i]) {
            leaderSettingForm.value.gripper_dxl_range[i] = [0, 0];
        }
        gripperDxlRangeSaved.value[i] = [false, false];
    }
}

onMounted(() => {
    connectROS();

    socket.on('start_process', (data) => {
        if (data.id === 'start_leader_robot') {
            leaderRobotStarted.value = true;
            createSubscriber('/dynamixel/data', 'dynamixel_ros/DynamixelData', (msg) => {
                // gripperDxlIdIndex = msg.ids.indexOf(gripperDxlId);
                if (leaderSettingStep.value === 1) {
                    leaderSettingForm.value.dxl_ids = [...msg.ids];
                    leaderSettingForm.value.origin = [...msg.values];
                }

                leaderSettingForm.value.gripper_dxl_ids.forEach((gripperDxlId, i) => {
                    const gripperDxlIdIndex = msg.ids.indexOf(gripperDxlId);

                    // 해당 그리퍼 ID가 메시지에 없으면 건너뛰기
                    if (gripperDxlIdIndex === -1) return;

                    const currentValue = Number(msg.values[gripperDxlIdIndex]);


                    // i번째 그리퍼의 최소값이 아직 저장되지 않았다면 현재 값으로 업데이트
                    if (!gripperDxlRangeSaved.value[i][0]) {
                        leaderSettingForm.value.gripper_dxl_range[i][0] = currentValue;
                    }
                    // i번째 그리퍼의 최대값이 아직 저장되지 않았다면 현재 값으로 업데이트
                    else if (!gripperDxlRangeSaved.value[i][1]) {
                        leaderSettingForm.value.gripper_dxl_range[i][1] = currentValue;
                    }
                });
            });
        }
        if (data.id === 'leader_teleoperation') {
            leaderTeleStarted.value = true;
        }
    });

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
});

</script>