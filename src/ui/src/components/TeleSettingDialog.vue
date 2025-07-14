<template>
    <q-dialog persistent>
        <q-card style="min-width: 80%">
            <q-card-section class="q-pt-none">
                <q-btn class="absolute-top-right" size="md" icon="close" flat round style="z-index: 3;" @click="$emit('hide')"></q-btn>
                <q-card-section>
                    <div class="text-h6 text-center">Teleoperation Setting</div>
                </q-card-section>
                <q-tabs
                    v-model="teleSettingTab"
                    dense
                    class="text-grey"
                    active-color="primary"
                    indicator-color="primary"
                    align="justify"
                    narrow-indicator
                >
                    <q-tab name="leader" label="Leader Robot" />
                    <q-tab name="keyboard" label="Keyboard" />
                </q-tabs>
                <q-separator />
                <q-tab-panels v-model="teleSettingTab" animated>
                    <q-tab-panel name="leader" class="row q-col-gutter-x-md">
                        <div class="col">
                            <div class="row q-col-gutter-x-md q-mt-md">
                                <div class="col">
                                    <q-select
                                        label="Serial Port"
                                        :options="['/dev/ttyUSB0', '/dev/ttyUSB1']"
                                        v-model="leaderPort"
                                        dense
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
                                class="q-mt-md"
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
                                            @click="goOriginPos"
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
                                            />
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="() => { leaderSettingStep = 2 }" color="primary" label="Save & Go Next" :disable="!leaderSettingForm.origin" />
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
                                        <div class="col-6" v-for="i in [0, 1]" :key="i">
                                            <q-input
                                                v-model="leaderSettingForm.gripper_dxl_range[i]"
                                                :label="`${i === 0 ? 'Open' : 'Close'} Position`"
                                                type="number"
                                                dense
                                                outlined
                                                v-if="i === 0 || gripperDxlRangeSaved[0]"
                                            >
                                                <template v-slot:append>
                                                    <q-btn
                                                        color="primary"
                                                        :outline="!gripperDxlRangeSaved[i]"
                                                        size="sm"
                                                        @click="() => { gripperDxlRangeSaved[i] = !gripperDxlRangeSaved[i]; }"
                                                    >Save</q-btn>
                                                </template>
                                            </q-input>
                                        </div>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="() => { leaderSettingStep = 3; saveLeaderSetting() }" color="primary" label="Go Next" :disable="!(gripperDxlRangeSaved[0] && gripperDxlRangeSaved[1])" />
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
                                            color="primary"
                                            label="Start Teleoperation"
                                            @click="startLeaderTele(robot.id, 'log_start_leader_robot')"
                                            class="full-width q-mt-md"
                                            outline
                                            v-if="!leaderTeleStarted"
                                        />
                                        <q-btn
                                            color="orange-8"
                                            label="Stop Teleoperation"
                                            @click="stopLeaderTele(robot.id, 'log_start_leader_robot')"
                                            class="full-width q-mt-md"
                                            outline
                                            v-else
                                        />
                                    </div>
                                    <div class="row q-col-gutter-sm q-mt-md">
                                        <div class="col-3" v-for="(sign, i) in leaderSettingForm.sign_corrector" :key="i">
                                            <q-toggle
                                                v-model="leaderSettingForm.sign_corrector[i]"
                                                :label="robot.joint_names[i]"
                                                outlined
                                                :true-value="-1"
                                                :false-value="1"
                                                @update:model-value="saveLeaderSetting"
                                            />
                                        </div>
                                        <q-banner class="col-12 q-mt-md bg-primary text-white">
                                            <div>
                                                Check the joint of the robot which moves in the opposite direction of the leader's joint.
                                            </div>
                                        </q-banner>
                                    </div>
                                    <q-stepper-navigation>
                                        <q-btn @click="$emit('hide', leaderSettingForm)" color="primary" label="Finish" />
                                    </q-stepper-navigation>
                                </q-step>

                            </q-stepper>
                        </div>
                    </q-tab-panel>

                    <q-tab-panel name="keyboard">
                        <div class="text-h6">Alarms</div>
                        Lorem ipsum dolor sit amet consectetur adipisicing elit.
                    </q-tab-panel>
                </q-tab-panels>
            </q-card-section>
        </q-card>
    </q-dialog>
</template>

<script setup>
import { ref, onMounted, onUnmounted, defineProps } from 'vue';
import { useSocket } from '../composables/useSocket';
import ProcessConsole from './ProcessConsole.vue';
import { useROS } from '../composables/useROS';
import { useLeaderTeleoperation } from '../composables/useLeaderTeleoperation';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';

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
const leaderPort = ref('/dev/ttyUSB0');
const leaderRobotStarted = ref(false);
// const robotId = ref(props.robot.id);
const leaderSettingForm = ref(props.robot.leader_robot_preset || {
    gripper_dxl_range: [0, 0],
    sign_corrector: [1, 1, 1, 1, 1, 1, 1],
});
const gripperDxlRangeSaved = ref([false, false]);


function startLeaderRobot() {
    api.post('/leader_robot:start', {
        serial_port: leaderPort.value
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

function goOriginPos() {
    const robot = props.robot;
    api.post(`/robot/${robot.id}/:move_to`, {
        goal_pos: [0, 0, 0, 0, 0, 0] // Default to zero if not set
    }).then(() => {
        Notify.create({
            color: 'positive',
            message: 'Robot moved to origin position'
        })
    }).catch((error) => {
        console.error('Error moving robot to origin:', error);
        Notify.create({
            color: 'negative',
            message: 'Failed to move robot to origin'
        })
    });
}

onMounted(() => {
    connectROS();

    socket.on('start_process', (data) => {
        if (data.id === 'start_leader_robot') {
            leaderRobotStarted.value = true;
            let gripperDxlId;
            let gripperDxlIdIndex;
            createSubscriber('/dynamixel/data', 'dynamixel_ros/DynamixelData', (msg) => {
                gripperDxlId = Math.max(...msg.ids);
                gripperDxlIdIndex = msg.ids.indexOf(gripperDxlId);
                if (leaderSettingStep.value === 1) {
                    leaderSettingForm.value.dxl_ids = msg.ids;
                    leaderSettingForm.value.origin = msg.values;
                }
                if (!gripperDxlRangeSaved.value[0]) {
                    leaderSettingForm.value.gripper_dxl_range[0] = msg.values[gripperDxlIdIndex];
                } else if (!gripperDxlRangeSaved.value[1]) {
                    leaderSettingForm.value.gripper_dxl_range[1] = msg.values[gripperDxlIdIndex];
                }
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
    stopLeaderTele(props.robot.id);
    socket.off('leader_robot_started');
    socket.off('leader_robot_stopped');
});

</script>