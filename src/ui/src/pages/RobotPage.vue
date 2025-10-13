<template>
    <q-page class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="robot in robots" :key="robot.id">
                <q-card>
                    <q-img :src="robot.image" @click="watchRobot(robot)" class="cursor-pointer" ratio="1">
                        <div class="absolute-bottom text-h6 row q-gutter-x-sm">
                            <div>{{ robot.name }}</div>
                            <q-space></q-space>
                            <span v-if="robot.type !== 'custom'">
                                <q-icon v-if="robot.status === 'on' && watchingRobot && watchingRobot.id === robot.id" color="positive" name="visibility" size="sm" class="cursor-pointer"></q-icon>
                                <q-icon v-if="robot.status === 'on'" color="positive" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleRobot(robot)"></q-icon>
                                <q-icon v-if="robot.status === 'off'" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleRobot(robot)"></q-icon>
                                <q-icon v-if="robot.status === 'loading'" color="orange-6" name="power_settings_new" size="sm" class="cursor-pointer"></q-icon>
                            </span>
                            <q-badge v-else-if="robot.status === 'on'" color="green-6">topic on</q-badge>
                            <q-badge v-else-if="robot.status === 'off'" color="grey-6">topic off</q-badge>
                        </div>
                        <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click="showRobotForm = true; robotForm = robot">
                                    <q-item-section>Edit Robot</q-item-section>
                                    <q-item-section side>
                                        <q-icon name="edit" size="xs" />
                                    </q-item-section>
                                </q-item>
                                <q-item clickable v-ripple @click="openTeleSetting(robot)">
                                    <q-item-section>Teleoperation Setting</q-item-section>
                                    <q-item-section side>
                                        <q-icon name="gamepad" size="xs" />
                                    </q-item-section>
                                </q-item>
                                <!-- <q-item clickable v-ripple class="text-negative" @click="deleteRobot(robot)">
                                    <q-item-section>Delete Robot</q-item-section>
                                    <q-item-section side>
                                        <q-icon color="negative" name="delete" size="xs" />
                                    </q-item-section>
                                </q-item> -->
                            </q-list>
                        </q-menu>
                    </q-img>


                    <q-card-section>
                    <div class="text-grey-6 text-caption">Robot</div>
                    <div class="row">
                        <div>{{ robot.type }}</div>
                    </div>
                    </q-card-section>
                </q-card>
            </div>
            <div class="col" v-if="!robots.length">
                <q-card class="full-height" flat bordered>
                    <q-card-section class="text-center">
                        <div class="text-h6">No Robots Found</div>
                        <div class="text-subtitle2">Click the button below to add a new robot.</div>
                    </q-card-section>
                </q-card>
            </div>
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" style="min-height: 220px;">
                <q-btn color="primary" class="full-height full-width" outline size="lg" icon="add" @click="showRobotForm = true"></q-btn>
            </div>
        </div>

        <bottom-terminal
            :tabs="robots.filter((e) => e.status !== 'off')"
            tab-label="name"
            tab-value="id"
            v-model="watchingRobot"
            v-if="robots.filter((e) => e.status !== 'off').length > 0 && watchingRobot"
            @update:model-value="watchRobot($event)"
        >
            <template v-for="robot in robots.filter((e) => e.status !== 'off')" :key="robot.id" v-slot:[robot.id]>
                <div class="row q-pa-md row q-gutter-x-md ">
                    <div class="col-3 column">
                        <div class="col" 
                            v-for="(joint, i) in watchingRobot.joint_names" :key="joint"
                        >
                            <div class="text-caption">{{ joint }}</div>
                            <q-slider
                                v-model="watchingRobot.joint_pos[i]"
                                :min="watchingRobot.joint_lower_bounds[i]"
                                :max="watchingRobot.joint_upper_bounds[i]"
                                :step="0.0001"
                                label
                                switch-label-side
                                thumb-size="1px"
                                color="red"
                                :disable="!canControl"
                                @update:model-value="(e) => watchingRobot.handler.moveRobot(i, e)"
                            />
                        </div>
                    </div>
                    <div class="col-8">
                        <process-console 
                            :process="robot.process_id" 
                            :key="robot.id"
                            style="height: 100%"
                        />
                    </div>
                    <div class="col">
                        <div class="q-gutter-sm">
                            <q-btn @click="watchingRobot.handler.goOriginPos" icon="home" color="green">
                                <q-tooltip class="text-body2">Click to go origin position</q-tooltip>
                            </q-btn>
                            <div v-if="watchingRobot.leader_robot_preset">
                                <q-btn @click="() => { startLeaderTele(watchingRobot, watchingRobot.leader_robot_preset, 'log_' + watchingRobot.process_id) }" icon="play_arrow" color="blue" v-if="!leaderTeleStarted">
                                    <q-tooltip class="text-body2">Start teleoperation with leader robot</q-tooltip>
                                </q-btn>
                                <q-btn @click="() => { stopLeaderTele() }" icon="pause" color="blue" v-else>
                                    <q-tooltip class="text-body2">Start teleoperation with leader robot</q-tooltip>
                                </q-btn>
                            </div>
                        </div>
                    </div>
                </div>
                
            </template>
        </bottom-terminal>
        <q-dialog v-model="showRobotForm">
            <q-card style="min-width: 350px">
                <q-card-section class="q-pt-none">
                    <q-card-section>
                        <div class="text-h6 text-center">Robot</div>
                    </q-card-section>
                    <q-input
                        dense
                        v-model="robotForm.name"
                        label="Robot Name"
                        autofocus
                        class="q-mb-md"
                    />

                    <q-select
                        dense
                        v-model="robotForm.type"
                        :options="robotTypeOptions"
                        label="Robot Type"
                        class="q-mb-md"
                        map-options
                        emit-value
                    />

                    <div v-if="robotForm.type === 'piper'">
                        <q-input
                            dense
                            v-for="input in robotTypeOptions.find((e) => e.value === 'piper').form"
                            :key="input.label"
                            v-model="robotForm[input.key]"
                            :label="input.label"
                            :type="input.type"
                            class="q-mb-md"
                        />
                    </div>

                    <div v-if="robotForm.type === 'custom'">
                        <q-input
                            dense
                            v-model="robotForm.read_topic"
                            label="Read Topic"
                            class="q-mb-md"
                        />
                        <q-input
                            dense
                            v-model="robotForm.read_topic_msg"
                            label="Read Topic Message Type"
                            class="q-mb-md"
                        />
                        <q-input
                            dense
                            v-model="robotForm.write_topic"
                            label="Write Topic"
                            class="q-mb-md"
                        />
                        <q-input
                            dense
                            v-model="robotForm.write_topic_msg"
                            label="Write Topic Message Type"
                            class="q-mb-md"
                        />
                        <div class="row q-mb-md q-col-gutter-sm">
                            <q-input
                                outlined
                                dense
                                v-model="robotForm.joint_names[i]"
                                :label="`Joint Name ${i + 1}`"
                                class="col-3"
                                v-for="(joint, i) in robotForm.joint_names"
                                :key="i"
                            >
                                <template v-slot:append>
                                    <q-icon size="xs" name="close" @click="robotForm.joint_names.splice(i, 1); robotForm.joint_lower_bounds.splice(i, 1); robotForm.joint_upper_bounds.splice(i, 1);" class="cursor-pointer" />
                                </template>
                            </q-input>
                            <div class="col-3">
                                <q-btn
                                    dense
                                    outline
                                    color="primary"
                                    label="+ Add Joint"
                                    class="full-width full-height"
                                    @click="robotForm.joint_names.push(''); robotForm.joint_lower_bounds.push(-3.14); robotForm.joint_upper_bounds.push(3.14);"
                                ></q-btn>
                            </div>
                        </div>
                        <q-separator class="q-my-md"></q-separator>
                        <div class="row q-mb-md q-col-gutter-sm">
                            <q-input
                                outlined
                                dense
                                v-model.number="robotForm.joint_lower_bounds[i]"
                                :label="`Joint ${i + 1} Lower Bound`"
                                class="col-3"
                                v-for="(joint, i) in robotForm.joint_lower_bounds"
                                :key="i"
                            ></q-input>
                        </div>
                        <q-separator class="q-my-md"></q-separator>
                        <div class="row q-mb-md q-col-gutter-sm">
                            <q-input
                                outlined
                                dense
                                v-model.number="robotForm.joint_upper_bounds[i]"
                                :label="`Joint ${robotForm.joint_names[i] || i + 1} Upper Bound`"
                                class="col-3"
                                v-for="(joint, i) in robotForm.joint_upper_bounds"
                                :key="i"
                            ></q-input>
                        </div>
                        <div class="row q-mb-md q-col-gutter-sm">
                            <q-input
                                outlined
                                dense
                                v-model.number="robotForm.gripper_range[i]"
                                :label="`Gripper Range`"
                                class="col-6"
                                v-for="i in [0, 1]"
                                :key="i"
                            ></q-input>
                        </div>

                    </div>
                </q-card-section>

                <q-card-actions align="center" class="text-primary">
                    <q-btn flat label="Save" @click="saveRobot" />
                    <q-btn flat color="grey-7" label="Close" v-close-popup @click="robotForm = {}" />
                </q-card-actions>
            </q-card>
        </q-dialog>
        <tele-setting-dialog
            v-if="showTeleSetting"
            v-model="showTeleSetting" 
            :robot="teleSettingRobot"
            @hide="closeTeleSetting"
        />
    </q-page>
</template>

<script setup>
import { onMounted, onUnmounted, ref, watch } from 'vue';

import { useSocket } from '../composables/useSocket';
import { useROS } from '../composables/useROS';
import { useLeaderTeleoperation } from '../composables/useLeaderTeleoperation';
import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import TeleSettingDialog from 'src/components/TeleSettingDialog.vue';
import { Notify } from 'quasar';
import BottomTerminal from 'src/components/BottomTerminal.vue';
import { useRobot } from '../composables/useRobot';

const { socket } = useSocket();
const { connectROS } = useROS();
const { leaderTeleStarted, startLeaderTele, stopLeaderTele } = useLeaderTeleoperation();

const robots = ref([]);

const robotForm = ref({
    name: '',
    type: '',
    read_topic: '',
    read_topic_msg: '',
    write_topic: '',
    write_topic_msg: '',
    joint_names: [],
    joint_lower_bounds: [],
    joint_upper_bounds: [],
    gripper_range: [0, 1],
});
const watchingRobot = ref(null);

const robotTypeOptions = [
    { label: 'UR5e', value: 'ur5e' },
    { label: 'PIPER', value: 'piper', form: [
        { label: 'CAN Port', key: 'can_port', type: 'text', default: 'can_0' }
    ]},
    { label: 'Custom Robot', value: 'custom'}
]

function listRobots() {                                                                                                     
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.image = '/images/' + robot.type + '.png'; // Default image if not provided
            robot.loading = false;
            robot.handler = useRobot(robot, () => {
                watchRobot(robot);
            });
            robot.joint_pos = []
            robot.joint_names.forEach((joint, i) => {
                robot.joint_pos[i] = 0
            })
        });
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

function saveRobot() {
    if (!robotForm.value.name || !robotForm.value.type) {
        Notify.create({
            color: 'negative',
            message: 'Please fill all robot fields'
        })
        return;
    }
    if (robotForm.value.type === 'custom' && (!robotForm.value.read_topic || !robotForm.value.read_topic_msg || !robotForm.value.write_topic || !robotForm.value.write_topic_msg)) {
        Notify.create({
            color: 'negative',
            message: 'Please fill all robot fields'
        })
        return;
    }
    if (robotForm.value.type === 'custom' && robotForm.value.joint_names.length === 0) {
        Notify.create({
            color: 'negative',
            message: 'Please add at least one joint'
        })
        return;
    }
    if (robotForm.value.type === 'custom') {
        for (let i = 0; i < robotForm.value.joint_names.length; i++) {
            if (robotForm.value.joint_names[i] === '' || robotForm.value.joint_lower_bounds[i] === undefined || robotForm.value.joint_upper_bounds[i] === undefined) {
                Notify.create({
                    color: 'negative',
                    message: 'Please fill all joint fields'
                })
                return;
            }
            if (robotForm.value.joint_lower_bounds[i] >= robotForm.value.joint_upper_bounds[i]) {
                Notify.create({
                    color: 'negative',
                    message: `Joint ${robotForm.value.joint_names[i]} lower bound must be less than upper bound`
                })
                return;
            }
            if (robotForm.value.gripper_range[0] >= robotForm.value.gripper_range[1]) {
                Notify.create({
                    color: 'negative',
                    message: 'Please set valid gripper range'
                })
                return;
            }
        }
    }
    const data = {
        'name': robotForm.value.name,
        'type': robotForm.value.type,
        'read_topic': robotForm.value.read_topic || '',
        'read_topic_msg': robotForm.value.read_topic_msg || '',
        'write_topic': robotForm.value.write_topic || '',
        'write_topic_msg': robotForm.value.write_topic_msg || '',
        'joint_names': robotForm.value.joint_names,
        'joint_lower_bounds': robotForm.value.joint_lower_bounds,
        'joint_upper_bounds': robotForm.value.joint_upper_bounds,
        'can_port': robotForm.value.can_port || 'can_0',
        'gripper_range': robotForm.value.gripper_range || [0, 1],
    };
    if (robotForm.value.id) {
        return api.put(`/robot/${robotForm.value.id}`, data).then(() => {
            showRobotForm.value = false;
            robotForm.value = {};
            listRobots()
        })
    } else {
        return api.post(`/robot`, data).then(() => {
            showRobotForm.value = false;
            robotForm.value = {};
            listRobots()
        })
    }
}

// function deleteRobot(robot) {
//     if (robot.status === 'on') {
//         Notify.create({
//             color: 'negative',
//             message: 'Turn off the robot first.'
//         })
//         return;
//     }
//     return api.delete(`/robot/${robot.id}`).then(() => {
//         initialize()
//     })
// }

function toggleRobot(robot) {
    if (robot.status === 'on') {
        robot.handler.stopRobot().then(() => {
            watchingRobot.value = null; // Stop watching if robot is stopped
        });
    } else {
        robot.handler.startRobot().then(() => {
            watchRobot(robot); // Start watching the robot after it is started
        })
    }
}

function watchRobot(robot) {
    robot.handler.subscribeRobot((msg) => {
        if (!canControl.value) {
            robot.joint_pos = msg.position
        }
    })
    robot.handler.publishRobot()
    canControl.value = true
    watchingRobot.value = robot
}

const canControl = ref(false)
const showRobotForm = ref(false)

const showTeleSetting = ref(false)
const teleSettingRobot = ref(null)

function openTeleSetting(robot) {
    if (robot.status === 'off') {
        Notify.create({
            color: 'negative',
            message: 'Turn on the robot first.'
        })
        return;
    }
    showTeleSetting.value = true;
    teleSettingRobot.value = robot;
    canControl.value = false; // Reset control state
}

function closeTeleSetting(leaderSettingForm) {
    if (leaderSettingForm) {
        robots.value.find((e) => e.id === teleSettingRobot.value.id).leader_robot_preset = leaderSettingForm;
    }
    showTeleSetting.value = false;
    teleSettingRobot.value = null;
    canControl.value = true; // Reset control state
}

watch(watchingRobot, (newVal, oldVal) => {
    if (oldVal) {
        stopLeaderTele(); // Stop leader teleoperation when switching robots
    }
});


function initialize() {
    listRobots()
}

onMounted(() => {
    
    connectROS()
    
    socket.on('start_process', (data) => {
        if (data.id === 'leader_teleoperation') {
            leaderTeleStarted.value = true;
        }
    });

    socket.on('stop_process', (data) => {
        if (data.id === 'leader_teleoperation') {
            leaderTeleStarted.value = false;
        }
    });


    initialize()
})

onUnmounted(() => {
    if (watchingRobot.value) {
        watchingRobot.value.handler.unSubscribeRobot();
    }
});

</script>
