<template>
    <q-page class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="robot in robots" :key="robot.id">
                <q-card >
                    <q-img :src="robot.image" @click="watchRobot(robot)" class="cursor-pointer" ratio="1">
                        <div class="absolute-bottom text-h6 row q-gutter-x-sm">
                            <div>{{ robot.name }}</div>
                            <q-space></q-space>
                            <q-icon v-if="robot.status === 'on' && watchingRobot === robot" color="positive" name="visibility" size="sm" class="cursor-pointer"></q-icon>
                            <q-icon v-if="robot.status === 'on'" color="positive" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleRobot(robot)"></q-icon>
                            <q-icon v-if="robot.status === 'off'" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleRobot(robot)"></q-icon>
                            <q-icon v-if="robot.status === 'loading'" color="orange-6" name="power_settings_new" size="sm" class="cursor-pointer"></q-icon>
                        </div>
                        <div class="absolute-top-right row q-gutter-x-sm" style="background: none;">
                            <q-btn-dropdown
                                dropdown-icon="more_vert"
                                flat
                                text-color="dark"
                                round
                            >
                                <q-list bordered separator>
                                    <q-item clickable v-ripple v-close-popup @click="showRobotForm = true; robotForm = robot">
                                        <q-item-section>Edit Robot</q-item-section>
                                        <q-item-section side>
                                            <q-icon name="edit" size="xs" />
                                        </q-item-section>
                                    </q-item>
                                    <q-item clickable v-ripple class="text-negative" @click="deleteRobot(robot)">
                                        <q-item-section>Delete Robot</q-item-section>
                                        <q-item-section side>
                                            <q-icon color="negative" name="delete" size="xs" />
                                        </q-item-section>
                                    </q-item>
                                </q-list>
                            </q-btn-dropdown>
                        </div>
                    </q-img>


                    <q-card-section>
                    <div class="text-grey-6 text-caption">Robot</div>
                    <div class="row">
                        <div>{{ robot.type }}</div>
                    </div>
                    </q-card-section>
                </q-card>
            </div>
            <div class="col-6 col-sm-4 col-md-3 col-lg-2">
                <q-btn color="grey-8" class="full-height full-width" outline size="lg" icon="add" @click="showRobotForm = true"></q-btn>
            </div>
        </div>
        <div class="absolute-bottom bg-grey-4"  v-if="watchingRobot">
            <q-separator />
            <div class="q-pa-md row q-gutter-x-md">
                <div class="col-3 column">
                    <div class="col" 
                        v-for="(joint, i) in watchingRobot.joint_names" :key="joint"
                    >
                        <div>{{ joint }}</div>
                        <q-slider
                            v-model="watchingRobot.joint_pos[i]"
                            :min="watchingRobot.joint_lower_bounds[i]"
                            :max="watchingRobot.joint_upper_bounds[i]"
                            :step="0.01"
                            label
                            label-always
                            switch-label-side
                            color="red"
                            :disable="!canControl"
                            @update:model-value="(e) => moveRobot(i, e)"
                        />
                    </div>
                </div>
                <div class="col">
                    <div style="height: 30px" class="row">
                        <div 
                            class="bg-dark col text-white text-center" 
                            v-for="robot in robots.filter((e) => e.status !== 'off')"
                            :key="robot.id"
                            :style="robot.id !== watchingRobot.id ? 'border: 1px solid #ffffff' : ''"
                            :class="robot.id !== watchingRobot.id ? 'cursor-pointer': ''"
                            @click="watchRobot(robot)"
                        >{{ robot.name }}</div>
                    </div>
                    <process-console 
                        :process="robot.process_id" 
                        v-for="robot in robots.filter((e) => e.status !== 'off')"
                        :key="robot.id"
                        v-show="robot.id === watchingRobot.id"
                    />
                </div>
            </div>
        </div>
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
                </q-card-section>

                <q-card-actions align="center" class="text-primary">
                    <q-btn flat label="Save" v-close-popup @click="saveRobot" />
                    <q-btn flat color="grey-7" label="Close" v-close-popup @click="robotForm = {}" />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { onMounted, onUnmounted, ref } from 'vue';

import { useSocket } from '../composables/useSocket';
import { useROS } from '../composables/useROS';
import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Notify } from 'quasar';

const { socket } = useSocket();
const { createSubscriber, createPublisher, connectROS, sendJointState } = useROS();

const robots = ref([]);

const robotForm = ref({});
const watchingRobot = ref(null);

const robotTypeOptions = [
    { label: 'UR5e', value: 'ur5e' },
    { label: 'PIPER', value: 'piper' },
]

function listRobots() {                                                                                                     
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.image = '/images/' + robot.type + '.png'; // Default image if not provided
            robot.status = 'off'; // Initialize robot state
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
            message: 'Please fill the form'
        })
        return;
    }
    if (robotForm.value.id) {
        return api.put(`/robot/${robotForm.value.id}`, {
            'name': robotForm.value.name,
            'type': robotForm.value.type
        }).then(() => {
            robotForm.value = {};
        })
    } else {
        return api.post(`/robot`, {
            'name': robotForm.value.name,
            'type': robotForm.value.type
        }).then(() => {
            robotForm.value = {};
            listRobots()
        })
    }
}

function deleteRobot(robot) {
    return api.delete(`/robot/${robot.id}`).then(() => {
        listRobots()
    })
}

function listProcesses() {
    return api.get('/processes').then((response) => {
        const processes = response.data.processes || [];
        robots.value.forEach(robot => {
            const process = processes.find(p => p === robot.process_id);
            if (process) {
                robot.status = 'on'; // Sensor is running
                robot.process = process;
                if (!watchingRobot.value) {
                    watchRobot(robot)
                }
            } else {
                robot.status = 'off'; // Sensor is not running
            }
        });
    }).catch((error) => {
        console.error('Error fetching processes:', error);
    });
}

function toggleRobot(robot) {
    if (robot.status === 'on') {
        stopRobot(robot).then(() => {
            watchingRobot.value = null; // Stop watching if robot is stopped
        });
    } else {
        startRobot(robot).then(() => {
        });
    }
}

function startRobot(robot) {
    robot.status = 'loading';
    watchingRobot.value = robot
    return api.post('/robot:start', robot).catch((error) => {
        console.error('Error starting robot:', error);
        robot.status = 'off'; // Reset status on error
    });
}

function stopRobot(robot) {
    robot.status = 'loading';
    return api.post('/robot:stop', robot).catch((error) => {
        console.error('Error stopping robot:', error);
        robot.status = 'on'; // Reset status on error
    });
}

let jointSub = null
let publishJointPos = () => {}

function watchRobot(robot) {
    if (robot.status === 'off') {
        return;
    }
    if (jointSub) {
        jointSub.unsubscribe()
    }
    jointSub = createSubscriber(robot.read_topic, robot.read_topic_msg, (msg) => {
        if (!canControl.value) {
            robot.joint_pos = msg.position
        }
        canControl.value = true
    })
    publishJointPos = createPublisher(robot.write_topic, robot.write_topic_msg)
    watchingRobot.value = robot
}

function moveRobot(joint_index, joint_pos) {
    watchingRobot.value.joint_pos[joint_index] = joint_pos
    sendJointState(watchingRobot.value.joint_names, watchingRobot.value.joint_pos, publishJointPos)
}

const canControl = ref(false)


const showRobotForm = ref(false)

onMounted(() => {
    
    connectROS()
    
    socket.on('start_process', (data) => {
        const robot = robots.value.find(s => s.process_id === data.id);
        if (robot) {
            robot.status = 'on';
            watchRobot(robot)
        }
    });

    socket.on('stop_process', (data) => {
        const robot = robots.value.find(s => s.process_id === data.id);
        if (robot) {
            robot.status = 'off';
            if (watchingRobot.value && watchingRobot.value.id === robot.id && robots.value.find((e) => e.status === 'on')) {
                watchRobot(robots.value.find((e) => e.status === 'on'))
            } else {
                watchingRobot.value = null
            }
        }
    });


    listRobots().then(() => {
        listProcesses();
    })
})

onUnmounted(() => {
    socket.off('start_process');
});

</script>
