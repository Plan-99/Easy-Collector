<template>
    <q-page class="q-pa-md full-height">
        <div class="row q-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="robot in robots" :key="robot.id">
                <q-card >
                    <q-img :src="robot.image" @click="watchRobot(robot)" class="cursor-pointer">
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
        <div class="absolute-bottom bg-grey-4">
            <q-separator />
            <div class="q-pa-md row q-gutter-x-md" v-if="watchingRobot">
                <process-console :process="watchingRobot.process_id" class="col" />
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

                    <q-input
                        dense
                        v-model="robotForm.serial_no"
                        label="Serial Number"
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
import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Notify } from 'quasar';
const { socket } = useSocket();

const robots = ref([]);

const robotForm = ref({});
const watchingRobot = ref(null);

const robotTypeOptions = [
    { label: 'UR5e', value: 'ur5e' }
]

function listRobots() {                                                                                                     
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.image = '/images/' + robot.type + '.png'; // Default image if not provided
            robot.status = 'off'; // Initialize robot state
        });
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

function saveRobot() {
    if (!robotForm.value.name || !robotForm.value.type || !robotForm.value.serial_no) {
        Notify.create({
            color: 'negative',
            message: 'Please fill the form'
        })
        return;
    }
    if (robotForm.value.id) {
        return api.put(`/robot/${robotForm.value.id}`, {
            'serial_no': robotForm.value.serial_no,
            'name': robotForm.value.name,
            'type': robotForm.value.type
        }).then(() => {
            robotForm.value = {};
        })
    } else {
        return api.post(`/robot`, {
            'serial_no': robotForm.value.serial_no,
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
                robot.status = 'on'; // Robot is running
                robot.process = process;
            } else {
                robot.status = 'off'; // Robot is not running
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


const showRobotForm = ref(false)

onMounted(() => {

    socket.on('start_process', (data) => {
        const robot = robots.value.find(s => s.process_id === data.id);
        if (robot) {
            robot.status = 'on';
        }
    });

    socket.on('stop_process', (data) => {
        const robot = robots.value.find(s => s.process_id === data.id);
        if (robot) {
            robot.status = 'off';
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
