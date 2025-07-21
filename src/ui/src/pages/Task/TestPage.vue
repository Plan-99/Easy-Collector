<template>
    <q-page class="q-pa-md">
        <q-stepper v-model="step" ref="stepper" color="primary" animated header-nav="false">
            <q-step
                :name="1"
                title="Setup Test Environment"
                icon="settings"
                :done="step > 1"
            >
                <div class="text-h6 q-mb-md">Configuration</div>

                <!-- Checkpoint Selection -->
                <q-select
                    outlined
                    v-model="selectedCheckpoint"
                    :options="checkpointOptions"
                    label="Select Checkpoint"
                    class="q-mb-md"
                    emit-value
                    map-options
                />

                <!-- Sensor Management -->
                <q-card class="q-mb-md">
                    <q-card-section>
                        <div class="text-h6">Sensors</div>
                    </q-card-section>
                    <q-separator />
                    <q-list bordered>
                        <q-item v-for="sensor in sensors" :key="sensor.id">
                            <q-item-section avatar>
                                <q-icon name="sensors" />
                            </q-item-section>
                            <q-item-section>
                                <q-item-label>{{ sensor.name }}</q-item-label>
                                <q-item-label caption>{{ sensor.type }}</q-item-label>
                            </q-item-section>
                            <q-item-section side>
                                <q-badge :color="sensor.is_running ? 'green' : 'red'" :label="sensor.is_running ? 'Online' : 'Offline'" />
                            </q-item-section>
                            <q-item-section side>
                                <q-btn
                                    size="sm"
                                    :color="sensor.is_running ? 'negative' : 'positive'"
                                    :label="sensor.is_running ? 'Stop' : 'Start'"
                                    @click="toggleSensor(sensor)"
                                />
                            </q-item-section>
                        </q-item>
                    </q-list>
                    <div v-if="cameraSensors.length" class="row q-col-gutter-md q-pa-md">
                        <div v-for="camera in cameraSensors" :key="camera.id" class="col-12 col-md-6">
                            <q-video v-if="camera.is_running" :src="camera.streaming_url" />
                        </div>
                    </div>
                </q-card>

                <!-- Robot Management -->
                <q-card>
                    <q-card-section>
                        <div class="text-h6">Robots</div>
                    </q-card-section>
                    <q-separator />
                    <q-list bordered>
                        <q-item v-for="robot in robots" :key="robot.id">
                            <q-item-section avatar>
                                <q-icon name="precision_manufacturing" />
                            </q-item-section>
                            <q-item-section>
                                <q-item-label>{{ robot.name }}</q-item-label>
                            </q-item-section>
                            <q-item-section side>
                                <q-badge :color="robot.is_running ? 'green' : 'red'" :label="robot.is_running ? 'Online' : 'Offline'" />
                            </q-item-section>
                             <q-item-section side>
                                <q-btn
                                    size="sm"
                                    :color="robot.is_running ? 'negative' : 'positive'"
                                    :label="robot.is_running ? 'Stop' : 'Start'"
                                    @click="toggleRobot(robot)"
                                />
                            </q-item-section>
                        </q-item>
                    </q-list>
                </q-card>

            </q-step>

            <q-step
                :name="2"
                title="Run Test"
                icon="play_circle_filled"
            >
                <div class="text-h6">Live Sensor Feeds</div>
                 <div class="row q-col-gutter-md">
                    <div v-for="camera in cameraSensors" :key="camera.id" class="col-12 col-md-6">
                        <q-card>
                            <q-card-section class="q-pa-none">
                                 <q-video v-if="camera.is_running" :src="camera.streaming_url" />
                                 <div v-else class="text-center q-pa-xl text-grey">
                                     <q-icon name="videocam_off" size="lg" />
                                     <div>{{ camera.name }} is offline</div>
                                 </div>
                            </q-card-section>
                        </q-card>
                    </div>
                </div>
            </q-step>

            <template v-slot:navigation>
                <q-stepper-navigation class="text-center">
                    <q-btn v-if="step > 1" flat color="primary" @click="stepper.previous()" label="Back" class="q-mr-sm" />
                    <q-btn @click="navigateStep" color="primary" :label="step === 1 ? 'Start Test' : 'Finish'" />
                </q-stepper-navigation>
            </template>
        </q-stepper>
    </q-page>
</template>

<script setup>
import { ref, onMounted, computed } from 'vue';
import { useQuasar, Notify } from 'quasar';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';

const $q = useQuasar();
const route = useRoute();
const stepper = ref(null);
const step = ref(1);
const taskId = route.params.id;

const checkpoints = ref([]);
const selectedCheckpoint = ref(null);
const sensors = ref([]);
const robots = ref([]);
const task = ref(null);

const checkpointOptions = computed(() =>
    checkpoints.value.map(c => ({ label: c.name, value: c.id }))
);

const cameraSensors = computed(() =>
    sensors.value.filter(s => s.type === 'camera' && s.is_running)
);

async function fetchTaskDetails() {
    try {
        const response = await api.get(`/tasks/${taskId}`);
        task.value = response.data.task;
        await fetchSensors(task.value.sensor_ids);
        await fetchRobots(task.value.robot_ids);
    } catch (error) {
        Notify.create({ color: 'negative', message: error + ': Failed to fetch task details.' });
    }
}

async function fetchCheckpoints() {
    try {
        const response = await api.get('/checkpoints');
        checkpoints.value = response.data.checkpoints.filter(c => c.task_id == taskId);
    } catch (error) {
        Notify.create({ color: 'negative', message: error + ': Failed to load checkpoints.' });
    }
}

async function fetchSensors(sensorIds) {
    try {
        const response = await api.get('/sensors');
        sensors.value = response.data.sensors
            .filter(s => sensorIds.includes(s.id))
            .map(s => ({ ...s, is_running: false, streaming_url: `http://localhost:8000/stream/${s.id}` })); // Placeholder URL
    } catch (error) {
        Notify.create({ color: 'negative', message: error + ': Failed to load sensors.' });
    }
}

async function fetchRobots(robotIds) {
    try {
        const response = await api.get('/robots');
        robots.value = response.data.robots
            .filter(r => robotIds.includes(r.id))
            .map(r => ({ ...r, is_running: false }));
    } catch (error) {
        Notify.create({ color: 'negative', message: error + ': Failed to load robots.' });
    }
}

async function toggleSensor(sensor) {
    const action = sensor.is_running ? 'stop' : 'start';
    try {
        // await api.post(`/sensor/${sensor.id}/${action}`);
        sensor.is_running = !sensor.is_running;
        Notify.create({ color: 'positive', message: `Sensor ${sensor.name} ${action}ed.` });
    } catch (error) {
        Notify.create({ color: 'negative', message: error + `: Failed to ${action} sensor.` });
    }
}

async function toggleRobot(robot) {
    const action = robot.is_running ? 'stop' : 'start';
    try {
        // await api.post(`/robot/${robot.id}/${action}`);
        robot.is_running = !robot.is_running;
        Notify.create({ color: 'positive', message: `Robot ${robot.name} ${action}ed.` });
    } catch (error) {
        Notify.create({ color: 'negative', message: error + `: Failed to ${action} robot.` });
    }
}

async function startTest() {
    try {
        await api.post('/task:start_test', {
            task_id: taskId,
            checkpoint_id: selectedCheckpoint.value,
        });
        Notify.create({ color: 'positive', message: 'Test started successfully.' });
    } catch (error) {
        Notify.create({ color: 'negative', message: error + 'Failed to start test.' });
    }
}

async function stopTest() {
    try {
        await api.post('/task:stop_test', { task_id: taskId });
        Notify.create({ color: 'info', message: 'Test stopped.' });
    } catch (error) {
        Notify.create({ color: 'negative', message: error + ': Failed to stop test.' });
    }
}

function navigateStep() {
    if (step.value === 1) {
        if (!selectedCheckpoint.value) {
            Notify.create({ color: 'warning', message: 'Please select a checkpoint.' });
            return;
        }
        const all_running = sensors.value.every(s => s.is_running) && robots.value.every(r => r.is_running);
        if (!all_running) {
             $q.dialog({
                title: 'Confirm',
                message: 'Not all sensors and robots are running. Continue anyway?',
                cancel: true,
                persistent: true
            }).onOk(() => {
                startTest();
                stepper.value.next();
            })
        } else {
            startTest();
            stepper.value.next();
        }
    } else {
        stopTest();
        $q.router.push('/tasks');
    }
}

onMounted(() => {
    fetchTaskDetails();
    fetchCheckpoints();
});

</script>

<style scoped>
.q-stepper {
    background-color: #f5f5f5;
}
</style>