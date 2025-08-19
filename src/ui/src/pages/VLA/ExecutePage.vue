<template>

    <div class="bg-white column text-white">
        <div class="row col">
            <div class="column col-md-3 q-pa-md">
                <q-select
                    v-model="vlaSettings.model"
                    dense
                    :options="checkpoints"
                    map-options emit-value option-label="name" option-value="id"
                    label="Select Model"
                    class="q-mb-md full-width" />
                <q-select
                    v-model="vlaSettings.robot_ids"
                    dense
                    :options="robots"
                    label="Select Robots"
                    multiple
                    map-options emit-value option-label="name" option-value="id" 
                    class="q-mb-md full-width" />
                <q-select
                    v-model="vlaSettings.sensor_ids"
                    dense
                    :options="sensors"
                    label="Select Sensors"
                    multiple
                    map-options emit-value option-label="name" option-value="id" 
                    class="q-mb-md full-width" />
                <div class="row q-mb-md">
                    <q-input
                        v-model.number="vlaSettings.image_width"
                        dense
                        type="number"
                        label="Image Width"
                        class="q-mb-md col" />
                    <q-input
                        v-model.number="vlaSettings.image_height"
                        dense
                        type="number"
                        label="Image Height"
                        class="q-mb-md col" />
                </div>
            </div>
            <div
                v-for="sensor in sensors.filter(e => vlaSettings.sensor_ids.includes(e.id))"
                :key="sensor.id"
                class="full-width full-height col"
            >
                <web-rtc-video
                    :process-id="`sensor_${sensor.id}`"
                    style="width: 400px; height: 300px;"
                    :topic="sensor.topic"
                    class="full-width"
                    v-if="sensor.handler.status() !== 'off'"
                    :resize="[vlaSettings.image_width, vlaSettings.image_height]"
                    :loading="sensor.status !== 'on'"
                ></web-rtc-video>
                <div
                    class="full-width full-height text-center q-pa-md bg-grey-8 flex flex-center"
                    style="border-right: 1px solid #ffffff;"
                    v-else
                >
                    <q-btn round flat icon="play_arrow" size="xl" @click="sensor.handler.startSensor(sensor)"></q-btn>
                </div>
            </div>
        </div>
        <div class="row col bg-grey-9">
            <div class="col-2 q-pa-md">
                <div v-for="robot in robots.filter(e => vlaSettings.robot_ids.includes(e.id))" :key="robot.id" class="q-mb-md">
                    <q-btn color="grey-7 full-width" icon="power_settings_new" v-if="robot.handler.status() === 'off'" @click="robot.handler.startRobot()">
                        {{ robot.name }}
                        <q-tooltip class="bg-primary text-body2">Start robot first to set home pose and end pose</q-tooltip>
                    </q-btn>
                    <q-btn color="orange full-width" icon="power_settings_new" v-if="robot.handler.status() === 'loading'">
                        {{ robot.name }}
                    </q-btn>
                    <q-btn color="green full-width" icon="power_settings_new" v-if="robot.handler.status() === 'on'" @click="robot.handler.stopRobot()">
                        {{ robot.name }}
                    </q-btn>
                </div>
            </div>
            <div class="col-10 q-pa-md">
                <div v-if="
                    robots.filter(e => vlaSettings.robot_ids.includes(e.id)).find((e) => e.handler.status() === 'off' || e.handler.status() === 'loading') 
                    || sensors.filter(e => vlaSettings.sensor_ids.includes(e.id)).find((e) => e.handler.status() === 'off' || e.handler.status() === 'loading')"
                    class="flex flex-center full-height"
                    style="border: 1px solid #ffffff;"
                >
                    Start all robots and sensors to start test
                </div>
                <div v-else class="text-center">
                    <div v-if="!testing" class="row">
                        <div class="col-10">
                            <q-input
                                v-model="prompt"
                                dense
                                outline
                                color="primary"
                                class="col-10 q-mr-lg bg-white"
                            />
                        </div>
                        <q-btn class="col-2 bg-white" outline color="primary" @click="testVLA">Send</q-btn>
                    </div>
                    <div v-else>
                        <div class="q-mb-md">
                            <q-linear-progress size="25px" instant-feedback :value="testingProgress" color="accent">
                                <div class="absolute-full flex flex-center">
                                    <q-badge color="white" text-color="accent" :label="`${Number(testingProgress * 100).toFixed(0)}%`" />
                                </div>
                            </q-linear-progress>
                        </div>
                        <q-btn 
                            class="full-width bg-white" 
                            outline 
                            color="orange-8" 
                            @click="stopTest" 
                        >Stop</q-btn>
                    </div>
                    <process-console
                        process="checkpoint_test"
                        class="full-width q-mt-md"
                        style="border: 1px solid #ffffff;"                                
                    >
                    </process-console> 
                </div>
            </div>
        </div>
    </div>
</template>

<script setup>
import { onMounted, ref, watch } from 'vue';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';
import { useRobot } from 'src/composables/useRobot.js';
import { useSensor } from 'src/composables/useSensor.js';
import WebRtcVideo from 'src/components/WebRtcVideo.vue';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Notify } from 'quasar';
import { useSocket } from 'src/composables/useSocket.js';

const route = useRoute();
const { socket } = useSocket();

const testing = ref(false);

const robots = ref([]);
const sensors = ref([]);

const checkpoints = ref([]);

const prompt = ref(null);
const policies = ref([]);
const selectedPolicy = ref(null);
const selectedCheckpoint = ref(null);

const vlaSettings = ref({
    model: null,
    robot_ids: [],
    sensor_ids: [],
    image_width: 640,
    image_height: 480
});

function listRobots() {
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.handler = useRobot(robot);
        });
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

function listSensors() {
    return api.get('/sensors').then((response) => {
        sensors.value = response.data.sensors || [];
        sensors.value.forEach(sensor => {
            sensor.handler = useSensor(sensor);
        });
    }).catch((error) => {
        console.error('Error fetching sensors:', error);
    });
}

function listCheckpoints() {
    return api.get('/checkpoints').then((response) => {
        checkpoints.value = response.data.checkpoints.filter(e => e.is_vla === 1)
    }).catch((error) => {
        console.error('Error fetching checkpoints:', error);
    });
}

function listPolicies() {
    return api.get('/policies').then((response) => {
        policies.value = response.data.policies || [];
    }).catch((error) => {
        console.error('Error fetching policies:', error);
    });
}

watch(() => vlaSettings.value.model, (newModel) => {
    if (newModel) {
        selectedCheckpoint.value = checkpoints.value.find(checkpoint => checkpoint.id === newModel);
        listPolicies().then(() => {
            selectedPolicy.value = policies.value.find(policy => policy.id === selectedCheckpoint.value.policy_id);
        })
    }
}, { immediate: true });


function testVLA() {
    api.post(`/vla:test_vla`, {
        model: selectedCheckpoint.value,
        policy: selectedPolicy.value,
        robots: robots.value.filter(e => vlaSettings.value.robot_ids.includes(e.id)),
        sensors: sensors.value.filter(e => vlaSettings.value.sensor_ids.includes(e.id)),
        image_size: [vlaSettings.value.image_width, vlaSettings.value.image_height],
        prompt: prompt.value
    }).then(() => {
        testing.value = true;
        Notify.create({
            color: 'positive',
            message: 'Test started'
        });
    }).catch((error) => {
        console.error('Error starting test:', error);
        Notify.create({
            color: 'negative',
            message: 'Error starting test'
        });
    });
}

// function stopTest() {
//     api.post(`/vla/:stop_test`).then(() => {
//         testing.value = false;
//         Notify.create({
//             color: 'positive',
//             message: 'Test stopped'
//         });
//         testingProgress.value = 0;
//     }).catch((error) => {
//         console.error('Error stopping test:', error);
//         Notify.create({
//             color: 'negative',
//             message: 'Error stopping test'
//         });
//     });
// }

onMounted(() => {
    listRobots();
    listSensors();
    listCheckpoints();
});
</script>