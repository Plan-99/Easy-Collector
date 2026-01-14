<template>
    <div class="bg-secondary border-rounded border-white column q-px-sm" style="max-height: 700px;">
        <div class="col-6 row flex felx-center" v-if="sensors.length > 0">
            <div v-for="sensor in sensors" :key="sensor.id" class="col q-py-sm q-px-xs relative-position">
                <web-rtc-video
                    :process-id="`sensor_${sensor.id}`"
                    :topic="sensor.topic"
                    class="full-height border-rounded cursor-pointer"
                    :key="sensor.id"
                    :loading="sensor.status !== 'on'"
                    v-if="sensor.status !== 'off'"
                    :class="{
                        'border-primary': focused.id === sensor.id && focused.device_type === 'sensor',
                    }"
                    @click="focusSensorRobot(sensor, 'sensor')"
                    :resize="[workspace.sensor_img_size[0], workspace.sensor_img_size[1]]"
                ></web-rtc-video>
                <div class="full-height border-white bg-dark border-rounded flex flex-center" v-else>
                    <q-btn round flat icon="play_arrow" text-color="white" size="xl" @click="sensor.handler.startSensor(sensor)"></q-btn>
                </div>
                <q-chip color="blue-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px">{{ sensor.name }} sensor</q-chip>
            </div>
        </div>
        <div v-else class="col-6 q-py-sm">
            <div class="text-white border-rounded border-white bg-dark full-height flex flex-center">No sensors available. Please add sensors to the workspace.</div>
        </div>
        <div class="col-5 row q-gutter-x-sm" v-if="robots.length > 0">
            <div v-for="robot in robots" :key="robot.id" class="col column q-pa-md relative-position border-rounded border-white text-white cursor-pointer"
                    :class="{
                        'border-primary': focused.id === robot.id && focused.device_type === 'robot',
                        'bg-dark': robot.status === 'off',
                    }"
                    @click="focusSensorRobot(robot, 'robot')"
                >
                <div v-for="(j, i) in robot.joint_names" :key="i" class="col flex flex-center q-gutter-x-md">
                    <div class="border-rounded border-white q-px-md q-py-xs text-center">{{ j }} {{ robot.jointState ? robot.jointState[i]?.toFixed(4) : 'Unreadable' }}</div>
                    <q-icon name="arrow_forward"></q-icon>
                    <div class="border-rounded border-primary q-px-md q-py-xs text-center text-primary">{{ j }} {{ robot.jointAction ? robot.jointAction[i]?.toFixed(4) : 'Unreadable' }}</div>
                </div>
                <q-btn
                    class="absolute-center q-mb-md q-mr-md"
                    round
                    flat
                    icon="play_arrow"
                    text-color="white"
                    size="xl"
                    @click="robot.handler.startRobot(robot)"
                    v-if="robot.status === 'off'"
                ></q-btn>
                <q-chip color="green-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px">{{ robot.name }} body</q-chip>
            </div>
        </div>
        <div v-else class="col-5 border-rounded border-white bg-dark flex flex-center">
            <div class="text-white">No robots available. Please add robots to the workspace.</div>
        </div>
        <div class="flex flex-center col" v-if="!isRobotSensorAllOn">
            <div class="text-yellow">Start all sensors and robots to view live data streams.</div>
        </div>
        <div class="col q-py-sm" v-else-if="!checkpoint">
            <div class="text-grey bg-dark border-rounded row full-height flex flex-center" v-if="status === 'pending'">
                <q-select
                    v-model="selectedDatasetId"
                    dense
                    outlined
                    dark
                    bg-color="dark"
                    label="Select Dataset for Data Collection"
                    style="width: 200px"
                    :options="datasets"
                    option-label="name"
                    option-value="id"
                    map-options
                    emit-value
                ></q-select>
                <q-space></q-space>
                <div class="q-mr-md">Teleoparation Type: </div>
                <div class="q-gutter-sm q-mr-xl">
                    <q-radio dark v-model="teleType" val="leader" :label="$t('leaderTele')" />
                    <q-radio dark v-model="teleType" val="keyboard" :label="$t('keyboardTele')" />
                    <q-radio dark v-model="teleType" val="externel" :label="$t('externalTele')" />
                </div>
                <q-btn
                    color="red"
                    icon="fiber_manual_record"
                    text-color="white"
                    label="REC"
                    @click="startDataCollection"
                ></q-btn>
            </div>
            <div class="row flex flex-center full-height" v-else>
                <div class="col q-pr-md">
                    <q-linear-progress
                        :value="collectingProgress"
                        color="primary"
                        track-color="black"
                        size="30px"
                        instant-feedback
                    >
                        <div class="absolute-full flex flex-center">
                            <q-badge color="white" text-color="dark" :label="`${Number(collectingProgress * 100).toFixed(0)}%`" />
                        </div>
                    </q-linear-progress>
                </div>

                <q-btn
                    color="white"
                    text-color="red"
                    label="STOP"
                    icon="stop"
                    @click="stopDataCollection"
                ></q-btn>
            </div>
        </div>
        <div v-else class="col">
            <div
                class="col q-pa-sm bg-dark border-rounded text-white row flex flex-center"
                v-if="status === 'pending'"
            >
                <div>
                    <div class="text-h6">{{ checkpoint?.name }}</div>
                </div>
                <div>
                    <q-btn
                        icon="close"
                        round
                        flat
                        @click="selectedCheckpointId = null"
                    ></q-btn>
                </div>
                <q-space class="col"></q-space>
                <div>
                    <q-btn
                        color="red"
                        text-color="white"
                        label="Start Inference"
                        icon="play_arrow"
                        @click="startInference"
                        v-if="status === 'pending'"
                    ></q-btn>
                </div>
            </div>
            <div
                class="col q-pa-sm bg-dark border-rounded text-white row flex flex-center"
                v-else
            >
                <q-space></q-space>
                <q-btn
                    color="white"
                    text-color="red"
                    label="Stop Inference"
                    icon="stop"
                    @click="stopInference"
                ></q-btn>
            </div>
        </div>
        <div style="position: fixed; bottom: 20px; left: 20px; z-index: 1000;">
            <q-btn
                push
                color="white"
                label="Terminal"
                text-color="dark"
                @click="showProcessConsole = !showProcessConsole"
                style=""
            />
            <process-console
                process="record_episode"
                class="q-mt-md"
                style="border: 1px solid #ffffff; background-color: #1e1e1e; z-index: 1000; width: 800px; height: 400px;"
                v-show="showProcessConsole"
            >
            </process-console> 
        </div>
    </div>
</template>

<script setup>
import { defineProps, ref, computed, defineModel, onMounted } from 'vue';
import { Notify } from 'quasar';
import { api } from 'src/boot/axios';
import ProcessConsole from './ProcessConsole.vue';
import { useSocket } from 'src/composables/useSocket.js';
import WebRtcVideo from './WebRtcVideo.vue';

const { socket } = useSocket();

const props = defineProps({
    workspace: {
        type: Object,
        required: true
    },
    robots: {
        type: Array,
        required: true
    },
    sensors: {
        type: Array,
        required: true
    },
    checkpoints: {
        type: Array,
        required: true
    },
    status: {
        type: String,
        required: true
    },
    datasets: {
        type: Array,
        required: true
    },
});

const focused = defineModel('focused', {
    type: Object,
    default: () => ({ id: null, device_type: null })
});
const selectedDatasetId = defineModel('selectedDatasetId', {
    type: [String, Number],
    default: null
});
const selectedCheckpointId = defineModel('selectedCheckpointId', {
    type: [String, Number],
    default: null
});
const checkpoint = computed(() => {
    return props.checkpoints?.find(c => c.id === selectedCheckpointId.value);
});

const teleType = ref('leader')
const isRobotSensorAllOn = computed(() => {
    const allRobotsOn = props.robots.every(r => r.status === 'on');
    const allSensorsOn = props.sensors.every(s => s.status === 'on');
    return allRobotsOn && allSensorsOn;
});

function focusSensorRobot(device, type) {
    if (device.status !== 'on') {
        return;
    }
    if (focused.value && focused.value.id === device.id && focused.value.device_type === type) {
        focused.value = {};
        return;
    }

    focused.value = {
        ...device, // device 객체의 모든 속성을 복사
        device_type: type // device_type 속성을 추가 (또는 덮어쓰기)
    };
}


const collectingProgress = ref(0);
const showProcessConsole = ref(false);

function startDataCollection() {
    if (!selectedDatasetId.value) {
        Notify.create({
            color: 'negative',
            message: 'Please select a dataset for data collection'
        });
        return;
    }
    if (teleType.value === 'keyboard') {
        addKeyboardListener();
    }
    showProcessConsole.value = true;
    collectingProgress.value = 0;
    api.post(`/dataset/${selectedDatasetId.value}/:start_collection`, {
        task: props.workspace,
        robots: props.robots,
        sensors: props.sensors,
        tele_type: teleType.value,
        assembly_id: props.workspace.assembly_id,
    }).catch((error) => {
        console.error('Error starting data collection:', error);
        Notify.create({
            color: 'negative',
            message: 'Error starting data collection'
        });
    });
}

const eeStepSize = ref(0.003);

const keyboardHandler = (event) => {
    if (focused.value?.device_type !== 'robot') return;
    if (focused.value.role === 'dual_arm') return;

    // [중요] focused 객체의 복사본이 아닌, props의 최신 로봇 데이터를 조회
    const currentRobot = props.robots.find(r => r.id === focused.value.id);
    if (!currentRobot || !currentRobot.eePos) return;

    const eeDelta = [0, 0, 0, 0, 0, 0, 0]; // [x, y, z, roll, pitch, yaw, tool]
    if (event.key === 'w') eeDelta[0] += eeStepSize.value;
    else if (event.key === 's') eeDelta[0] -= eeStepSize.value;
    else if (event.key === 'a') eeDelta[1] += eeStepSize.value;
    else if (event.key === 'd') eeDelta[1] -= eeStepSize.value;
    else if (event.key === 'e') eeDelta[2] += eeStepSize.value;
    else if (event.key === 'z') eeDelta[2] -= eeStepSize.value;
    else if (event.key === '[') eeDelta[3] += eeStepSize.value*4;
    else if (event.key === '.') eeDelta[3] -= eeStepSize.value*4;
    else if (event.key === 'p') eeDelta[4] += eeStepSize.value*4;
    else if (event.key === ';') eeDelta[4] -= eeStepSize.value*4;
    else if (event.key === 'l') eeDelta[5] += eeStepSize.value*4;
    else if (event.key === ',') eeDelta[5] -= eeStepSize.value*4;
    else if (event.key === 'b') eeDelta[6] += eeStepSize.value*30;
    else if (event.key === 'n') eeDelta[6] -= eeStepSize.value*30;
    else return;

    // 백엔드로 전송
    currentRobot.handler.moveRobotEEDelta({
        ee: eeDelta
    });
};

function addKeyboardListener() {
    // 중복 등록 방지를 위해 먼저 제거 후 등록
    window.removeEventListener('keydown', keyboardHandler);
    window.addEventListener('keydown', keyboardHandler);
}

function removeKeyboardListener() {
    window.removeEventListener('keydown', keyboardHandler);
}

function stopDataCollection() {
    if (teleType.value === 'keyboard') {
        removeKeyboardListener();
    }
    api.post(`/dataset/${selectedDatasetId.value}/:stop_collection`).then(() => {
        collectingProgress.value = 0;
    })
}

function startInference() {
    showProcessConsole.value = true;
    api.post(`/checkpoint/${selectedCheckpointId.value}/:start_test`, {
        task: props.workspace,
        policy: checkpoint.value.policy,
        robot_ids: props.robots.map(r => r.id),
        sensors: props.sensors,
        checkpoint: checkpoint.value,
    }).catch((error) => {
        console.error('Error starting test:', error);
        Notify.create({
            color: 'negative',
            message: 'Error starting test'
        });
    });
}

function stopInference() {
    return api.post(`/checkpoint/${selectedCheckpointId.value}/:stop_test`).catch((error) => {
        console.error('Error stopping test:', error);
        Notify.create({
            color: 'negative',
            message: 'Error stopping test'
        });
    });
}

onMounted(() => {

    socket.on('stop_process', (data) => {
        if (data.id === 'record_episode') {
            collectingProgress.value = 0;
        }
        if (data.id === 'checkpoint_test') {
            Notify.create({
                color: 'positive',
                message: 'Inference stopped'
            });
        }
    });

    socket.on('record_episode_progress', (data) => {
        collectingProgress.value = data.progress;
    });
});
</script>