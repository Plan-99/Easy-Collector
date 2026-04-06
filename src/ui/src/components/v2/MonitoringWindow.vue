<template>
    <div class="bg-secondary border-rounded border-white column q-px-sm" style="max-height: 700px;">
        <div class="col-6 row flex felx-center q-col-gutter-x-sm" v-if="sensors.length > 0">
            <div v-for="sensor in sensors" :key="sensor.id" class="col q-py-sm relative-position">
                <web-rtc-video
                    :process-id="`sensor_${sensor.id}`"
                    :topic="sensor.read_topic"
                    :msg-type="sensor.read_topic_msg"
                    :sensor-id="sensor.id"
                    class="full-height border-rounded cursor-pointer"
                    :key="sensor.id"
                    :loading="sensor.status !== 'on'"
                    v-if="sensor.status !== 'off'"
                    :class="{
                        'border-primary': focused.id === sensor.id && focused.device_type === 'sensor',
                    }"
                    @click="focusSensorRobot(sensor, 'sensor')"
                    :resize="workspace.sensor_img_size[sensor.id] || [640, 480]"
                    :cropped_area="workspace.sensor_cropped_area[sensor.id] || {}"
                    :rotate="workspace.sensor_rotate[sensor.id] || 0"
                ></web-rtc-video>
                <div class="full-height border-white bg-dark border-rounded flex flex-center" v-else>
                    <q-btn round flat icon="play_arrow" text-color="white" size="xl" @click="sensor.handler.startSensor(sensor)"></q-btn>
                </div>
                <q-chip color="blue-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px">{{ sensor.name }} {{ $t('sensorSuffix') }}</q-chip>
            </div>
        </div>
        <div v-else class="col-6 q-py-sm">
            <div class="text-white border-rounded border-white bg-dark full-height flex flex-center">{{ $t('noSensorsMsg') }}</div>
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
                    <div class="border-rounded border-white q-px-md q-py-xs text-center">{{ j }} {{ robot.jointState ? robot.jointState[i]?.toFixed(4) : $t('unreadable') }}</div>
                    <q-icon name="arrow_forward"></q-icon>
                    <div class="border-rounded border-primary q-px-md q-py-xs text-center text-primary">{{ j }} {{ robot.jointAction ? robot.jointAction[i]?.toFixed(4) : $t('unreadable') }}</div>
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
                <q-chip color="green-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px">{{ robot.name }} {{ $t('robotSuffix') }}</q-chip>
            </div>
        </div>
        <div v-else class="col-5 border-rounded border-white bg-dark flex flex-center">
            <div class="text-white">{{ $t('noRobotsMsg') }}</div>
        </div>
        <div class="flex flex-center col" v-if="!isRobotSensorAllOn">
            <div class="text-yellow">{{ $t('startAllDevices') }}</div>
        </div>
        <div class="col q-py-sm" v-else>
            <div v-if="selectedEpisode.name && selectedDatasetId">
                <div class="row q-mb-sm">
                    <div
                        class="col bg-dark border-rounded text-white row items-center q-col-gutter-x-sm"
                    >
                        <div>
                            <div class="text-h6">{{ selectedEpisode?.name }}</div>
                        </div>
                        <div>
                            <q-btn
                                icon="close"
                                round
                                flat
                                @click="selectedEpisode = {}"
                                v-if="status === 'pending'"
                            ></q-btn>
                        </div>
                        <q-space class="col"></q-space>
                        <div class="row items-center q-gutter-x-sm" v-if="status === 'pending'">
                            <span class="text-caption text-grey">{{ $t('replayActionType') }}:</span>
                            <q-radio
                                v-model="replayActionType"
                                val="qaction"
                                :label="$t('replayActionQaction')"
                                dense dark color="primary"
                            />
                            <q-radio
                                v-model="replayActionType"
                                val="ee_delta_action"
                                :label="$t('replayActionEeDelta')"
                                dense dark color="primary"
                            />
                            <q-checkbox
                                v-model="replayCapture"
                                label="Capture Episode"
                                dark dense
                                :disable="!allSensorsOn"
                            >
                                <q-tooltip v-if="!allSensorsOn">All cameras must be on to capture</q-tooltip>
                            </q-checkbox>
                            <q-select
                                v-if="replayCapture"
                                v-model="replayCaptureDatasetId"
                                dense outlined dark bg-color="dark"
                                label="Target Dataset"
                                style="min-width: 180px"
                                :options="datasets"
                                option-label="name"
                                option-value="id"
                                map-options emit-value
                            />
                            <q-input
                                v-model.number="replayHz"
                                dense outlined dark bg-color="dark"
                                label="Hz"
                                style="width: 80px"
                                type="number"
                                :min="1"
                            />
                        </div>
                        <div>
                            <q-btn
                                color="red"
                                text-color="white"
                                :label="$t('replayPlay')"
                                icon="play_arrow"
                                @click="startReplay"
                                v-if="status === 'pending'"
                            ></q-btn>
                            <q-btn
                                color="white"
                                text-color="red"
                                :label="$t('replayStop')"
                                icon="stop"
                                @click="stopReplay"
                                v-else
                            ></q-btn>
                        </div>
                    </div>
                </div>
                <div class="row q-mb-sm" v-if="replayCapture && status !== 'pending'">
                    <div class="col">
                        <q-linear-progress
                            :value="replayProgress"
                            color="primary"
                            track-color="black"
                            size="30px"
                            instant-feedback
                        >
                            <div class="absolute-full flex flex-center">
                                <q-badge color="white" text-color="dark" :label="`Capturing ${Number(replayProgress * 100).toFixed(0)}%`" />
                            </div>
                        </q-linear-progress>
                    </div>
                </div>
            </div>
            <div v-else-if="checkpoint" class="row q-mb-sm">
                <div
                    class="col bg-dark border-rounded text-white row flex flex-center q-col-gutter-x-sm"
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
                        <q-input
                            v-model.number="hz"
                            dense
                            outlined
                            dark
                            bg-color="dark"
                            :label="$t('frequencyHz')"
                        >
                        </q-input>
                    </div>
                    <div>
                        <q-btn
                            color="red"
                            text-color="white"
                            :label="$t('startInference')"
                            icon="play_arrow"
                            @click="startInference"
                            v-if="status === 'pending'"
                        >
                            <q-badge 
                                @click.stop="moveHomposeInDataCollection = !moveHomposeInDataCollection" 
                                :color="moveHomposeInDataCollection ? 'blue' : 'grey-5'"
                                floating>
                                <q-icon name="home" size="xs" class="cursor-pointer" />
                            </q-badge>
                        </q-btn>
                    </div>
                </div>
                <div
                    class="col q-pa-sm bg-dark border-rounded text-white row flex flex-center"
                    v-else
                >
                    <q-space></q-space>
                    <q-badge
                        v-if="succeedScore !== null"
                        :color="succeedScore > 0.7 ? 'green' : 'red'"
                        :label="`Succeed: ${succeedScore.toFixed(2)}`"
                        class="q-mr-sm q-pa-sm text-bold"
                        outline
                    />
                    <q-badge
                        v-if="oodScore !== null"
                        :color="oodTotal > 1.0 ? 'red' : 'orange'"
                        class="q-mr-sm q-pa-sm text-bold"
                        outline
                    >
                        OOD: {{ oodScoreDisplay }}
                    </q-badge>
                    <q-btn
                        color="white"
                        text-color="red"
                        label="Stop"
                        icon="stop"
                        @click="stopInference"
                    ></q-btn>
                </div>
            </div>
            <div v-else>
                <div class="text-grey bg-dark border-rounded row full-height flex flex-center" v-if="status === 'pending'">
                    <q-select
                        v-model="selectedDatasetId"
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        :label="$t('selectDataset')"
                        style="width: 200px"
                        :options="datasets"
                        option-label="name"
                        option-value="id"
                        map-options
                        emit-value
                    ></q-select>
                    <q-input
                        v-model="languageInstruction"
                        dense outlined dark bg-color="dark"
                        label="Language Instruction"
                        style="min-width: 200px; max-width: 400px;"
                        class="q-ml-sm"
                        clearable
                    />
                    <q-space></q-space>
                    <div class="row items-center q-mr-xl q-gutter-x-sm">
                        <q-select
                            dense outlined dark bg-color="dark"
                            v-model="teleType"
                            :options="teleTypeOptions"
                            map-options emit-value
                            style="min-width: 220px;"
                            label="Teleoperation Type"
                        />
                        <q-input
                            v-model.number="collectionHz"
                            dense outlined dark bg-color="dark"
                            label="Hz"
                            style="width: 80px"
                            type="number"
                            :min="1"
                        />
                        <q-btn
                            v-if="teleType === 'motion_planning'"
                            icon="settings"
                            round flat size="sm"
                            color="primary"
                            @click="showRos2ServiceDialog = true"
                        >
                            <q-tooltip>Configure ROS2 Service</q-tooltip>
                        </q-btn>
                    </div>
                    <q-btn
                        color="red"
                        icon="fiber_manual_record"
                        text-color="white"
                        :label="$t('rec')"
                        @click="startDataCollection"
                    >
                        <q-badge
                            @click.stop="moveHomposeInDataCollection = !moveHomposeInDataCollection"
                            :color="moveHomposeInDataCollection ? 'blue' : 'grey-5'"
                            floating>
                            <q-icon name="home" size="xs" class="cursor-pointer" />
                        </q-badge>
                    </q-btn>
                </div>
                <div class="row flex flex-center full-height" v-else-if="viveInitializing">
                    <q-spinner-dots color="primary" size="2em" class="q-mr-md" />
                    <div class="text-white q-mr-lg">{{ $t('viveWaiting') }}</div>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('cancel')"
                        icon="close"
                        @click="cancelViveInit"
                    ></q-btn>
                </div>
                <div class="row flex flex-center full-height" v-else-if="movingHomepose">
                    <q-spinner-dots color="primary" size="2em" class="q-mr-md" />
                    <div class="text-white q-mr-lg">{{ $t('movingToHomepose') }}</div>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('stopCollection')"
                        icon="stop"
                        @click="stopDataCollection"
                    ></q-btn>
                </div>
                <div class="row flex flex-center full-height" v-else>
                    <q-input
                        v-if="teleType === 'keyboard'"
                        v-model.number="eeStepSize"
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        label="Step Size"
                        type="number"
                        step="0.0001"
                        style="width: 140px"
                        class="q-mr-md"
                    />
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
                        :color="succeedFlag ? 'yellow' : 'blue-grey'"
                        text-color="white"
                        label="SUCCESS (C)"
                        icon="emoji_events"
                        @click="setSucceed"
                        :disable="succeedFlag"
                        class="q-mr-sm"
                    ></q-btn>
                    <q-btn
                        color="green"
                        text-color="white"
                        :label="$t('completeEpisode')"
                        icon="check"
                        @click="completeEpisode"
                        class="q-mr-sm"
                    ></q-btn>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('stopCollection')"
                        icon="stop"
                        @click="stopDataCollection"
                    ></q-btn>
                </div>
            </div>
        </div>

        <!-- Vive 모드 선택 다이얼로그 (v-if 체인 외부) -->
        <q-dialog v-model="viveRobotDialog" persistent>
            <q-card dark style="min-width: 360px">
                <q-card-section>
                    <div class="text-h6">{{ $t('viveRobotDialogTitle') }}</div>
                    <div class="text-body2 q-mt-sm text-grey-4">{{ $t('viveRobotDialogMsg') }}</div>
                </q-card-section>
                <q-card-actions align="center" class="q-pb-md q-gutter-sm">
                    <q-btn
                        color="primary"
                        outline
                        :label="$t('viveWithRobot')"
                        icon="smart_toy"
                        @click="confirmViveMode('vive_external')"
                    />
                    <q-btn
                        color="white"
                        outline
                        :label="$t('viveWithoutRobot')"
                        icon="videocam"
                        @click="confirmViveMode('vive_only')"
                    />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <div style="position: fixed; bottom: 20px; left: 20px; z-index: 1000;">
            <q-btn
                push
                color="white"
                :label="$t('terminal')"
                text-color="dark"
                @click="showProcessConsole = !showProcessConsole"
                style=""
            />
            <process-console
                :process="status === 'testing' ? 'checkpoint_test' : 'record_episode'"
                class="q-mt-md"
                style="border: 1px solid #ffffff; background-color: #1e1e1e; z-index: 1000; width: 800px; height: 400px;"
                v-show="showProcessConsole"
            >
            </process-console>
        </div>
        <div style="position: fixed; top: 10px; right: 20px; z-index: 10000; width: 600px" 
                v-if="selectedEpisode.name && selectedDatasetId"
                class="border-white border-rounded bg-dark p-md relative-position"
        >
            <q-btn
                class="absolute-top-right"
                icon="close"
                round
                flat
                @click="selectedEpisode = {}"
                color="white"
                v-if="status === 'pending'"
            ></q-btn>
            <hdf5-viewer
                :path="`${selectedDatasetId}/${selectedEpisode.name}`"
            ></hdf5-viewer>
        </div>
        <!-- Inference Settings Dialog -->
        <form-dialog
            v-model="showInferenceDialog"
            :title="$t('inferenceSettings')"
            :form="inferenceForm"
            :ok-button-label="$t('startInference')"
            min-width="380px"
            @submit="onInferenceSubmit"
        />

        <!-- ROS2 Service Dialog -->
        <q-dialog v-model="showRos2ServiceDialog">
            <q-card style="min-width: 550px;" class="bg-secondary text-white">
                <q-card-section>
                    <div class="text-h6">ROS2 Service (std_srvs/Trigger)</div>
                </q-card-section>
                <q-card-section>
                    <q-input
                        dense outlined dark bg-color="dark"
                        v-model="ros2Service"
                        label="Service Name"
                        placeholder="e.g. /pick_and_place"
                        hint="Service name only (e.g. /pick_and_place), not the full ros2 command"
                    />
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat label="Cancel" color="grey" v-close-popup />
                    <q-btn flat label="Save" color="primary" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </div>
</template>

<script setup>
import { defineProps, ref, computed, defineModel, onMounted, onUnmounted } from 'vue';
import { Notify } from 'quasar';
import { useI18n } from 'vue-i18n';
import { api } from 'src/boot/axios';
import ProcessConsole from './ProcessConsole.vue';
import { useSocket } from 'src/composables/useSocket.js';
import WebRtcVideo from './WebRtcVideo.vue';
import Hdf5Viewer from 'src/components/v2/Hdf5Viewer.vue';
import FormDialog from './FormDialog.vue';


const { socket } = useSocket();
const { t } = useI18n();

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
const selectedEpisode = defineModel('selectedEpisode', {
    type: Object,
    default: {}
});
const replayActionType = ref('qaction');
const checkpoint = computed(() => {
    return props.checkpoints?.find(c => c.id === selectedCheckpointId.value);
});

const teleType = ref('leader')
const collectionHz = ref(20)
const languageInstruction = ref('')
const viveInitializing = ref(false)
const viveRobotDialog = ref(false)


const teleTypeOptions = [
    { label: 'Easy Controller', value: 'leader' },
    { label: 'Keyboard', value: 'keyboard' },
    { label: 'External', value: 'external' },
    { label: 'Vive', value: 'vive_external' },
    { label: 'External + Motion Planning', value: 'motion_planning' },
]

const replayCapture = ref(false)
const replayCaptureDatasetId = ref(null)
const replayHz = ref(5)
const replayProgress = ref(0)

const showRos2ServiceDialog = ref(false)
const ros2Service = ref('')
const allSensorsOn = computed(() => {
    return props.sensors.every(s => s.status === 'on');
});

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

const moveHomposeInDataCollection = ref(false);
const movingHomepose = ref(false);
const succeedFlag = ref(false);
const inferenceSucceed = ref(false);
const succeedScore = ref(null);
const oodScore = ref(null);
const oodTotal = computed(() => {
    if (!oodScore.value) return 0;
    const img = oodScore.value.image;
    const state = oodScore.value.state;
    if (typeof img === 'number' && typeof state === 'number') return (img + state) / 2;
    if (typeof img === 'number') return img;
    if (typeof state === 'number') return state;
    return 0;
});
const oodScoreDisplay = computed(() => {
    if (!oodScore.value) return '';
    const img = oodScore.value.image;
    const state = oodScore.value.state;
    const imgStr = typeof img === 'number' ? img.toFixed(2) : '-';
    const stateStr = typeof state === 'number' ? state.toFixed(2) : '-';
    return `${oodTotal.value.toFixed(2)} (${imgStr} + ${stateStr})`;
});

function startDataCollection() {
    if (!selectedDatasetId.value) {
        Notify.create({
            color: 'negative',
            message: t('selectDatasetRequired')
        });
        return;
    }
    // vive_external 선택 시: 실물 로봇 여부를 다이얼로그로 확인
    if (teleType.value === 'vive_external') {
        viveRobotDialog.value = true;
        return;
    }
    _doStartDataCollection(teleType.value);
}

function confirmViveMode(effectiveTeleType) {
    viveRobotDialog.value = false;
    _doStartDataCollection(effectiveTeleType);
}

function _doStartDataCollection(effectiveTeleType) {
    if (effectiveTeleType === 'keyboard') {
        addKeyboardListener();
    }
    if (effectiveTeleType === 'vive_external' || effectiveTeleType === 'vive_only') {
        viveInitializing.value = true;
    }
    succeedFlag.value = false;
    addSucceedKeyListener();
    showProcessConsole.value = true;
    collectingProgress.value = 0;
    const payload = {
        task: props.workspace,
        robots: props.robots,
        sensors: props.sensors,
        tele_type: effectiveTeleType,
        assembly_id: props.workspace.assembly_id,
        move_homepose: moveHomposeInDataCollection.value,
        hz: collectionHz.value,
        language_instruction: languageInstruction.value || '',
    };
    if (teleType.value === 'motion_planning' && ros2Service.value) {
        payload.ros2_service = ros2Service.value;
    }
    api.post(`/dataset/${selectedDatasetId.value}/:start_collection`, payload).catch((error) => {
        viveInitializing.value = false;
        console.error('Error starting data collection:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStartCollection')
        });
    });
}

function cancelViveInit() {
    viveInitializing.value = false;
    api.post(`/dataset/${selectedDatasetId.value}/:stop_collection`).then(() => {
        collectingProgress.value = 0;
    });
}

const eeStepSize = ref(0.0005);

const keyboardHandler = (event) => {
    if (focused.value?.device_type !== 'robot') return;

    const currentRobot = props.robots.find(r => r.id === focused.value.id);
    if (!currentRobot || !currentRobot.role === 'dual_arm') return;

    if (currentRobot.role === 'single_arm') {
        const eeDelta = Array(currentRobot.tool_inner ? 7 : 6).fill(0); // x, y, z, roll, pitch, yaw, tool

        if (event.key === 'w') eeDelta[0] += eeStepSize.value;
        else if (event.key === 's') eeDelta[0] -= eeStepSize.value;
        else if (event.key === 'a') eeDelta[1] += eeStepSize.value;
        else if (event.key === 'd') eeDelta[1] -= eeStepSize.value;
        else if (event.key === 'e') eeDelta[2] += eeStepSize.value;
        else if (event.key === 'z') eeDelta[2] -= eeStepSize.value;
        else if (event.key === '.') eeDelta[3] += eeStepSize.value*20;
        else if (event.key === '[') eeDelta[3] -= eeStepSize.value*20;
        else if (event.key === 'p') eeDelta[4] += eeStepSize.value*20;
        else if (event.key === ';') eeDelta[4] -= eeStepSize.value*20;
        else if (event.key === 'l') eeDelta[5] += eeStepSize.value*20;
        else if (event.key === "'") eeDelta[5] -= eeStepSize.value*20;
        else if (event.key === 'b' && currentRobot.tool_inner) eeDelta[6] += eeStepSize.value*30;
        else if (event.key === 'n' && currentRobot.tool_inner) eeDelta[6] -= eeStepSize.value*30;
        else if (event.key === ' ' || event.key === 32) eeDelta.fill(0); // stop
        else return;

        // 백엔드로 전송
        currentRobot.handler.moveRobotEEDelta({
            ee: eeDelta
        });
    } else if (currentRobot.role === 'tool') {
        console.log('tool teleop');
        const toolDelta = Array(1).fill(0); // tool only

        if (event.key === 'b') toolDelta[0] += 0.12;
        else if (event.key === 'n') toolDelta[0] -= 0.12;
        else return;
        console.log('toolDelta', toolDelta);
        // 백엔드로 전송
        currentRobot.handler.moveRobotJointDelta(toolDelta);
    }

};

function addKeyboardListener() {
    // 중복 등록 방지를 위해 먼저 제거 후 등록
    window.removeEventListener('keydown', keyboardHandler);
    window.addEventListener('keydown', keyboardHandler);
}

function removeKeyboardListener() {
    window.removeEventListener('keydown', keyboardHandler);
}

function completeEpisode() {
    api.post(`/dataset/${selectedDatasetId.value}/:complete_episode`).catch((error) => {
        console.error('Error completing episode:', error);
        Notify.create({
            color: 'negative',
            message: t('errorCompleteEpisode'),
        });
    });
}

function setSucceed() {
    if (succeedFlag.value) return;
    succeedFlag.value = true;
    api.post(`/dataset/${selectedDatasetId.value}/:set_succeed`).catch((error) => {
        console.error('Error setting succeed flag:', error);
    });
}

const succeedKeyHandler = (event) => {
    if (event.key === 'c' || event.key === 'C') {
        setSucceed();
    }
};

function addSucceedKeyListener() {
    window.removeEventListener('keydown', succeedKeyHandler);
    window.addEventListener('keydown', succeedKeyHandler);
}

function removeSucceedKeyListener() {
    window.removeEventListener('keydown', succeedKeyHandler);
}

function stopDataCollection() {
    if (teleType.value === 'keyboard') {
        removeKeyboardListener();
    }
    removeSucceedKeyListener();
    succeedFlag.value = false;
    viveInitializing.value = false;
    movingHomepose.value = false;
    api.post(`/dataset/${selectedDatasetId.value}/:stop_collection`).then(() => {
        collectingProgress.value = 0;
    })
}

const hz = ref(10);
const showInferenceDialog = ref(false);
const inferenceForm = ref([
    {
        key: 're_inference_steps',
        label: t('reInferenceSteps'),
        type: 'number',
        value: 1,
    },
    {
        key: 'temporal_ensemble_coeff',
        label: t('temporalEnsembleCoeff'),
        type: 'number',
        value: 0.01,
        show: (form) => form.find(f => f.key === 're_inference_steps')?.value === 1,
    },
]);

function startInference() {
    showInferenceDialog.value = true;
}

function onInferenceSubmit(formData) {
    showProcessConsole.value = true;
    api.post(`/checkpoint/${selectedCheckpointId.value}/:start_test`, {
        task: props.workspace,
        policy: checkpoint.value.policy,
        robot_ids: props.robots.map(r => r.id),
        sensors: props.sensors,
        checkpoint: checkpoint.value,
        move_homepose: moveHomposeInDataCollection.value,
        hz: hz.value,
        re_inference_steps: formData.re_inference_steps,
        temporal_ensemble_coeff: formData.re_inference_steps === 1 ? formData.temporal_ensemble_coeff : null,
    }).catch((error) => {
        console.error('Error starting test:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStartTest')
        });
    });
}

function stopInference() {
    return api.post(`/checkpoint/${selectedCheckpointId.value}/:stop_test`).catch((error) => {
        console.error('Error stopping test:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStopTest')
        });
    });
}

function startReplay() {
    replayProgress.value = 0;
    const payload = {
        episode: selectedEpisode.value,
        robot_ids: props.robots.map(r => r.id),
        sensors: props.sensors,
        task: props.workspace,
        action_type: replayActionType.value,
        hz: replayHz.value,
    }
    if (replayCapture.value && replayCaptureDatasetId.value) {
        payload.capture_dataset_id = replayCaptureDatasetId.value
    }
    api.post(`/dataset/${selectedDatasetId.value}/${selectedEpisode.value.name}/:start_replay_episode`, payload).catch((error) => {
        console.error('Error starting replay:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStartReplay')
        });
    });
}

function stopReplay() {
    api.post(`/dataset/${selectedDatasetId.value}/${selectedEpisode.value.name}/:stop_replay_episode`).catch((error) => {
        console.error('Error stopping replay:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStopReplay')
        });
    });
}

onMounted(() => {

    console.log(props.workspace);

    socket.on('stop_process', (data) => {
        if (data.id === 'record_episode') {
            if (data.episode_saved) {
                Notify.create({
                    color: 'positive',
                    message: t('episodeSaved'),
                });
            }
            collectingProgress.value = 0;
            viveInitializing.value = false;
            movingHomepose.value = false;
            succeedFlag.value = false;
            removeSucceedKeyListener();
        }
        if (data.id === 'replay_episode') {
            replayProgress.value = 0;
        }
        if (data.id === 'checkpoint_test') {
            inferenceSucceed.value = false;
            succeedScore.value = null;
            oodScore.value = null;
            Notify.create({
                color: 'positive',
                message: t('inferenceStopped')
            });
        }
    });

    socket.on('record_episode_progress', (data) => {
        collectingProgress.value = data.progress;
    });

    socket.on('moving_homepose', (data) => {
        movingHomepose.value = data.moving;
    });

    socket.on('episode_saved', () => {
        succeedFlag.value = false;
    });

    socket.on('inference_succeed', (data) => {
        inferenceSucceed.value = data.succeed;
        succeedScore.value = data.score;
    });

    socket.on('ood_score', (data) => {
        oodScore.value = data;
    });

    socket.on('vive_node_ready', () => {
        viveInitializing.value = false;
    });

    socket.on('vive_node_error', (data) => {
        viveInitializing.value = false;
        Notify.create({
            color: 'negative',
            message: `${t('viveConnectFail')}: ${data?.message || t('unreadable')}`,
        });
    });

    socket.on('replay_capture_progress', (data) => {
        replayProgress.value = data.progress;
    });
});

onUnmounted(() => {
    socket.off('vive_node_ready');
    socket.off('vive_node_error');
    socket.off('inference_succeed');
    socket.off('ood_score');
    removeSucceedKeyListener();
});
</script>