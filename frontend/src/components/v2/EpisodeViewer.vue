<template>
    <div style="width: 100%; min-width: 0; box-sizing: border-box; display: flex; flex-direction: column;">
        <div v-if="languageInstruction" class="text-white text-caption q-pb-sm">
            {{ $t('hdf5LanguagePrefix', { value: languageInstruction }) }}
        </div>
        <div class="q-py-sm" :style="imagesGridStyle">
            <div
                v-for="(image, name) in images"
                :key="name"
                :style="cellStyle"
                :class="imageClass"
            >
                <img :src="image" alt="" :style="imageStyle">
            </div>
        </div>

        <!-- Playback controls -->
        <div
            class="q-pt-sm items-center"
            style="display: flex; gap: 8px; width: 100%; box-sizing: border-box; min-width: 0;"
        >
            <q-btn
                round
                flat
                dense
                color="white"
                :icon="playbackIcon"
                :disable="disablePlayback"
                @click="onTogglePlayback"
            />
            <q-slider
                v-model="sliderFrame"
                :min="0"
                :max="sliderMax"
                :step="1"
                color="primary"
                track-color="grey-8"
                style="flex: 1 1 0; min-width: 0;"
                dense
                :disable="disableSeek || sliderMax === 0"
                @pan="onSliderPan"
                @update:model-value="onSliderInput"
            />
            <div class="text-caption text-grey-5" style="min-width: 60px; text-align: right; flex-shrink: 0;">
                {{ sliderFrame }} / {{ sliderMax }}
            </div>
        </div>
    </div>
</template>

<script setup>
import { defineProps, onMounted, watch, ref, computed, onUnmounted, defineEmits } from 'vue';
import { api } from 'src/boot/axios';
import { LocalStorage } from 'quasar';
import { useSocket } from 'src/composables/useSocket.js';

const { socket } = useSocket();

const props = defineProps({
    path: {
        type: String,
        required: true,
    },
    config: {
        type: Object,
        default: () => ({}),
    },
    imageClass: {
        type: String,
        default: '',
    },
    totalFrames: {
        type: Number,
        default: 0,
    },
    disableSeek: {
        type: Boolean,
        default: false,
    },
    disablePlayback: {
        type: Boolean,
        default: false,
    },
    previewFrame: {
        type: Number,
        default: null,
    },
});

const emit = defineEmits(['update:robotStates']);

const images = ref([]);
const robot_states = ref({});
const languageInstruction = ref('');

const paused = ref(false);
const finished = ref(false);
const seeking = ref(false);
const sliderFrame = ref(0);

const sliderMax = computed(() => Math.max(0, (props.totalFrames || 0) - 1));

const imageCount = computed(() => Object.keys(images.value || {}).length);

const cellHeight = computed(() => {
    const count = imageCount.value;
    if (count <= 1) return 'min(380px, 38vh)';
    if (count === 2) return 'min(220px, 20vh)';
    if (count === 3) return 'min(190px, 18vh)';
    return 'min(170px, 16vh)';
});

const cellStyle = computed(() => ({
    height: cellHeight.value,
    width: '100%',
    minWidth: 0,
    overflow: 'hidden',
    borderRadius: '4px',
    background: '#000',
    position: 'relative',
}));

const imageStyle = computed(() => ({
    width: '100%',
    height: '100%',
    objectFit: 'contain',
    display: 'block',
}));

const imagesGridStyle = computed(() => ({
    display: 'grid',
    gridTemplateColumns: `repeat(${Math.max(imageCount.value, 1)}, minmax(0, 1fr))`,
    gap: '8px',
    width: '100%',
    boxSizing: 'border-box',
}));

const playbackIcon = computed(() => {
    if (finished.value) return 'replay';
    return paused.value ? 'play_arrow' : 'pause';
});

function startReading(path, startFrame = 0) {
    if (!path) return;
    api.post(`/dataset/${path}/:start_read_dataset`, {
        sid: LocalStorage.getItem('socketId'),
        start_frame: Math.max(0, Math.floor(startFrame || 0)),
    }).then(() => {
        finished.value = false;
        paused.value = false;
        sliderFrame.value = Math.max(0, Math.floor(startFrame || 0));
    }).catch((error) => {
        console.error('Error starting episode replay:', error);
    });
}

function stopReading(path) {
    if (!path) return;
    api.post(`/dataset/${path}/:stop_read_dataset`).catch((error) => {
        console.error('Error stopping episode replay:', error);
    });
}

function fetchFrame(path, index) {
    if (!path) return Promise.resolve();
    return api.get(`/dataset/${path}/:frame`, { params: { index } })
        .then((res) => {
            if (res?.data?.images) {
                images.value = res.data.images;
            }
        })
        .catch((error) => {
            console.error('Error fetching frame:', error);
        });
}

function onTogglePlayback() {
    if (props.disablePlayback) return;
    if (finished.value) {
        startReading(props.path, 0);
        return;
    }
    if (paused.value) {
        // Resume from the current slider position
        startReading(props.path, sliderFrame.value);
        return;
    }
    paused.value = true;
    stopReading(props.path);
}

let panTimer = null;
function onSliderPan(phase) {
    if (props.disableSeek) return;
    if (phase === 'start') {
        seeking.value = true;
        if (!paused.value && !finished.value) {
            stopReading(props.path);
            paused.value = true;
        }
    } else if (phase === 'end') {
        seeking.value = false;
        fetchFrame(props.path, sliderFrame.value);
    }
}

function onSliderInput(val) {
    if (props.disableSeek) return;
    if (panTimer) clearTimeout(panTimer);
    panTimer = setTimeout(() => {
        fetchFrame(props.path, val);
    }, 60);
}

watch(() => props.path, (newVal, oldVal) => {
    if (oldVal) stopReading(oldVal);
    paused.value = false;
    finished.value = false;
    sliderFrame.value = 0;
    if (newVal) startReading(newVal);
});

// Replay 모드 전환 시 로컬 스트림 / paused 상태를 정리해서, 백엔드 replay 가
// 발행하는 show_episode_step / replay_progress 가 그대로 viewer 에 흘러가도록 한다.
watch(() => props.disablePlayback, (locked) => {
    if (locked) {
        // replay 시작 — 백엔드가 read_dataset_* 프로세스를 이미 죽였으므로 로컬
        // stop 호출은 불필요. 단지 paused 게이트만 풀어주면 됨.
        paused.value = false;
        finished.value = false;
        seeking.value = false;
    } else {
        // replay 종료 — 로컬 미리보기 스트림 재개.
        if (props.path) startReading(props.path, sliderFrame.value);
    }
});

watch(() => props.previewFrame, (newVal) => {
    if (newVal === null || newVal === undefined) return;
    // Pause streaming and show the requested preview frame (used by trim handles)
    if (!paused.value && !finished.value) {
        stopReading(props.path);
        paused.value = true;
    }
    sliderFrame.value = newVal;
    fetchFrame(props.path, newVal);
});

onMounted(() => {
    if (props.path) startReading(props.path);

    socket.on('show_episode_step', (data) => {
        if (paused.value || seeking.value) return;
        images.value = data.images;
        robot_states.value = data.robot_states || {};
        languageInstruction.value = data.language_instruction || '';
        emit('update:robotStates', robot_states.value);
    });

    socket.on('replay_progress', (data) => {
        if (seeking.value || paused.value) return;
        const p = typeof data?.progress === 'number' ? data.progress : 0;
        const clamped = Math.max(0, Math.min(1, p));
        sliderFrame.value = Math.round(clamped * sliderMax.value);
        if (clamped >= 1) {
            finished.value = true;
        }
    });
});

onUnmounted(() => {
    socket.off('show_episode_step');
    socket.off('replay_progress');
    stopReading(props.path);
});
</script>
