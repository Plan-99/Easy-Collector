<template>
    <div class="column" style="min-width: 0; max-width: 100%; overflow-x: hidden;">
        <div v-if="languageInstruction" class="text-white text-caption q-pb-sm">
            {{ $t('hdf5LanguagePrefix', { value: languageInstruction }) }}
        </div>
        <div
            class="row q-py-sm q-gutter-sm justify-center items-center"
            style="min-height: 220px; min-width: 0; max-width: 100%; overflow-x: hidden; flex-wrap: wrap;"
        >
            <img
                v-for="(image, name) in images"
                :key="name"
                :src="image"
                alt=""
                style="flex: 1 1 240px; min-width: 0; max-width: 100%; max-height: 320px; height: auto; object-fit: contain;"
                :class="imageClass"
            >
        </div>

        <!-- Playback controls -->
        <div class="row items-center q-gutter-sm q-px-sm q-pt-sm">
            <q-btn
                round
                flat
                dense
                color="white"
                :icon="playbackIcon"
                :disable="disableSeek"
                @click="onTogglePlayback"
            />
            <q-slider
                v-model="sliderFrame"
                :min="0"
                :max="sliderMax"
                :step="1"
                color="primary"
                track-color="grey-8"
                class="col"
                dense
                :disable="disableSeek || sliderMax === 0"
                @pan="onSliderPan"
                @update:model-value="onSliderInput"
            />
            <div class="text-caption text-grey-5" style="min-width: 80px; text-align: right;">
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
    if (props.disableSeek) return;
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
