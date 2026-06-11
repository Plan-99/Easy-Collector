<template>
    <div style="width: 100%; min-width: 0; box-sizing: border-box; display: flex; flex-direction: column;">
        <div v-if="languageInstruction" class="text-white text-caption q-pb-sm">
            {{ $t('hdf5LanguagePrefix', { value: languageInstruction }) }}
        </div>
        <div style="position: relative;">
            <div class="q-py-sm" :style="imagesGridStyle">
                <div
                    v-for="(image, name) in images"
                    :key="name"
                    :style="cellStyle"
                    :class="imageClass"
                >
                    <img :src="image" alt="" :style="imageStyle">
                    <!-- Vision map heatmap overlay. Positioned absolutely on top of
                         the camera image, sized to match. pointer-events:none so
                         slider/play controls still work. -->
                    <img
                        v-if="visionMapOn && currentHeatmaps[name]"
                        :src="currentHeatmaps[name]"
                        alt=""
                        :style="overlayImageStyle"
                    />
                </div>
            </div>
            <div v-if="vmLoading" :style="vmLoadingOverlayStyle">
                <q-spinner color="primary" size="44px" />
                <div class="text-white q-mt-sm text-caption">
                    {{ $t('visionMapPreparing') || '비전 맵 준비 중...' }}
                </div>
                <div v-if="vmProgressTotal > 0" class="text-grey-4 q-mt-xs" style="font-size: 11px;">
                    {{ vmProgressDone }} / {{ vmProgressTotal }}
                </div>
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

        <!-- Vision map row -->
        <div class="q-pt-xs items-center" style="display: flex; gap: 8px; flex-wrap: wrap;">
            <q-checkbox
                v-model="visionMapOn"
                :label="$t('visionMap') || 'Vision Map'"
                dark
                size="sm"
                color="primary"
                class="text-white"
                :disable="!vmCheckpointId"
            />
            <q-btn
                flat
                dense
                round
                size="sm"
                icon="settings"
                color="grey-5"
                @click="openVisionMapSettings"
            >
                <q-tooltip>{{ $t('visionMapSettings') || 'Vision Map Settings' }}</q-tooltip>
            </q-btn>
            <span v-if="visionMapStatus" class="text-caption" :class="visionMapStatusClass">
                {{ visionMapStatus }}
            </span>
            <!-- 현재 프레임의 OOD / success 점수 — vision map 과 같은 추론에서 계산.
                 OOD: 0(분포 내) → 1(분포 밖). Success: succeed 토큰 값(0~1 근처). -->
            <template v-if="showScorePanel">
                <q-separator vertical dark />
                <q-badge v-if="vmHasOod" :color="oodColor" text-color="white">
                    {{ $t('oodScore') || 'OOD' }}&nbsp;{{ currentScore.ood != null ? currentScore.ood.toFixed(3) : '—' }}
                    <q-tooltip>{{ $t('oodScoreHint') || '학습 분포 대비 이미지 OOD 정도 (0 = 분포 내, 1 = 분포 밖)' }}</q-tooltip>
                </q-badge>
                <q-badge v-if="vmHasSuccess" :color="successColor" text-color="white">
                    {{ $t('successScore') || 'Success' }}&nbsp;{{ currentScore.success != null ? currentScore.success.toFixed(3) : '—' }}
                    <q-tooltip>{{ $t('successScoreHint') || '정책의 succeed 토큰 값 (>0.5 면 성공 예측)' }}</q-tooltip>
                </q-badge>
            </template>
        </div>

        <!-- Vision map settings dialog -->
        <q-dialog v-model="showVMSettings">
            <q-card class="bg-secondary text-white" style="min-width: 380px;">
                <q-card-section class="row items-center">
                    <q-icon name="visibility" size="22px" color="primary" class="q-mr-sm" />
                    <div class="text-h6">{{ $t('visionMapSettings') || 'Vision Map Settings' }}</div>
                </q-card-section>
                <q-card-section class="q-pt-none">
                    <q-select
                        v-model="vmCheckpointId"
                        :options="checkpointOptions"
                        :label="$t('checkpoint') || 'Checkpoint'"
                        emit-value
                        map-options
                        dark
                        outlined
                        dense
                        bg-color="dark"
                        :no-options-label="$t('visionMapNoCheckpoints') || 'No finished ACT checkpoints'"
                    >
                        <template v-slot:no-option>
                            <q-item><q-item-section class="text-grey-5">
                                {{ $t('visionMapNoCheckpoints') || 'No finished ACT checkpoints for this workspace' }}
                            </q-item-section></q-item>
                        </template>
                    </q-select>
                    <q-select
                        v-model="vmMethod"
                        :options="methodOptions"
                        :label="$t('visionMapMethod') || 'Method'"
                        emit-value
                        map-options
                        dark
                        outlined
                        dense
                        bg-color="dark"
                        class="q-mt-md"
                    />
                    <div class="text-caption text-grey-5 q-mt-sm">
                        {{ vmMethodHelp }}
                    </div>
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat :label="$t('close') || 'Close'" color="grey" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>
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
    // Workspace/task id — used to filter Vision Map checkpoint candidates to
    // the current task. Null/undefined ⇒ vision map UI shows "no checkpoints".
    taskId: {
        type: [Number, String],
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

const overlayImageStyle = computed(() => ({
    position: 'absolute',
    top: 0,
    left: 0,
    width: '100%',
    height: '100%',
    objectFit: 'contain',
    pointerEvents: 'none',
}));

const vmLoadingOverlayStyle = computed(() => ({
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    background: 'rgba(0, 0, 0, 0.55)',
    borderRadius: '4px',
    zIndex: 10,
    pointerEvents: 'none',
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

// =========================================================================
// Vision Map — per-frame ACT saliency overlay (episode precompute).
//
// On toggle ON (or checkpoint/method change) we ask the backend to compute
// heatmaps for every frame of the current episode and stream them back over
// Socket.IO:
//   POST /checkpoint/<id>/:vision_map_episode_start
//     body: { dataset_id, episode_idx, method, session_id }
//   ← vision_map_episode_start { session_id, total_frames }
//   ← vision_map_episode_frame { session_id, frame_idx, heatmaps }  × N
//   ← vision_map_episode_done  { session_id, computed, total }
//
// Playback is paused with a loading overlay until the *whole* episode is
// computed; then it auto-resumes and the overlay swaps in the matching
// per-frame heatmap from heatmapCache. Single backend slot — starting a new
// precompute cancels any prior one.
// =========================================================================
const visionMapOn = ref(false);
const showVMSettings = ref(false);
const vmCheckpointId = ref(LocalStorage.getItem('vmCheckpointId') || null);
const vmMethod = ref(LocalStorage.getItem('vmMethod') || 'attention');
const heatmapCache = ref({});        // { [frame_idx]: { sensor_name: data-url } }
// 프레임별 success(succeed 토큰) / ood(image OOD 백분위) 점수. heatmap 과 같은
// vision_map_episode_frame 이벤트로 함께 들어온다. has* 플래그는 start 에서 받아
// 점수 컬럼 노출 여부를 정한다(체크포인트에 해당 정보가 없으면 숨김).
const scoreCache = ref({});          // { [frame_idx]: { success, ood } }
const vmHasSuccess = ref(false);
const vmHasOod = ref(false);
const visionMapStatus = ref('');
const visionMapStatusClass = ref('text-grey-5');
const taskCheckpoints = ref([]);
// Loading overlay state — true while the episode is being precomputed.
// Playback is paused for the duration and auto-resumes on completion.
const vmLoading = ref(false);
const vmProgressDone = ref(0);
const vmProgressTotal = ref(0);
let vmShouldResumePlayback = false;
// Active session id — only socket events with this id update state. Mutated
// (not reffed) because we don't need reactivity on it.
let vmSessionId = null;

// Heatmaps overlaid on the current frame. Empty until the matching frame_idx
// has been computed.
const currentHeatmaps = computed(() => {
    if (!visionMapOn.value) return {};
    return heatmapCache.value[sliderFrame.value] || {};
});

// 현재 프레임의 success / ood 점수 (vision map 켜진 동안 readout 에 표시).
const currentScore = computed(() => {
    if (!visionMapOn.value) return {};
    return scoreCache.value[sliderFrame.value] || {};
});
const showScorePanel = computed(() =>
    visionMapOn.value && (vmHasSuccess.value || vmHasOod.value)
);
// OOD: 낮음(분포 내)=초록 → 높음(분포 밖)=빨강. Success: >0.5 면 초록.
const oodColor = computed(() => {
    const v = currentScore.value.ood;
    if (v == null) return 'grey-6';
    return v >= 0.66 ? 'negative' : v >= 0.33 ? 'warning' : 'positive';
});
const successColor = computed(() => {
    const v = currentScore.value.success;
    if (v == null) return 'grey-6';
    return v > 0.5 ? 'positive' : 'grey-7';
});

const methodOptions = [
    { label: 'Attention (decoder cross-attention)', value: 'attention' },
    { label: 'Grad-CAM (backbone saliency)', value: 'gradcam' },
];

const vmMethodHelp = computed(() => {
    if (vmMethod.value === 'gradcam') {
        return '액션 노름에 대한 backbone feature 의 Grad-CAM. 액션을 만드는 데 영향을 주는 픽셀 위치를 보여줍니다. (느림 — backward pass 필요)';
    }
    return '디코더 cross-attention 의 평균 — action query 가 visual token 의 어디를 보는지. (빠름)';
});

// Only ACT-based checkpoints support the vision_map endpoint — the dropdown
// hides every other policy type so the user can't pick something unsupported.
const actCheckpoints = computed(() =>
    taskCheckpoints.value.filter((c) => String(c?.policy?.type || '').toLowerCase() === 'act'),
);

const checkpointOptions = computed(() =>
    actCheckpoints.value.map((c) => ({
        label: `#${c.id} · ${c.name || `(unnamed)`}`,
        value: c.id,
    })),
);

function loadCheckpointsForTask() {
    if (!props.taskId) {
        taskCheckpoints.value = [];
        return;
    }
    api.get('/checkpoints', { params: { where: `task_id,=,${props.taskId}|status,=,finished` } })
        .then((res) => {
            taskCheckpoints.value = res.data?.checkpoints || [];
            // Validate persisted ID against ACT-only options. If invalid or
            // missing, fall back to the most recent ACT checkpoint so the
            // checkbox is usable out of the box.
            const acts = actCheckpoints.value;
            const persistedValid = vmCheckpointId.value &&
                acts.some((c) => String(c.id) === String(vmCheckpointId.value));
            if (!persistedValid) {
                if (acts.length > 0) {
                    const sorted = [...acts].sort((a, b) => {
                        if (a.created_at && b.created_at) {
                            return new Date(b.created_at) - new Date(a.created_at);
                        }
                        return (b.id || 0) - (a.id || 0);
                    });
                    vmCheckpointId.value = sorted[0].id;
                } else {
                    vmCheckpointId.value = null;
                }
            }
        })
        .catch((err) => {
            console.error('[VisionMap] failed to load checkpoints:', err);
            taskCheckpoints.value = [];
        });
}

function openVisionMapSettings() {
    loadCheckpointsForTask();
    showVMSettings.value = true;
}

function parsePath(path) {
    // path = `${dataset_id}/${episode_name}` e.g. "abc123/episode_3"
    if (!path) return null;
    const parts = String(path).split('/');
    if (parts.length < 2) return null;
    const dataset_id = parts[0];
    const epName = parts[1];
    const m = String(epName).replace('.hdf5', '').match(/(\d+)/);
    if (!m) return null;
    return { dataset_id, episode_idx: parseInt(m[1], 10) };
}

function newSessionId() {
    return `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
}

function startEpisodePrecompute() {
    if (!visionMapOn.value || !vmCheckpointId.value) return;
    const parsed = parsePath(props.path);
    if (!parsed) return;

    // Pause playback for the duration of the precompute; remember whether
    // we were playing so we can auto-resume on completion.
    const wasPlaying = !paused.value && !finished.value;
    if (wasPlaying) {
        stopReading(props.path);
        paused.value = true;
    }
    vmShouldResumePlayback = wasPlaying;

    // Reset state, generate a fresh session id so straggler events from a
    // prior request are ignored.
    heatmapCache.value = {};
    vmProgressDone.value = 0;
    vmProgressTotal.value = 0;
    vmLoading.value = true;
    visionMapStatus.value = '비전 맵 준비 중...';
    visionMapStatusClass.value = 'text-grey-5';
    vmSessionId = newSessionId();

    api.post(`/checkpoint/${vmCheckpointId.value}/:vision_map_episode_start`, {
        dataset_id: parsed.dataset_id,
        episode_idx: parsed.episode_idx,
        method: vmMethod.value,
        session_id: vmSessionId,
    }).catch((err) => {
        const msg = err?.response?.data?.message || err?.message || 'failed';
        visionMapStatus.value = `오류: ${msg}`;
        visionMapStatusClass.value = 'text-red-4';
        vmLoading.value = false;
        vmShouldResumePlayback = false;
        vmSessionId = null;
        console.error('[VisionMap] start failed:', err);
    });
}

function stopEpisodePrecompute() {
    // Best-effort cancel — backend ignores if nothing is running.
    if (!vmCheckpointId.value) return;
    api.post(`/checkpoint/${vmCheckpointId.value}/:vision_map_episode_stop`)
        .catch((err) => console.warn('[VisionMap] stop failed:', err));
}

// Socket handlers — only mutate state for the current session.
function onVMEpisodeStart(payload) {
    if (!payload || payload.session_id !== vmSessionId) return;
    vmProgressTotal.value = payload.total_frames || 0;
    vmProgressDone.value = 0;
    vmHasSuccess.value = !!payload.has_success;
    vmHasOod.value = !!payload.has_ood;
    scoreCache.value = {};
    visionMapStatus.value = `0 / ${vmProgressTotal.value} 프레임`;
}

function onVMEpisodeFrame(payload) {
    if (!payload || payload.session_id !== vmSessionId) return;
    if (typeof payload.frame_idx !== 'number') return;
    // Mutate then reassign so the computed dependency triggers without
    // making a deep copy of every frame's PNGs.
    heatmapCache.value[payload.frame_idx] = payload.heatmaps || {};
    heatmapCache.value = { ...heatmapCache.value };
    // success / ood 점수도 같은 프레임 키로 캐시(없으면 null).
    scoreCache.value[payload.frame_idx] = {
        success: payload.success ?? null,
        ood: payload.ood ?? null,
    };
    scoreCache.value = { ...scoreCache.value };
    vmProgressDone.value += 1;
    if (vmProgressTotal.value > 0) {
        visionMapStatus.value = `${vmProgressDone.value} / ${vmProgressTotal.value} 프레임`;
    }
}

function onVMEpisodeDone(payload) {
    if (!payload || payload.session_id !== vmSessionId) return;
    vmLoading.value = false;
    visionMapStatus.value = '';
    // Auto-resume playback now that every frame has its overlay.
    if (vmShouldResumePlayback) {
        vmShouldResumePlayback = false;
        if (visionMapOn.value && !finished.value && props.path) {
            startReading(props.path, sliderFrame.value);
        }
    }
}

function onVMEpisodeError(payload) {
    if (!payload || payload.session_id !== vmSessionId) return;
    visionMapStatus.value = `오류: ${payload.message || 'failed'}`;
    visionMapStatusClass.value = 'text-red-4';
    vmLoading.value = false;
    vmShouldResumePlayback = false;
    vmSessionId = null;
}

// Persist settings + restart precompute when they change while toggle is on.
watch(vmCheckpointId, (v) => {
    LocalStorage.set('vmCheckpointId', v);
    if (visionMapOn.value) startEpisodePrecompute();
});
watch(vmMethod, (v) => {
    LocalStorage.set('vmMethod', v);
    if (visionMapOn.value) startEpisodePrecompute();
});

// Toggle on → start precompute. Toggle off → cancel + drop overlays.
watch(visionMapOn, (on) => {
    if (on) {
        if (!vmCheckpointId.value) {
            openVisionMapSettings();
            return;
        }
        startEpisodePrecompute();
    } else {
        stopEpisodePrecompute();
        heatmapCache.value = {};
        visionMapStatus.value = '';
        vmLoading.value = false;
        vmShouldResumePlayback = false;
        vmSessionId = null;
    }
});

// Episode change → restart precompute for the new episode.
watch(() => props.path, () => {
    if (visionMapOn.value && vmCheckpointId.value) {
        startEpisodePrecompute();
    } else {
        heatmapCache.value = {};
    }
});

// Pull checkpoints whenever task changes (so the picker is fresh when user
// opens the settings dialog without an explicit refresh).
watch(() => props.taskId, () => loadCheckpointsForTask(), { immediate: true });
// =========================================================================

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

    socket.on('vision_map_episode_start', onVMEpisodeStart);
    socket.on('vision_map_episode_frame', onVMEpisodeFrame);
    socket.on('vision_map_episode_done', onVMEpisodeDone);
    socket.on('vision_map_episode_error', onVMEpisodeError);
});

onUnmounted(() => {
    socket.off('show_episode_step');
    socket.off('replay_progress');
    socket.off('vision_map_episode_start', onVMEpisodeStart);
    socket.off('vision_map_episode_frame', onVMEpisodeFrame);
    socket.off('vision_map_episode_done', onVMEpisodeDone);
    socket.off('vision_map_episode_error', onVMEpisodeError);
    if (visionMapOn.value) stopEpisodePrecompute();
    stopReading(props.path);
});
</script>
