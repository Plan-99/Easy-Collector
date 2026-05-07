<template>
    <div class="column text-white" v-if="episode">
        <!-- Header -->
        <div class="row items-center q-mb-md">
            <div class="text-subtitle1 text-weight-medium">{{ episode.name }}</div>
            <q-space />
            <q-btn flat round dense icon="close" color="white" @click="$emit('close')" />
        </div>

        <!-- Video (socketio 프레임 스트리밍 + 재생/슬라이더) -->
        <div class="bg-dark border-rounded q-pa-sm q-mb-md" style="min-width: 0; overflow-x: hidden;">
            <episode-viewer
                :key="hdf5Path"
                :path="hdf5Path"
                :total-frames="totalFrames"
                :preview-frame="trimPreviewFrame"
            />
        </div>

        <!-- Trim range slider with two handles -->
        <div class="bg-dark border-rounded q-pa-sm q-mb-md">
            <div class="row items-center q-mb-xs text-caption text-grey-5">
                <span>{{ $t('datasetTrimRangeLabel') }}</span>
                <q-space />
                <span v-if="totalFrames > 0">
                    {{ trimStart }} – {{ trimEnd }}
                    ({{ $t('datasetFrames', { count: Math.max(0, trimEnd - trimStart) }) }})
                </span>
            </div>
            <q-range
                v-model="trimRange"
                :min="0"
                :max="Math.max(1, totalFrames - 1)"
                :step="1"
                :disable="totalFrames === 0"
                label
                color="amber"
                drag-range
            />
            <div class="row q-gutter-sm q-mt-sm">
                <q-btn
                    color="primary"
                    :label="$t('datasetTrimApply')"
                    icon="content_cut"
                    size="sm"
                    :disable="!canTrim"
                    @click="$emit('trim', { start: trimStart, end: trimEnd })"
                />
                <q-btn
                    outline
                    color="grey-5"
                    :label="$t('datasetTrimReset')"
                    icon="restore"
                    size="sm"
                    @click="resetTrim"
                />
            </div>
        </div>

        <!-- Language prompt -->
        <div class="q-mb-md">
            <q-input
                v-model="languageDraft"
                outlined
                dense
                dark
                color="grey-5"
                bg-color="dark"
                :label="$t('datasetLanguagePrompt')"
                type="textarea"
                autogrow
                input-class="text-white"
                label-color="grey-5"
            >
                <template v-slot:append>
                    <q-btn
                        flat
                        dense
                        size="sm"
                        color="primary"
                        icon="save"
                        :disable="languageDraft === (data?.language_instruction || '')"
                        @click="$emit('saveLanguage', languageDraft)"
                    >
                        <q-tooltip>{{ $t('saveLanguage') }}</q-tooltip>
                    </q-btn>
                </template>
            </q-input>
        </div>

        <!-- Channel / state field selector -->
        <div class="row q-gutter-sm q-mb-sm items-center">
            <q-select
                v-model="selectedChannelKey"
                :options="channelOptions"
                outlined
                dense
                dark
                color="grey-5"
                bg-color="dark"
                emit-value
                map-options
                :label="$t('datasetChannelLabel')"
                style="min-width: 240px;"
                input-class="text-white"
                label-color="grey-5"
            />
            <q-space />
            <div class="text-caption text-grey-5">
                {{ $t('datasetSeriesShown', { count: visibleSeriesCount }) }}
            </div>
        </div>

        <!-- Per-channel name multi-select -->
        <div class="q-mb-sm" v-if="currentChannel">
            <q-select
                v-model="visibleSeries"
                :options="currentChannel.names"
                outlined
                dense
                dark
                color="grey-5"
                bg-color="dark"
                multiple
                use-chips
                :label="$t('datasetSelectSeries')"
                input-class="text-white"
                label-color="grey-5"
            >
                <template v-slot:before>
                    <q-btn flat dense size="sm" color="grey-5" :label="$t('datasetAll')" @click="selectAllSeries" />
                    <q-btn flat dense size="sm" color="grey-5" :label="$t('datasetNone')" @click="visibleSeries = []" />
                </template>
            </q-select>
        </div>

        <!-- Chart -->
        <div style="height: 260px; position: relative;" class="q-mb-md">
            <Line
                v-if="chartData.datasets.length"
                :data="chartData"
                :options="chartOptions"
            />
            <div v-else class="text-grey-5 text-center q-pa-md">
                {{ $t('datasetGraphEmpty') }}
            </div>
        </div>
    </div>
</template>

<script setup>
import { ref, computed, watch } from 'vue';
import { Line } from 'vue-chartjs';
import { api } from 'src/boot/axios';
import EpisodeViewer from 'src/components/v2/EpisodeViewer.vue';
import {
    Chart as ChartJS,
    Title,
    Tooltip,
    Legend,
    PointElement,
    LineElement,
    CategoryScale,
    LinearScale,
} from 'chart.js';

ChartJS.register(Title, Tooltip, Legend, PointElement, LineElement, CategoryScale, LinearScale);

const props = defineProps({
    datasetId: { type: [Number, String], default: null },
    episode: { type: Object, default: null },
});

defineEmits(['close', 'trim', 'saveLanguage']);

const data = ref(null);
const trimRange = ref({ min: 0, max: 0 });
const selectedChannelKey = ref('observation.state');
const visibleSeries = ref([]);
const languageDraft = ref('');
const trimPreviewFrame = ref(null);
let prevTrim = null;

const totalFrames = computed(() => data.value?.num_frames || 0);
const channels = computed(() => data.value?.channels || {});

const hdf5Path = computed(() => {
    if (!props.datasetId || !props.episode?.name) return '';
    return `${props.datasetId}/${props.episode.name}`;
});

const channelOptions = computed(() =>
    Object.keys(channels.value).map((k) => ({
        label: humanizeKey(k),
        value: k,
    })),
);

const currentChannel = computed(() => channels.value[selectedChannelKey.value] || null);
const visibleSeriesCount = computed(() => visibleSeries.value.length);

const trimStart = computed(() => trimRange.value.min);
const trimEnd = computed(() => trimRange.value.max);
const canTrim = computed(
    () => totalFrames.value > 0 && (trimStart.value > 0 || trimEnd.value < totalFrames.value - 1),
);

const colorPalette = [
    '#42A5F5', '#EF5350', '#66BB6A', '#FFA726', '#AB47BC',
    '#26A69A', '#FFCA28', '#5C6BC0', '#EC407A', '#8D6E63',
    '#26C6DA', '#9CCC65', '#FF7043', '#7E57C2', '#789262',
];

const chartData = computed(() => {
    const ch = currentChannel.value;
    if (!ch || !ch.data || !ch.data.length) return { labels: [], datasets: [] };
    const labels = ch.data.map((_, i) => i);
    const datasets = ch.names
        .map((name, idx) => ({ name, idx }))
        .filter(({ name }) => visibleSeries.value.includes(name))
        .map(({ name, idx }, n) => ({
            label: name,
            data: ch.data.map((row) => row[idx]),
            borderColor: colorPalette[n % colorPalette.length],
            backgroundColor: colorPalette[n % colorPalette.length],
            pointRadius: 0,
            borderWidth: 1.5,
            tension: 0,
        }));
    return { labels, datasets };
});

const chartOptions = computed(() => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    interaction: { mode: 'index', intersect: false },
    plugins: {
        legend: { labels: { color: '#e0e0e0' } },
        tooltip: { mode: 'index', intersect: false },
    },
    scales: {
        x: {
            type: 'linear',
            title: { display: true, text: 'frame', color: '#bdbdbd' },
            ticks: { color: '#bdbdbd' },
            grid: { color: 'rgba(255,255,255,0.05)' },
        },
        y: {
            ticks: { color: '#bdbdbd' },
            grid: { color: 'rgba(255,255,255,0.05)' },
        },
    },
}));

function humanizeKey(k) {
    return k.replace('observation.', 'obs.').replace('.', ' › ');
}

function resetTrim() {
    trimRange.value = { min: 0, max: Math.max(0, totalFrames.value - 1) };
}

function selectAllSeries() {
    if (currentChannel.value) {
        visibleSeries.value = [...currentChannel.value.names];
    }
}

async function loadEpisodeData() {
    if (!props.datasetId || !props.episode) return;
    data.value = null;
    try {
        const res = await api.get(
            `/dataset/${props.datasetId}/${props.episode.name}/:get_data`,
        );
        data.value = res.data.episode;
        languageDraft.value = data.value.language_instruction || '';
        trimRange.value = { min: 0, max: Math.max(0, data.value.num_frames - 1) };
        if (channels.value['observation.state']) {
            selectedChannelKey.value = 'observation.state';
        } else {
            const keys = Object.keys(channels.value);
            if (keys.length) selectedChannelKey.value = keys[0];
        }
        if (currentChannel.value) {
            visibleSeries.value = currentChannel.value.names.slice(0, 3);
        }
    } catch (err) {
        console.error('[EpisodeViewer] load failed:', err);
    }
}

watch(
    () => [props.datasetId, props.episode?.name],
    () => {
        prevTrim = null;
        trimPreviewFrame.value = null;
        loadEpisodeData();
    },
    { immediate: true },
);

watch(
    trimRange,
    (newVal) => {
        if (!newVal) return;
        if (!prevTrim) {
            prevTrim = { ...newVal };
            return;
        }
        const minChanged = newVal.min !== prevTrim.min;
        const maxChanged = newVal.max !== prevTrim.max;
        let target = null;
        if (minChanged && !maxChanged) target = newVal.min;
        else if (maxChanged && !minChanged) target = newVal.max;
        else if (minChanged && maxChanged) target = newVal.min;
        prevTrim = { ...newVal };
        if (target !== null) trimPreviewFrame.value = target;
    },
    { deep: true },
);

watch(selectedChannelKey, () => {
    if (currentChannel.value) {
        visibleSeries.value = currentChannel.value.names.slice(0, 3);
    }
});
</script>
