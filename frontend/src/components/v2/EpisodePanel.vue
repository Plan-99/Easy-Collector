<template>
    <div class="column text-white ep-panel" v-if="episode">
        <!-- Header -->
        <div class="row items-center q-mb-md ep-panel__header">
            <q-icon name="movie_creation" size="22px" color="primary" class="q-mr-sm" />
            <div class="column" style="min-width: 0;">
                <div class="text-subtitle1 text-weight-medium ellipsis">{{ episode.name }}</div>
                <div class="text-caption text-grey-5" v-if="totalFrames > 0">
                    {{ $t('datasetFrames', { count: totalFrames }) }}
                </div>
            </div>
            <q-space />
            <q-btn flat round dense icon="close" color="grey-5" @click="$emit('close')">
                <q-tooltip>{{ $t('close') || 'Close' }}</q-tooltip>
            </q-btn>
        </div>

        <!-- Video section (socketio 프레임 스트리밍 + 재생/슬라이더) -->
        <section class="ep-section q-mb-md" style="min-width: 0;">
            <episode-viewer
                :key="hdf5Path"
                :path="hdf5Path"
                :total-frames="totalFrames"
                :preview-frame="trimPreviewFrame"
            />
        </section>

        <!-- Trim section -->
        <section class="ep-section q-mb-md">
            <div class="ep-section__title">
                <q-icon name="content_cut" size="16px" color="amber-5" />
                <span>{{ $t('datasetTrimRangeLabel') }}</span>
                <q-space />
                <q-badge
                    v-if="totalFrames > 0"
                    color="dark"
                    text-color="amber-5"
                    class="ep-trim-badge"
                >
                    <span class="text-weight-medium">{{ trimStart }}</span>
                    <q-icon name="arrow_forward" size="12px" class="q-mx-xs" />
                    <span class="text-weight-medium">{{ trimEnd }}</span>
                    <span class="text-grey-5 q-ml-sm">
                        · {{ $t('datasetFrames', { count: Math.max(0, trimEnd - trimStart) }) }}
                    </span>
                </q-badge>
            </div>
            <div class="q-px-sm q-pt-sm">
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
            </div>
            <div class="row q-gutter-sm q-mt-sm q-px-sm">
                <q-btn
                    color="primary"
                    :label="$t('datasetTrimApply')"
                    icon="content_cut"
                    size="sm"
                    unelevated
                    no-caps
                    :disable="!canTrim"
                    @click="$emit('trim', { start: trimStart, end: trimEnd })"
                />
                <q-btn
                    flat
                    color="grey-4"
                    :label="$t('datasetTrimReset')"
                    icon="restore"
                    size="sm"
                    no-caps
                    @click="resetTrim"
                />
            </div>
        </section>

        <!-- Language prompt section -->
        <section class="ep-section q-mb-md">
            <div class="ep-section__title">
                <q-icon name="translate" size="16px" color="primary" />
                <span>{{ $t('datasetLanguagePrompt') }}</span>
                <q-space />
                <q-btn
                    flat
                    dense
                    size="sm"
                    color="primary"
                    icon="save"
                    :label="$t('saveLanguage')"
                    no-caps
                    :disable="languageDraft === (data?.language_instruction || '')"
                    @click="$emit('saveLanguage', languageDraft)"
                />
            </div>
            <q-input
                v-model="languageDraft"
                outlined
                dense
                dark
                color="primary"
                bg-color="dark"
                type="textarea"
                autogrow
                input-class="text-white"
                :placeholder="$t('datasetLanguagePlaceholder')"
            />
        </section>

        <!-- Sensor data section -->
        <section class="ep-section">
            <div class="ep-section__title">
                <q-icon name="show_chart" size="16px" color="primary" />
                <span>{{ $t('datasetSectionSensors') }}</span>
                <q-space />
                <span class="text-caption text-grey-5" v-if="visibleSeriesCount">
                    {{ $t('datasetSeriesShown') }}: <span class="text-white">{{ visibleSeriesCount }}</span>
                </span>
            </div>

            <q-select
                v-model="selectedChannelKey"
                :options="channelOptions"
                outlined
                dense
                dark
                color="primary"
                bg-color="dark"
                emit-value
                map-options
                :label="$t('datasetChannelLabel')"
                input-class="text-white"
                label-color="grey-5"
                class="q-mb-sm"
            />

            <q-select
                v-if="currentChannel"
                v-model="visibleSeries"
                :options="currentChannel.names"
                outlined
                dense
                dark
                color="primary"
                bg-color="dark"
                multiple
                use-chips
                :label="$t('datasetSelectSeries')"
                input-class="text-white"
                label-color="grey-5"
                class="q-mb-md"
            >
                <template v-slot:before>
                    <q-btn flat dense size="sm" color="primary" :label="$t('datasetAll')" no-caps @click="selectAllSeries" />
                    <q-btn flat dense size="sm" color="grey-5" :label="$t('datasetNone')" no-caps @click="visibleSeries = []" />
                </template>
            </q-select>

            <div class="ep-chart-wrapper">
                <Line
                    v-if="chartData.datasets.length"
                    :data="chartData"
                    :options="chartOptions"
                />
                <div v-else class="ep-chart-empty">
                    <q-icon name="bar_chart" size="40px" color="grey-7" />
                    <div class="text-caption text-grey-6 q-mt-xs">{{ $t('datasetGraphEmpty') }}</div>
                </div>
            </div>
        </section>
    </div>
</template>

<style scoped>
.ep-panel__header {
    padding: 4px 4px 0 4px;
}

.ep-section {
    background: rgba(255, 255, 255, 0.03);
    border: 1px solid rgba(255, 255, 255, 0.07);
    border-radius: 10px;
    padding: 12px 14px;
}

.ep-section__title {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 12px;
    padding-bottom: 10px;
    border-bottom: 1px solid rgba(255, 255, 255, 0.06);
    color: #e0e0e0;
    font-size: 13px;
    font-weight: 500;
}

.ep-trim-badge {
    padding: 4px 10px;
    font-size: 12px;
    border-radius: 6px;
}

.ep-chart-wrapper {
    position: relative;
    height: 240px;
    background: rgba(0, 0, 0, 0.4);
    border-radius: 8px;
    padding: 8px;
}

.ep-chart-empty {
    position: absolute;
    inset: 0;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 4px;
}
</style>

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
