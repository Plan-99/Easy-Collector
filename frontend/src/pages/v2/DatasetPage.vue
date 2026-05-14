<template>
    <q-page class="q-pt-md q-pr-md text-white full-height column">
        <!-- Header / workspace selector -->
        <div class="border-rounded bg-secondary q-pa-md q-mb-md row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-md">{{ $t('datasetPageTitle') }}</div>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        color="grey-5"
                        v-model="selectedWorkspaceId"
                        :options="workspaces"
                        :label="$t('workspaceSelectLabel')"
                        style="width: 400px"
                        class="q-ml-md"
                        map-options
                        emit-value
                        option-label="name"
                        option-value="id"
                        input-class="text-white"
                        label-color="grey-5"
                        :disable="pageLoading"
                    />
                </div>
                <div class="text-body2 text-grey-5">{{ $t('datasetPageBody') }}</div>
                <div class="text-body2 text-grey-5">{{ $t('datasetPageBody2') }}</div>
            </div>
        </div>

        <div
            class="col q-mb-md border-rounded border-grey flex-center flex column"
            v-if="pageLoading"
        >
            <q-spinner-gears size="50px" color="primary" class="q-mb-md" />
            <div class="text-h6 text-grey">{{ $t('plannerInitializing') }}</div>
        </div>
        <div
            class="col q-mb-md border-rounded border-grey text-grey-5 flex-center flex text-h6"
            v-else-if="!selectedWorkspaceId"
        >
            {{ $t('selectWorkspaceFirst') }}
        </div>

        <div class="col row q-mb-md items-stretch" style="min-height: 0;" v-else>
            <!-- LEFT: dataset list with episodes -->
            <div class="col-4 bg-secondary q-mr-md border-rounded q-pa-sm column" style="min-height: 0;">
                <q-btn
                    outline
                    rounded
                    color="primary"
                    icon="add"
                    class="full-width q-mb-sm bg-dark"
                    :label="$t('workspaceAddDatasetFolder')"
                    @click="openAddDatasetForm"
                />

                <!-- Selection action bar -->
                <div
                    v-if="selectedEpisodeKeys.length"
                    class="q-mb-sm row items-center bg-dark border-rounded q-pa-xs"
                >
                    <div class="text-caption text-grey-5 q-mr-sm q-ml-sm">
                        {{ $t('datasetSelectedCount', { count: selectedEpisodeKeys.length }) }}
                    </div>
                    <q-space />
                    <q-btn
                        size="sm"
                        flat
                        dense
                        color="primary"
                        icon="content_copy"
                        :label="$t('datasetCopyEpisode')"
                        @click="openMoveOrCopyDialog('copy', selectedTransfers())"
                    />
                    <q-btn
                        size="sm"
                        flat
                        dense
                        color="primary"
                        icon="drive_file_move"
                        :label="$t('datasetMoveEpisode')"
                        @click="openMoveOrCopyDialog('move', selectedTransfers())"
                    />
                    <q-btn
                        size="sm"
                        flat
                        dense
                        color="negative"
                        icon="delete"
                        :label="$t('delete')"
                        @click="confirmBatchDelete"
                    />
                    <q-btn
                        size="sm"
                        flat
                        dense
                        color="grey-5"
                        icon="close"
                        @click="clearSelection"
                    />
                </div>

                <q-scroll-area class="col">
                    <q-list bordered separator dark class="border-rounded bg-dark">
                        <q-expansion-item
                            v-for="dataset in datasets"
                            :key="dataset.id"
                            expand-separator
                            dark
                            header-class="text-white"
                            :model-value="expandedDatasetIds.has(dataset.id)"
                            @show="onDatasetExpand(dataset)"
                            @hide="onDatasetCollapse(dataset)"
                            @dragover.prevent
                            @drop="onDropToDataset($event, dataset)"
                        >
                            <template v-slot:header>
                                <q-icon name="folder" class="q-mr-sm" size="md" color="grey-5" />
                                <div class="col">
                                    <div class="text-white">{{ dataset.name }} ({{ dataset.id }})</div>
                                    <div class="text-caption text-grey-5">
                                        {{ dataset.episodes.length }} {{ $t('datasetEpisodesSuffix') }}
                                    </div>
                                </div>
                                <q-menu context-menu>
                                    <q-list bordered separator dark class="bg-dark text-white">
                                        <q-item clickable v-ripple v-close-popup @click="openEditDatasetForm(dataset)">
                                            <q-item-section>{{ $t('workspaceDatasetEdit') }}</q-item-section>
                                            <q-item-section side><q-icon name="edit" size="xs" color="grey-5" /></q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="openAugmentationForm(dataset)">
                                            <q-item-section>{{ $t('workspaceDatasetAugment') }}</q-item-section>
                                            <q-item-section side><q-icon name="auto_fix_high" size="xs" color="grey-5" /></q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="openDownsampleForm(dataset)">
                                            <q-item-section>{{ $t('datasetDownsample') }}</q-item-section>
                                            <q-item-section side><q-icon name="compress" size="xs" color="grey-5" /></q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="openMergeDatasetForm(dataset)">
                                            <q-item-section>{{ $t('workspaceDatasetMerge') }}</q-item-section>
                                            <q-item-section side><q-icon name="merge_type" size="xs" color="grey-5" /></q-item-section>
                                        </q-item>
                                        <q-separator dark />
                                        <q-item clickable v-ripple class="text-negative" v-close-popup @click="deleteDataset(dataset)">
                                            <q-item-section>{{ $t('workspaceDatasetDelete') }}</q-item-section>
                                            <q-item-section side><q-icon color="negative" name="delete" size="xs" /></q-item-section>
                                        </q-item>
                                    </q-list>
                                </q-menu>
                                <q-space />
                            </template>

                            <q-list bordered separator dark dense v-if="expandedDatasetIds.has(dataset.id)">
                                <q-item
                                    v-for="episode in dataset.episodes"
                                    :key="`${dataset.id}_${episode.name}`"
                                    clickable
                                    v-ripple
                                    draggable="true"
                                    @dragstart="onEpisodeDragStart($event, dataset, episode)"
                                    @click="selectEpisode(dataset, episode)"
                                    :class="[
                                        'text-white',
                                        currentEpisode &&
                                        currentDatasetIdForViewer === dataset.id &&
                                        currentEpisode.name === episode.name
                                            ? 'bg-primary'
                                            : '',
                                    ]"
                                >
                                    <q-item-section side>
                                        <q-checkbox
                                            :model-value="isSelected(dataset.id, episode.name)"
                                            dark
                                            dense
                                            color="primary"
                                            @click.stop
                                            @update:model-value="(v) => toggleSelect(dataset.id, episode.name, v)"
                                        />
                                    </q-item-section>
                                    <q-item-section>
                                        <q-item-label>{{ episode.name }}</q-item-label>
                                        <q-item-label caption class="text-grey-5">
                                            {{ $t('datasetFrames', { count: episode.length || 0 }) }}
                                        </q-item-label>
                                    </q-item-section>

                                    <!-- Right-click context menu: Copy / Move / Delete -->
                                    <q-menu touch-position context-menu>
                                        <q-list bordered separator dark class="bg-dark text-white">
                                            <q-item dense class="text-caption text-grey-5">
                                                <q-item-section>
                                                    {{ contextScopeLabel(dataset.id, episode.name) }}
                                                </q-item-section>
                                            </q-item>
                                            <q-separator dark />
                                            <q-item
                                                clickable
                                                v-ripple
                                                v-close-popup
                                                @click="onContextAction('copy', dataset, episode)"
                                            >
                                                <q-item-section avatar>
                                                    <q-icon name="content_copy" color="grey-5" />
                                                </q-item-section>
                                                <q-item-section>{{ $t('datasetCopyEpisode') }}</q-item-section>
                                            </q-item>
                                            <q-item
                                                clickable
                                                v-ripple
                                                v-close-popup
                                                @click="onContextAction('move', dataset, episode)"
                                            >
                                                <q-item-section avatar>
                                                    <q-icon name="drive_file_move" color="grey-5" />
                                                </q-item-section>
                                                <q-item-section>{{ $t('datasetMoveEpisode') }}</q-item-section>
                                            </q-item>
                                            <q-separator dark />
                                            <q-item
                                                clickable
                                                v-ripple
                                                v-close-popup
                                                class="text-negative"
                                                @click="onContextAction('delete', dataset, episode)"
                                            >
                                                <q-item-section avatar>
                                                    <q-icon name="delete" color="negative" />
                                                </q-item-section>
                                                <q-item-section>{{ $t('delete') }}</q-item-section>
                                            </q-item>
                                        </q-list>
                                    </q-menu>
                                </q-item>
                                <q-item v-if="!dataset.episodes.length">
                                    <q-item-section class="text-grey-5">
                                        {{ $t('datasetEmpty') }}
                                    </q-item-section>
                                </q-item>
                            </q-list>
                        </q-expansion-item>
                    </q-list>
                </q-scroll-area>
            </div>

            <!-- RIGHT: Episode viewer -->
            <div class="col bg-secondary border-rounded q-pa-md column" style="min-width: 0; min-height: 0; overflow: auto;">
                <div
                    v-if="!currentEpisode"
                    class="flex flex-center text-grey-5 text-h6"
                    style="min-height: 400px;"
                >
                    {{ $t('datasetSelectEpisodeHint') }}
                </div>
                <episode-panel
                    v-else
                    :dataset-id="currentDatasetIdForViewer"
                    :episode="currentEpisode"
                    :key="`${currentDatasetIdForViewer}_${currentEpisode.name}`"
                    @close="currentEpisode = null"
                    @trim="onTrimEpisode"
                    @save-language="onSaveLanguage"
                />
            </div>
        </div>

        <!-- Dialogs -->
        <form-dialog
            v-model="showDatasetForm"
            :title="$t(datasetForm.find(f => f.key === 'id').value ? 'datasetEditFormTitle' : 'datasetAddFormTitle')"
            :form="datasetForm"
            @submit="saveDataset"
            :ok-button-label="$t(datasetForm.find(f => f.key === 'id').value ? 'save' : 'add')"
        />

        <form-dialog
            v-model="showMergeDatasetForm"
            :title="$t('mergeDatasetFormTitle')"
            :form="mergeDatasetForm"
            @submit="mergeDatasets"
            :ok-button-label="$t('save')"
        />

        <form-dialog
            v-model="showDownsampleForm"
            :title="$t('datasetDownsampleTitle')"
            :form="downsampleForm"
            @submit="downsampleDataset"
            :ok-button-label="$t('datasetDownsample')"
        />

        <q-dialog v-model="showAugmentationForm" persistent full-width>
            <data-augmentation-dialog
                :dataset="augmentingDataset"
                :task-id="selectedWorkspaceId"
                v-if="selectedWorkspaceId"
            />
        </q-dialog>

        <!-- Move/copy target picker -->
        <q-dialog v-model="showMoveCopyDialog">
            <q-card class="bg-dark text-white" style="min-width: 380px;">
                <q-card-section>
                    <div class="text-h6">
                        {{ moveCopyMode === 'move' ? $t('datasetMoveDialogTitle') : $t('datasetCopyDialogTitle') }}
                    </div>
                    <div class="text-caption text-grey-5">
                        {{ $t('datasetSelectedCount', { count: pendingTransferEpisodes.length }) }}
                    </div>
                </q-card-section>
                <q-card-section>
                    <q-select
                        v-model="moveCopyTargetId"
                        :options="moveCopyTargetOptions"
                        outlined
                        dark
                        color="grey-5"
                        bg-color="secondary"
                        emit-value
                        map-options
                        :label="$t('datasetMoveTargetLabel')"
                        input-class="text-white"
                        label-color="grey-5"
                    />
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat color="grey-5" :label="$t('cancel')" v-close-popup />
                    <q-btn
                        flat
                        color="primary"
                        :label="moveCopyMode === 'move' ? $t('datasetMoveEpisode') : $t('datasetCopyEpisode')"
                        :disable="!moveCopyTargetId"
                        @click="confirmMoveOrCopy"
                    />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue';
import { Notify, Loading } from 'quasar';
import { useI18n } from 'vue-i18n';
import { api } from 'src/boot/axios';
import { useSocket } from 'src/composables/useSocket';
import FormDialog from 'src/components/v2/FormDialog.vue';
import DataAugmentationDialog from 'src/components/v2/DataAugmentationDialog.vue';
import EpisodePanel from 'src/components/v2/EpisodePanel.vue';

const { t } = useI18n();
const { socket } = useSocket();

// ─── Workspaces ─────────────────────────────────────────────────────────────
const workspaces = ref([]);
const selectedWorkspaceId = ref(null);
const pageLoading = ref(true);

function listWorkspaces() {
    return api.get('/tasks').then((res) => {
        workspaces.value = res.data.tasks || [];
    });
}

watch(selectedWorkspaceId, (val) => {
    expandedDatasetIds.value = new Set();
    currentEpisode.value = null;
    currentDatasetIdForViewer.value = null;
    selectedEpisodeKeys.value = [];
    if (val) listDatasets();
});

// ─── Datasets ───────────────────────────────────────────────────────────────
const datasets = ref([]);
const expandedDatasetIds = ref(new Set());

function listDatasets() {
    return api
        .get('/datasets', { params: { task_id: selectedWorkspaceId.value } })
        .then((res) => {
            datasets.value = res.data.datasets || [];
        })
        .catch((err) => {
            console.error('Error fetching datasets:', err);
            Notify.create({ color: 'negative', message: t('trainLoadDatasetsFailed') });
        });
}

function onDatasetExpand(dataset) {
    if (expandedDatasetIds.value.has(dataset.id)) return;
    const next = new Set(expandedDatasetIds.value);
    next.add(dataset.id);
    expandedDatasetIds.value = next;
}

function onDatasetCollapse(dataset) {
    if (!expandedDatasetIds.value.has(dataset.id)) return;
    const next = new Set(expandedDatasetIds.value);
    next.delete(dataset.id);
    expandedDatasetIds.value = next;
}

// ─── Episode selection (multi-select) ──────────────────────────────────────
const selectedEpisodeKeys = ref([]);

function selKey(datasetId, name) {
    return `${datasetId}::${name}`;
}

function isSelected(datasetId, name) {
    return selectedEpisodeKeys.value.includes(selKey(datasetId, name));
}

function toggleSelect(datasetId, name, val) {
    const key = selKey(datasetId, name);
    if (val && !selectedEpisodeKeys.value.includes(key)) {
        selectedEpisodeKeys.value.push(key);
    } else if (!val) {
        selectedEpisodeKeys.value = selectedEpisodeKeys.value.filter((k) => k !== key);
    }
}

function clearSelection() {
    selectedEpisodeKeys.value = [];
}

function selectedTransfers() {
    return selectedEpisodeKeys.value.map((k) => {
        const [dsId, name] = k.split('::');
        return { dataset_id: Number(dsId), name };
    });
}

// ─── Episode viewer ────────────────────────────────────────────────────────
const currentEpisode = ref(null);
const currentDatasetIdForViewer = ref(null);

function selectEpisode(dataset, episode) {
    currentDatasetIdForViewer.value = dataset.id;
    currentEpisode.value = episode;
}

// ─── Right-click context menu helpers ─────────────────────────────────────
function targetsForContext(dataset, episode) {
    // If the right-clicked episode is checked, apply to ALL checked episodes.
    // Otherwise, apply to just this episode.
    if (isSelected(dataset.id, episode.name) && selectedEpisodeKeys.value.length) {
        return selectedTransfers();
    }
    return [{ dataset_id: dataset.id, name: episode.name }];
}

function contextScopeLabel(datasetId, name) {
    if (isSelected(datasetId, name) && selectedEpisodeKeys.value.length > 1) {
        return t('datasetSelectedCount', { count: selectedEpisodeKeys.value.length });
    }
    return name;
}

function onContextAction(action, dataset, episode) {
    const targets = targetsForContext(dataset, episode);
    if (action === 'delete') {
        confirmDeleteTargets(targets);
    } else {
        openMoveOrCopyDialog(action, targets);
    }
}

// ─── Episode delete (single + batch unified) ──────────────────────────────
function confirmDeleteTargets(targets) {
    if (!targets.length) return;
    const message =
        targets.length === 1
            ? t('datasetConfirmDeleteEpisode', { name: targets[0].name })
            : t('datasetConfirmBatchDelete', { count: targets.length });
    Notify.create({
        message,
        color: 'negative',
        timeout: 0,
        actions: [
            { label: t('cancel'), color: 'white' },
            {
                label: t('delete'),
                color: 'white',
                handler: () => doDeleteTargets(targets),
            },
        ],
    });
}

async function doDeleteTargets(targets) {
    if (targets.length === 1) {
        const { dataset_id, name } = targets[0];
        try {
            await api.delete(`/dataset/${dataset_id}/${name}`);
            if (
                currentEpisode.value?.name === name &&
                currentDatasetIdForViewer.value === dataset_id
            ) {
                currentEpisode.value = null;
            }
            await listDatasets();
        } catch (err) {
            console.error(err);
            Notify.create({ color: 'negative', message: t('datasetDeleteFailed') });
        }
        return;
    }
    // Batch
    const groups = {};
    for (const ep of targets) {
        if (!groups[ep.dataset_id]) groups[ep.dataset_id] = [];
        groups[ep.dataset_id].push(ep.name);
    }
    Loading.show();
    try {
        for (const [dsId, names] of Object.entries(groups)) {
            await api.post(`/dataset/${dsId}/:batch_delete`, { episode_indices: names });
        }
        clearSelection();
        if (currentEpisode.value) {
            const ds = datasets.value.find((d) => d.id === currentDatasetIdForViewer.value);
            const stillExists = ds?.episodes.some((e) => e.name === currentEpisode.value.name);
            if (!stillExists) currentEpisode.value = null;
        }
        await listDatasets();
        Notify.create({
            color: 'positive',
            message: t('datasetBatchDeleteDone', { count: targets.length }),
        });
    } catch (err) {
        console.error(err);
        Notify.create({ color: 'negative', message: t('datasetBatchDeleteFailed') });
    } finally {
        Loading.hide();
    }
}

function confirmBatchDelete() {
    confirmDeleteTargets(selectedTransfers());
}

// ─── Trim ──────────────────────────────────────────────────────────────────
async function onTrimEpisode({ start, end }) {
    if (!currentEpisode.value || !currentDatasetIdForViewer.value) return;
    Loading.show({ message: t('datasetTrimInProgress') });
    try {
        await api.post(
            `/dataset/${currentDatasetIdForViewer.value}/${currentEpisode.value.name}/:trim`,
            { start, end },
        );
        await listDatasets();
        const ds = datasets.value.find((d) => d.id === currentDatasetIdForViewer.value);
        const ep = ds?.episodes.find((e) => e.name === currentEpisode.value.name);
        currentEpisode.value = ep || null;
        Notify.create({ color: 'positive', message: t('datasetTrimDone') });
    } catch (err) {
        console.error(err);
        Notify.create({ color: 'negative', message: t('datasetTrimFailed') });
    } finally {
        Loading.hide();
    }
}

// ─── Save language ─────────────────────────────────────────────────────────
async function onSaveLanguage(language) {
    if (!currentEpisode.value || !currentDatasetIdForViewer.value) return;
    try {
        await api.post(
            `/dataset/${currentDatasetIdForViewer.value}/${currentEpisode.value.name}/:set_language`,
            { language_instruction: language },
        );
        Notify.create({ color: 'positive', message: t('datasetLanguageSaved') });
    } catch (err) {
        console.error(err);
        Notify.create({ color: 'negative', message: t('datasetLanguageSaveFailed') });
    }
}

// ─── Move / Copy ───────────────────────────────────────────────────────────
const showMoveCopyDialog = ref(false);
const moveCopyMode = ref('copy');
const moveCopyTargetId = ref(null);
const pendingTransferEpisodes = ref([]);

const moveCopyTargetOptions = computed(() => {
    // copy: source 폴더 포함 모든 데이터셋 (현재 폴더 선택 시 같은 폴더에 복제됨).
    // move: source 폴더는 의미 없으므로 제외.
    if (moveCopyMode.value === 'copy') {
        return datasets.value.map((d) => ({ label: `${d.name} (${d.id})`, value: d.id }));
    }
    const sourceIds = new Set(pendingTransferEpisodes.value.map((p) => Number(p.dataset_id)));
    return datasets.value
        .filter((d) => !sourceIds.has(d.id))
        .map((d) => ({ label: `${d.name} (${d.id})`, value: d.id }));
});

function openMoveOrCopyDialog(mode, targets) {
    if (!targets || !targets.length) return;
    moveCopyMode.value = mode;
    moveCopyTargetId.value = null;
    pendingTransferEpisodes.value = targets;
    showMoveCopyDialog.value = true;
}

async function confirmMoveOrCopy() {
    if (!moveCopyTargetId.value || !pendingTransferEpisodes.value.length) return;
    Loading.show();
    try {
        const groups = {};
        for (const ep of pendingTransferEpisodes.value) {
            if (!groups[ep.dataset_id]) groups[ep.dataset_id] = [];
            groups[ep.dataset_id].push(ep.name);
        }
        const endpoint = moveCopyMode.value === 'move' ? ':batch_move' : ':batch_copy';
        for (const [srcId, names] of Object.entries(groups)) {
            await api.post(`/dataset/${endpoint}`, {
                source_dataset_id: Number(srcId),
                target_dataset_id: moveCopyTargetId.value,
                episode_indices: names,
            });
        }
        showMoveCopyDialog.value = false;
        clearSelection();
        if (moveCopyMode.value === 'move' && currentEpisode.value) {
            const isMoved = pendingTransferEpisodes.value.some(
                (p) =>
                    Number(p.dataset_id) === currentDatasetIdForViewer.value &&
                    p.name === currentEpisode.value.name,
            );
            if (isMoved) currentEpisode.value = null;
        }
        await listDatasets();
        Notify.create({
            color: 'positive',
            message:
                moveCopyMode.value === 'move'
                    ? t('datasetMoveDone')
                    : t('datasetCopyDone'),
        });
    } catch (err) {
        console.error(err);
        Notify.create({ color: 'negative', message: t('datasetTransferFailed') });
    } finally {
        Loading.hide();
    }
}

// ─── Drag & drop ───────────────────────────────────────────────────────────
function onEpisodeDragStart(ev, dataset, episode) {
    const key = selKey(dataset.id, episode.name);
    let payload;
    if (selectedEpisodeKeys.value.includes(key)) {
        payload = selectedTransfers();
    } else {
        payload = [{ dataset_id: dataset.id, name: episode.name }];
    }
    ev.dataTransfer.effectAllowed = 'copyMove';
    ev.dataTransfer.setData('application/json', JSON.stringify(payload));
}

async function onDropToDataset(ev, targetDataset) {
    const raw = ev.dataTransfer.getData('application/json');
    if (!raw) return;
    let payload;
    try {
        payload = JSON.parse(raw);
    } catch {
        return;
    }
    const filtered = payload.filter((p) => Number(p.dataset_id) !== Number(targetDataset.id));
    if (!filtered.length) return;

    const isCopy = ev.ctrlKey || ev.metaKey;
    pendingTransferEpisodes.value = filtered;
    moveCopyMode.value = isCopy ? 'copy' : 'move';
    moveCopyTargetId.value = targetDataset.id;
    await confirmMoveOrCopy();
}

// ─── Add / edit dataset ────────────────────────────────────────────────────
const showDatasetForm = ref(false);
const datasetForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('datasetName'), type: 'text', value: '', default: '' },
]);

function openAddDatasetForm() {
    datasetForm.value.forEach((f) => (f.value = f.default ?? null));
    showDatasetForm.value = true;
}
function openEditDatasetForm(dataset) {
    datasetForm.value.forEach((f) => (f.value = dataset[f.key] ?? f.default ?? null));
    datasetForm.value.find((f) => f.key === 'id').value = dataset.id;
    showDatasetForm.value = true;
}

async function saveDataset(form) {
    try {
        const payload = {};
        datasetForm.value.forEach((f) => (payload[f.key] = f.value));
        const finalData = { ...form, ...payload };
        const id = datasetForm.value.find((f) => f.key === 'id').value;
        if (id) {
            await api.put(`/dataset/${id}`, finalData);
        } else {
            await api.post('/dataset', { ...finalData, task_id: selectedWorkspaceId.value });
        }
        showDatasetForm.value = false;
        await listDatasets();
    } catch (err) {
        console.error(err);
        Notify.create({ color: 'negative', message: t('datasetSaveFailed') });
    }
}

function deleteDataset(dataset) {
    Notify.create({
        message: t('workspaceConfirmDeleteDataset', { name: dataset.name }),
        color: 'negative',
        timeout: 0,
        actions: [
            { label: t('cancel'), color: 'white' },
            {
                label: t('delete'),
                color: 'white',
                handler: async () => {
                    await api.delete(`/dataset/${dataset.id}`);
                    if (expandedDatasetIds.value.has(dataset.id)) {
                        const next = new Set(expandedDatasetIds.value);
                        next.delete(dataset.id);
                        expandedDatasetIds.value = next;
                    }
                    if (currentDatasetIdForViewer.value === dataset.id) {
                        currentEpisode.value = null;
                        currentDatasetIdForViewer.value = null;
                    }
                    await listDatasets();
                },
            },
        ],
    });
}

// ─── Augmentation ──────────────────────────────────────────────────────────
const showAugmentationForm = ref(false);
const augmentingDataset = ref({});

function openAugmentationForm(dataset) {
    if (!dataset.episodes.length) {
        Notify.create({ color: 'negative', message: t('workspaceAugmentNeedsEpisodes') });
        return;
    }
    augmentingDataset.value = dataset;
    showAugmentationForm.value = true;
}

// ─── Merge ─────────────────────────────────────────────────────────────────
const showMergeDatasetForm = ref(false);
const mergeDatasetForm = ref([
    {
        key: 'source_dataset_id',
        label: t('mergeDatasetSourceLabel'),
        type: 'select',
        options: computed(() => datasets.value.map((d) => ({ label: d.name, value: d.id }))),
        value: null,
        default: null,
    },
    {
        key: 'target_dataset_ids',
        label: t('mergeDatasetTargetsLabel'),
        type: 'multiselect_list',
        options: computed(() => datasets.value.map((d) => ({ label: d.name, value: d.id }))),
        value: [],
        default: [],
    },
]);

function openMergeDatasetForm(dataset) {
    mergeDatasetForm.value.forEach((f) => (f.value = f.default));
    mergeDatasetForm.value.find((f) => f.key === 'source_dataset_id').value = dataset.id;
    showMergeDatasetForm.value = true;
}

async function mergeDatasets(form) {
    Loading.show();
    try {
        await api.post('/dataset/:merge', form);
        await listDatasets();
    } finally {
        Loading.hide();
    }
}

// ─── Downsample ────────────────────────────────────────────────────────────
const showDownsampleForm = ref(false);
const downsamplingDatasetId = ref(null);
const downsampleForm = ref([
    { key: 'name', label: t('datasetName'), type: 'text', value: '', default: '' },
    { key: 'keep', label: t('datasetDownsampleKeep'), type: 'number', value: 1, default: 1 },
    { key: 'every', label: t('datasetDownsampleEvery'), type: 'number', value: 2, default: 2 },
]);

function openDownsampleForm(dataset) {
    downsamplingDatasetId.value = dataset.id;
    downsampleForm.value.forEach((f) => (f.value = f.default));
    downsampleForm.value.find((f) => f.key === 'name').value = `${dataset.name}_downsampled`;
    showDownsampleForm.value = true;
}

async function downsampleDataset(form) {
    if (!downsamplingDatasetId.value) return;
    try {
        await api.post(`/dataset/${downsamplingDatasetId.value}/downsample`, {
            ...form,
            task_id: selectedWorkspaceId.value,
        });
        showDownsampleForm.value = false;
        Notify.create({ color: 'positive', message: t('datasetDownsampleStarted') });
    } catch (err) {
        console.error(err);
        Notify.create({ color: 'negative', message: t('datasetDownsampleFailed') });
    }
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────
onMounted(async () => {
    pageLoading.value = true;
    try {
        await listWorkspaces();
    } finally {
        pageLoading.value = false;
    }
    socket.on('augmentation_complete', () => {
        showAugmentationForm.value = false;
        listDatasets();
    });
    socket.on('episode_added', () => {
        listDatasets();
    });
});
</script>

<style scoped>
.q-expansion-item__container > .q-item:hover {
    background: rgba(255, 255, 255, 0.05);
}
</style>
