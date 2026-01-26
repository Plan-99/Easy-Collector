<template>
    <q-page class="q-pt-lg q-pr-lg full-height column">
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <!-- <div class="float-right">
                    <q-toggle
                        class="text-white"
                        label="Failure Detector"
                        :model-value="processStore.isRunning('failure_detection')"
                        @update:model-value="toggleFailureDetection()"
                        dense
                    ></q-toggle>
                    <div class="row q-mt-sm" v-if="processStore.isRunning('failure_detection')">
                        <div class="text-white">Uncertainty Score:</div>
                        <div
                            class="q-ml-sm"
                            :class="isFailureDetected ? 'text-red text-bold' : 'text-green text-bold'"
                        >{{ uncertaintyScore }}</div>
                    </div>
                </div> -->
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-lg">{{ $t('workspaceIntroTitle') }}</div>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="selectedWorkspaceId"
                        :options="workspaces"
                        label="Select Workspace"
                        style="width: 400px"
                        class="q-ml-lg"
                        map-options
                        emit-value
                        option-label="name"
                        option-value="id"
                    >
                        <template v-slot:option="scope">
                            <q-item :props="scope.props" v-if="scope.opt.id === 'new'" v-bind="scope.itemProps" @click="openCreateWorkspaceForm">
                                <q-item-section>
                                    <q-item-label class="text-yellow">{{ scope.opt.name }}</q-item-label>
                                </q-item-section>
                            </q-item>
                            <q-item :props="scope.props" v-bind="scope.itemProps" v-else>
                                <q-item-section>
                                    <q-item-label>{{ scope.opt.name }}</q-item-label>
                                </q-item-section>
                                <q-item-section side>
                                    <div class="row">
                                        <q-btn icon="edit" size="sm" flat round @click.stop="openEditWorkspaceForm(scope.opt)" />
                                        <q-btn icon="delete" size="sm" color="red" flat round @click.stop="deleteWorkspace(scope.opt)" />
                                    </div>
                                </q-item-section>
                            </q-item>   
                        </template>
                    </q-select>
                </div>
                <div class="text-body text-white">{{ $t('workspaceIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('workspaceIntroBody2') }}</div>
            </div>
        </div>
        <div class="col q-mb-lg border-rounded border-grey text-grey flex-center flex text-h6" v-if="!selectedWorkspaceId">
            Select Workspace First
        </div>
        <div class="col row q-mb-lg" v-else>
            <div class="col-3 bg-secondary q-mr-lg border-rounded q-pa-sm" v-if="status === 'pending'">
                <q-tabs
                    dense
                    v-model="selectedTab"
                    :breakpoint="0"
                    class="text-white"
                    active-color="primary"
                >
                    <q-tab name="setting" label="setting"></q-tab>
                    <q-tab name="data" label="data"></q-tab>
                    <q-tab name="inference" label="inference"></q-tab>
                </q-tabs>
                <div v-if="selectedTab === 'setting'" class="q-pt-md q-px-sm text-white">
                    <q-list dark bordered separator class="border-rounded bg-dark" >
                        <q-expansion-item
                            icon="camera"
                            :label="`${$t('sensorSetting')} (${selectedWorkspaceSensors.length})`"
                        >
                            <q-card class="bg-dark">
                                <q-card-section>
                                    <div
                                        class="q-pa-sm q-px-md q-my-sm border-rounded row"
                                        v-for="sensor in selectedWorkspaceSensors" 
                                        :key="sensor.id"
                                        :class="sensor.status === 'on' ? 'bg-green-10' : 'bg-grey-8'"
                                    >
                                        <div>{{ sensor.name }}</div>
                                        <q-space />
                                        <q-toggle
                                            :model-value="sensor.status === 'on'"
                                            dense
                                            color="positive"
                                            @click="toggleSensor(sensor)"
                                            v-if="sensor.type !== 'custom'"
                                        ></q-toggle>
                                        <div v-else>
                                            <div class="text-caption text-grey" v-if="sensor.status === 'off'">TOPIC OFF</div>
                                            <div class="text-caption text-positive" v-else-if="sensor.status === 'on'">TOPIC ON</div>
                                        </div>
                                        <q-inner-loading :showing="sensor.status === 'loading'"></q-inner-loading>
                                    </div>
                                    <q-btn outline
                                        class="full-width"
                                        rounded
                                        color="primary"
                                        icon="add"
                                        @click="openSensorForm"
                                    ></q-btn>
                                    <div class="row q-gutter-x-sm q-mt-md q-mb-sm">
                                        <q-input
                                            dense outlined dark bg-color="dark"
                                            label="Common Width"
                                            class="col"
                                            v-model.number="commonSensorResolution.width"
                                            @update:model-value="updateAllSensorResolutions"
                                        ></q-input>

                                        <q-input
                                            dense outlined dark bg-color="dark"
                                            label="Common Height"
                                            class="col"
                                            v-model.number="commonSensorResolution.height"
                                            @update:model-value="updateAllSensorResolutions"
                                        ></q-input>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                        <q-expansion-item
                            icon="adb"
                            :label="`${$t('robotSetting')}`"
                        >
                            <q-card class="bg-dark border-rounded">
                            <q-card-section>
                                <q-select
                                    dense
                                    outlined
                                    dark
                                    bg-color="dark"
                                    v-model="selectedWorkspace.assembly_id"
                                    :options="assemblies"
                                    label="Robot Assembly"
                                    style="width: 100%"
                                    map-options
                                    emit-value
                                    option-label="name"
                                    option-value="id"
                                    @update:model-value="updateWorkspace({ assembly_id: selectedWorkspace.assembly_id })"
                                ></q-select>
                                <div
                                    class="q-pa-sm q-px-md q-my-sm border-rounded row"
                                    v-for="robot in robots" 
                                    :key="robot.id"
                                    :class="robot.status === 'on' ? 'bg-green-10' : (robot.status === 'error' ? 'bg-red-10' : 'bg-grey-8')"
                                >
                                    <div>{{ robot.name }}</div>
                                    <q-space />
                                    <q-toggle
                                        :model-value="robot.status === 'on'"
                                        dense
                                        color="positive"
                                        @click="toggleRobot(robot)"
                                        v-if="robot.type !== 'custom'"
                                    ></q-toggle>
                                    <div v-else>
                                        <div class="text-caption text-grey" v-if="robot.status === 'off'">TOPIC OFF</div>
                                        <div class="text-caption text-positive" v-else-if="robot.status === 'on'">TOPIC ON</div>
                                        <div class="text-caption text-negative" v-else-if="robot.status === 'error'">ERROR</div>
                                    </div>
                                    <q-inner-loading :showing="robot.status === 'loading'"></q-inner-loading>
                                </div>
                            </q-card-section>
                            </q-card>
                        </q-expansion-item>
                        <q-expansion-item
                            icon="cleaning_services"
                            :label="`${$t('taskSetting')}`"
                        >
                            <q-card class="bg-dark border-rounded">
                                <q-card-section>
                                    <div class="row q-gutter-x-sm">
                                        <q-input
                                            dense
                                            outlined
                                            dark
                                            bg-color="dark"
                                            label="Episode Length"
                                            class="col"
                                            v-model.number="selectedWorkspace.episode_len"
                                            @change="updateWorkspace({ episode_len: selectedWorkspace.episode_len })"
                                        ></q-input>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
                    <div v-if="focused.id" class="q-mt-md text-white h6">
                        <div>{{ $t(`${focused.device_type} Config`) }} <span class="text-primary">{{ focused.name }}</span></div>
                        <div
                            class="q-pa-sm q-px-md q-mt-sm border-rounded row border-white"
                            v-if="focused.device_type === 'sensor'"
                        >
                            <div class="text-caption col-12">
                                You can crop the streaming here!
                                <q-btn size="xs" outline color="pink-3" icon="sync" @click="resetCroppedArea()" class="q-ml-sm" />
                                <!-- <q-btn size="xs" outline color="primary" icon="save" @click="saveCroppedArea" class="q-ml-sm" /> -->
                            </div>
                            <div
                                class="relative-position"
                                style="min-width: 200px; min-height: 200px; border-color: blue;"
                                @mousedown="startCrop"
                                @mousemove="doCrop"
                                @mouseup="endCrop"
                                @mouseleave="cancelCrop"
                                ref="videoContainer"
                            >
                                <web-rtc-video
                                    :process-id="`sensor_${focused.id}`"
                                    :topic="focused.read_topic"
                                    class="border-rounded"
                                    :key="focused.id"
                                    :loading="focused.status !== 'on'"
                                    v-if="focused.status !== 'off'"
                                    show_original_video
                                ></web-rtc-video>
                                <div
                                    class="crop-area"
                                    :style="cropAreaStyle"
                                    v-if="cropAreaStyle.width"
                                ></div>
                            </div>
                            <div>
                                <div class="text-caption q-mt-sm">Cropped Area</div>
                                <div>
                                    {{ selectedWorkspace.sensor_settings?.[focused.id]?.cropped_area || 'Not Set' }}
                                </div>
                                <div class="text-caption q-mt-sm">Cropped Size</div>
                                <div>
                                    {{ selectedWorkspace.sensor_settings?.[focused.id]?.cropped_area ? `${selectedWorkspace.sensor_settings?.[focused.id]?.cropped_area[2] - selectedWorkspace.sensor_settings?.[focused.id]?.cropped_area[0]} x ${selectedWorkspace.sensor_settings?.[focused.id]?.cropped_area[3] - selectedWorkspace.sensor_settings?.[focused.id]?.cropped_area[1]}` : 'Not Set' }}
                                </div>
                            </div>
                        </div>
                        <div
                            class="q-pa-sm q-px-md q-mt-sm border-rounded border-white"
                            v-else-if="focused.device_type === 'robot'"
                        >
                            <div class="text-caption q-mb-xs row q-gutter-x-sm">
                                <div>Home Pose</div>
                                <q-space></q-space>
                                <q-btn size="xs" outline color="pink-3" icon="sync" @click="selectedWorkspace.home_pose[focused.id] = [...focused.jointState]" />
                                <q-btn size="xs" outline color="primary" icon="save" @click="updateWorkspaceDeviceSetting({
                                    device_type: 'robots',
                                    key: 'home_pose',
                                    setting: selectedWorkspace.home_pose,
                                })" />
                            </div>
                            <div class="row q-col-gutter-sm">
                                <q-input
                                    dense
                                    outlined
                                    dark
                                    bg-color="dark"
                                    :label="val"
                                    class="col-6"
                                    v-model.number="selectedWorkspace.home_pose[focused.id][index]"
                                    v-for="(val, index) in focused.joint_names"
                                    :key="index"
                                ></q-input>
                            </div>
                        </div>
                    </div>
                </div>
                <div
                    v-else-if="selectedTab === 'data'"
                    class="q-pt-md q-px-sm text-white"
                    style="height: 90%"
                >
                    <q-btn
                        outline
                        class="full-width q-mb-sm"
                        rounded
                        color="primary bg-dark"
                        icon="add"
                        label="Add Dataset Folder"
                        @click="openAddDatasetForm"
                    ></q-btn>
                    <q-scroll-area class="full-height">
                        <q-list bordered separator class="border-rounded bg-dark" dark>
                            <q-expansion-item
                                expand-separator
                                icon="folder"
                                :label="dataset.name"
                                :caption="`${dataset.episodes.length} episodes`"
                                v-for="dataset in datasets"
                                :key="dataset.id"
                                class="text-white"
                                dark
                                :model-value="selectedDatasetId === dataset.id"
                                @show="selectedDatasetId = dataset.id"
                                header-class="items-center"
                            >
                                <template v-slot:header>
                                    <q-icon name="folder" class="q-mr-lg" size="lg"></q-icon>
                                    <div class="col">
                                        <div>{{ dataset.name }}</div>
                                        <div class="text-caption">{{ dataset.episodes.length }} episodes</div>
                                         <q-menu context-menu>
                                            <q-list bordered separator>
                                                <q-item clickable v-ripple v-close-popup @click="openEditDatasetForm(dataset)">
                                                    <q-item-section>Edit Dataset</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon name="edit" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                                <q-item clickable v-ripple v-close-popup 
                                                    @click="openAugmentationForm(dataset)">
                                                    <q-item-section>Augment Dataset</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon name="edit" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                                <q-item clickable v-ripple class="text-negative" @click="deleteDataset(dataset)">
                                                    <q-item-section>Delete Dataset</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon color="negative" name="delete" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                            </q-list>
                                        </q-menu>
                                    </div>
                                    <q-space class="col"></q-space>
                                </template>
                                <q-list bordered separator dark dense v-if="selectedDatasetId === dataset.id">
                                    <q-item
                                        v-for="episode in dataset.episodes"
                                        :key="episode.name"
                                        clickable
                                        v-ripple
                                        @click="selectEpisode(episode)"
                                        :class="{ 'bg-primary': selectedEpisode.name === episode.name && selectedDataset && selectedDataset.id === dataset.id }"
                                    >
                                        <q-item-section>
                                            <q-item-label class="q-ml-xl">{{ episode.name }}</q-item-label>
                                        </q-item-section>
                                        <q-item-section side>
                                            <q-btn
                                                icon="close"
                                                size="sm"
                                                flat
                                                round
                                                @click.stop="deleteFile(dataset.id, episode)"
                                            />
                                        </q-item-section>
                                    </q-item>
                                </q-list>
                            </q-expansion-item>
                        </q-list>
                    </q-scroll-area>
                </div>
                <div v-else-if="selectedTab === 'inference'" style="height: 90%" class="q-pt-md q-px-sm text-white">
                    <q-scroll-area class="full-height">
                        <div class="row q-col-gutter-md">
                        <div class="col-6" v-for="checkpoint in checkpoints" :key="checkpoint.id">
                            <q-card
                                class="q-pa-md bg-dark border-rounded border-white text-white"
                                :class="selectedCheckpointId === checkpoint.id ? 'border-primary' : ''"
                                @click="selectedCheckpointId = checkpoint.id"
                            >
                                <q-menu context-menu>
                                    <q-list bordered separator>
                                        <q-item
                                            clickable
                                            v-ripple
                                            v-close-popup
                                            @click="openCheckpointInfoDialog(checkpoint)"
                                        >
                                            <q-item-section>Show Details</q-item-section>
                                            <q-item-section side>
                                                <q-icon name="add" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="openCheckpointForm(checkpoint)">
                                            <q-item-section>Edit Checkpoint</q-item-section>
                                            <q-item-section side>
                                                <q-icon name="edit" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple class="text-negative" @click="deleteCheckpoint(checkpoint)">
                                            <q-item-section>Delete Checkpoint</q-item-section>
                                            <q-item-section side>
                                                <q-icon color="negative" name="delete" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                    </q-list>
                                </q-menu>
                                <q-card-section class="q-pa-none q-mt-sm text-center">
                                    <q-img
                                        src="images/brain-icon.png"
                                        class="cursor-pointer"
                                        fit="contain"
                                        width="80px"
                                    >
                                    </q-img>
                                    <div class="text-bold q-mt-md">{{ checkpoint.name }}</div>
                                </q-card-section>
                            </q-card>
                        </div>
                    </div>
                    </q-scroll-area>
                </div>
            </div>
            <monitoring-window
                class="col"
                :workspace="selectedWorkspace"
                :robots="robots"
                :sensors="selectedWorkspaceSensors"
                v-model:selected-dataset-id="selectedDatasetId"
                v-model:selected-checkpoint-id="selectedCheckpointId"
                v-model:focused="focused"
                v-model:selected-episode="selectedEpisode"
                :datasets="datasets"
                :checkpoints="checkpoints"
                :status="status"
                :class="isFailureDetected ? 'border-red' : 'border-white'"
            />
        </div>
        <form-dialog
            v-model="showWorkspaceForm"
            :title="$t(workspaceForm.find((e) => e.key === 'id').value ? 'workerkspaceEditFormTitle' : 'workspaceCreateFormTitle')"
            :form="workspaceForm"
            :ok-button-label="$t(workspaceForm.find((e) => e.key === 'id').value ? 'save' : 'create')"
            :cancel-button-label="$t('cancel')"
            @submit="saveWorkspace"
        ></form-dialog>
        <form-dialog
            v-model="showSensorForm"
            :title="$t('workspaceSensorFormTitle')"
            :form="sensorForm"
            @submit="saveSensorSettings"
            :ok-button-label="$t('save')"
        ></form-dialog>
        <form-dialog
            v-model="showDatasetForm"
            :title="$t(datasetForm.id ? 'datasetEditFormTitle' : 'datasetAddFormTitle')"
            :form="datasetForm"
            @submit="saveDataset"
            :ok-button-label="$t(datasetForm.id ? 'save' : 'add')"
        ></form-dialog>
        <form-dialog
            v-model="showCheckpointForm"
            :title="$t('checkpointEditFormTitle')"
            :form="checkpointForm"
            @submit="saveCheckpoint"
            :ok-button-label="$t('save')"
        ></form-dialog>

        <q-dialog
            v-model="showAugmentationForm"
            maximized
            persistent
        >
            <data-augmentation-dialog
                :dataset="augmentingDataset"
                :task-id="selectedWorkspaceId"
                v-if="selectedWorkspaceId"
            />
        </q-dialog>

        <q-dialog v-model="showCheckpointInfo" full-width>
            <q-card class="bg-secondary border-rounded border-white" dark>
                <q-card-section class="bg-dark text-white row">
                    <div class="text-h6">{{ $t('checkpointInfo') }}</div>
                    <q-space></q-space>
                    <q-btn dense color="white" round icon="close" text-color="dark" @click="showCheckpointInfo = false"/>
                </q-card-section>

                <q-separator />
                <q-card-section class="bg-secondary q-px-xl q-py-lg">
                    <checkpoint-info
                        :checkpoint="selectedCheckpoint"
                        :height="400"
                    />
                </q-card-section>
                <q-card-section class="bg-secondary q-px-xl q-pt-none" align="right">
                    <div>Loss: {{ selectedCheckpoint.loss.toFixed(4) }} / Best Epoch: {{ selectedCheckpoint.best_epoch }}</div>
                </q-card-section>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import FormDialog from 'src/components/v2/FormDialog.vue';
import { useSensor } from '../../composables/useSensor';
import { useRobot } from 'src/composables/useRobot';
import { Notify } from 'quasar';
import { useI18n } from 'vue-i18n';
import DataAugmentationDialog from 'src/components/v2/DataAugmentationDialog.vue';
import { useSocket } from 'src/composables/useSocket.js';
import { useProcessStore } from 'src/stores/processStore';
import MonitoringWindow from 'src/components/v2/MonitoringWindow.vue';
import CheckpointInfo from 'src/components/v2/CheckpointInfo.vue';
import WebRtcVideo from 'src/components/v2/WebRtcVideo.vue';

const processStore = useProcessStore();

const { socket } = useSocket();

const { t } = useI18n()

const workspaces = ref([]);
const selectedWorkspaceId = ref(null);

const showWorkspaceForm = ref(false);
const workspaceForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: 'Workspace Name', type: 'text', value: '', default: '' },
]);

function openCreateWorkspaceForm() {
    selectedWorkspaceId.value = null;
    workspaceForm.value.forEach((field) => {
        field.value = field.default;
    });
    showWorkspaceForm.value = true;
}

function openEditWorkspaceForm(workspace) {
    workspaceForm.value.forEach((field) => {
        field.value = workspace[field.key] || field.default;
    });
    workspaceForm.value.find((e) => e.key === 'id').value = workspace.id; // Set ID for edit
    showWorkspaceForm.value = true;
}

function saveWorkspace (form) {
    if (workspaceForm.value.find((e) => e.key === 'id').value) {
        return api.put(`/task/${form.id}`, form).then(() => {
            listWorkspaces().then(() => {
                selectedWorkspaceId.value = workspaces.value.find(w => w.name === form.name).id;
            });
        });
    }
    return api.post('/task', form).then(() => {
        listWorkspaces().then(() => {
            selectedWorkspaceId.value = workspaces.value.find(w => w.name === form.name).id;
        });
    });
};

function listWorkspaces() {
    return api.get('/tasks').then((response) => {
        workspaces.value = response.data.tasks;
        workspaces.value.push({ id: 'new', name: 'Create New Workspace +' });
    });
}

const selectedWorkspace = computed(() => {
    return workspaces.value.find(w => w.id === selectedWorkspaceId.value) || {};
});

console.log('Selected Workspace:', selectedWorkspace);

const sensors = ref([]);
const robots = computed(() => {
    if (!selectedWorkspace.value) {
        return [];
    }
    if (!selectedWorkspace.value.assembly) {
        return [];
    }
    return selectedWorkspace.value.assembly.robots.map((robot) => {
        const handler = useRobot(robot);
        robot.handler = handler;
        if (robot.type === 'custom') {
            robot.handler.checkRobotTopic();
        }   
        return robot;
    })
});

const selectedWorkspaceSensors = computed(() => {
    if (!selectedWorkspace.value || !selectedWorkspace.value.sensor_settings) {
        return [];
    }
    const selectedIds = Object.keys(selectedWorkspace.value.sensor_settings);
    return sensors.value.filter(s => selectedIds.includes(String(s.id)));
});

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

const assemblies = ref([]);
function listAssemblies() {
    return api.get('/assemblies').then((response) => {
        assemblies.value = response.data.assemblies || [];
    }).catch((error) => {
        console.error('Error fetching assemblies:', error);
    });
}

const selectedTab = ref('setting');

function toggleSensor(sensor) {
    if (sensor.status === 'on') {
        sensor.handler.stopSensor()
    } else {
        sensor.handler.startSensor()
    }
}

const showSensorForm = ref(false);
const sensorForm = ref([
    { key: 'id', value: null },
    { key: 'sensor_ids', label: 'Sensors', type: 'multiselect_list', options: computed(() => sensors.value.map(s => ({ label: s.name, value: s.id }))), value: [] }
])

const sensorSettingsMap = ref({});
watch(() => selectedWorkspace.value?.sensor_settings, (newSettings) => {
    sensorSettingsMap.value = JSON.parse(JSON.stringify(newSettings || {}));
}, { deep: true, immediate: true });

const commonSensorResolution = ref({ width: 640, height: 480 });

watch(selectedWorkspace, (workspace) => {
    if (workspace && workspace.sensor_settings) {
        const firstSensorId = Object.keys(workspace.sensor_settings)[0];
        if (firstSensorId) {
            const settings = workspace.sensor_settings[firstSensorId];
            if (settings && settings.img_size && settings.img_size.length === 2) {
                commonSensorResolution.value = {
                    width: settings.img_size[0],
                    height: settings.img_size[1]
                };
            }
        }
    }
}, { deep: true, immediate: true });

function updateAllSensorResolutions() {
    if (!sensorSettingsMap.value) return;

    const newWidth = commonSensorResolution.value.width;
    const newHeight = commonSensorResolution.value.height;

    const newSettings = { ...sensorSettingsMap.value };

    for (const sensorId in newSettings) {
        if (Object.prototype.hasOwnProperty.call(newSettings, sensorId)) {
            newSettings[sensorId].img_size = [newWidth, newHeight];
        }
    }

    sensorSettingsMap.value = newSettings;

    updateWorkspace({ sensor_settings: sensorSettingsMap.value });
}


function openSensorForm() {
    if (selectedWorkspace.value) {
        sensorForm.value.find(e => e.key === 'sensor_ids').value = Object.keys(selectedWorkspace.value.sensor_settings || {}).map(id => parseInt(id, 10));
    }
    showSensorForm.value = true;
}

function saveSensorSettings(form) {
    const newSettings = {};
    const existingSettings = selectedWorkspace.value.sensor_settings || {};

    for (const id of form.sensor_ids) {
        newSettings[id] = existingSettings[id] || {
            sensor_id: id,
            img_size: [640, 480],
            cropped_area: [0, 0, 640, 480]
        };
    }
    
    const updateForm = {
        sensor_settings: newSettings,
        sensor_ids: []
    }
    
    return updateWorkspace(updateForm);
}

function updateWorkspace(form) {
    console.log('Updating workspace with form:', form);
    return api.put(`/task/${selectedWorkspace.value.id}`, form).then(() => {
        listWorkspaces();
    });
}

function updateWorkspaceDeviceSetting(form) {
    return api.put(`/task/${selectedWorkspace.value.id}/device_settings`, form).then(() => {
        listWorkspaces();
    });
}

const focused = ref({});
watch(selectedWorkspaceId, (newVal) => {
    if (!newVal) {
        return;
    }
    focused.value = {};
    listDatasets();
    listCheckpoints();

});

function toggleRobot(robot) {
    robot.process_id = `robot_${robot.id}`;
    const startFlow = () => robot.handler.startRobot();
    if (robot.status === 'on') {
        robot.handler.stopRobot();
    } else if (robot.status === 'error') {
        robot.handler.stopRobot().finally(() => startFlow());
    } else {
        startFlow();
    }
}

const datasets = ref([]);
function listDatasets() {
    return api.get('/datasets', {
        params: {
            task_id: selectedWorkspaceId.value,
        }
    }).then((res) => {
        datasets.value = res.data.datasets || [];
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
    });
}

function deleteFile(datasetId, file) {
    api.delete(`/dataset/${datasetId}/${file.name}`).then(() => {
        datasets.value = datasets.value.map((d) => {
            if (d.id === datasetId) {
                return {
                    ...d,
                    episodes: d.episodes.filter((e) => e.name !== file.name),
                };
            }
            return d;
        });
    })
}

const selectedEpisode = ref({});
function selectEpisode(episode) {
    selectedEpisode.value = episode;
}
function deselectEpisode() {
    selectedEpisode.value = {};
}   

function deleteDataset(dataset) {
    Notify.create({
        message: `Are you sure you want to delete dataset "${dataset.name}"?`,
        color: 'negative',
        actions: [
            { label: 'Cancel', color: 'white', handler: () => { /* do nothing */ } },
            { label: 'Delete', color: 'white', handler: () => {
                api.delete(`/dataset/${dataset.id}`).then(() => {
                    listDatasets();
                }).catch((error) => {
                    console.error('Error deleting dataset:', error);
                    Notify.create({
                        color: 'negative',
                        message: 'Error deleting dataset'
                    });
                });
            }}
        ]
    });
}

const showDatasetForm = ref(false);
const datasetForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('datasetName'), type: 'text', value: '', default: '' },
])

function openAddDatasetForm() {
    datasetForm.value.forEach((field) => {
        field.value = field.default;
    });
    showDatasetForm.value = true;
}

function openEditDatasetForm(dataset) {
    datasetForm.value.forEach((field) => {
        field.value = dataset[field.key] || field.default;
    });
    datasetForm.value.find((e) => e.key === 'id').value = dataset.id; // Set ID for edit
    showDatasetForm.value = true;
}

function saveDataset(form) {
    if (datasetForm.value.find((e) => e.key === 'id').value) {
        return api.put(`/dataset/${form.id}`, form).then(() => {
            datasetForm.value.forEach(field => field.value = field.default); // Reset form fields
            listDatasets()
        })
    }
    return api.post(`/dataset`, { ...form, task_id: selectedWorkspaceId.value }).then(() => {
        datasetForm.value.forEach(field => field.value = field.default); // Reset form fields
        listDatasets()
    })
}

const showAugmentationForm = ref(false);
const augmentingDataset = ref({});
watch(showAugmentationForm, () => {
    deselectEpisode();
});
function openAugmentationForm(dataset) {
    if (!dataset.episodes.length) {
        Notify.create({
            color: 'negative',
            message: 'Dataset must have at least one episode to augment.'
        });
        return;
    }
    augmentingDataset.value = dataset;
    showAugmentationForm.value = true;
}

function deleteWorkspace(workspace) {
    Notify.create({
        message: `Are you sure you want to delete dataset "${workspace.name}"?`,
        color: 'negative',
        actions: [
            { label: 'Cancel', color: 'white', handler: () => { /* do nothing */ } },
            { label: 'Delete', color: 'white', handler: () => {
                return api.delete(`/task/${workspace.id}`).then(() => {
                    listWorkspaces();
                    if (selectedWorkspaceId.value === workspace.id) {
                        selectedWorkspaceId.value = null;
                    }
                });
            }}
        ]
    });
}


const status = computed(() => {
    if (processStore.isRunning('checkpoint_test')) {
        if (processStore.isRunning('record_episode')) {
            return 'recording_inference';
        }
        return 'inferencing';
    } else if (processStore.isRunning('record_episode')) {
        return 'recording';
    } else if (processStore.isRunning('replay_episode')) {
        return 'replaying';
    } else {
        return 'pending';
    }
});

const selectedDatasetId = ref(null);
const selectedDataset = computed(() => {
    return datasets.value.find(d => d.id === selectedDatasetId.value) || null;
});
const selectedCheckpointId = ref(null);
const selectedCheckpoint = computed(() => {
    return checkpoints.value.find(c => c.id === selectedCheckpointId.value) || null;
});
const checkpoints = ref([]);
function listCheckpoints() {
    const myCheckpointsReq = api.get('/checkpoints', {
        params: {
            where: `task_id,=,${selectedWorkspaceId.value}|status,=,finished`,
            order: 'created_at DESC'
        }
    });

    // 2. 베이스 모델 조회
    const baseModelsReq = api.get('/checkpoints', {
        params: {
            where: `is_base_model,=,1`,
            order: 'created_at DESC'
        }
    });

    // 3. 두 요청을 동시에 실행하고 결과 합치기
    return Promise.all([myCheckpointsReq, baseModelsReq])
        .then(([myRes, baseRes]) => {
            const myData = myRes.data.checkpoints || [];
            const baseData = baseRes.data.checkpoints || [];
            
            // 두 배열을 합침 (중복 제거가 필요하다면 id로 필터링 추가)
            checkpoints.value = [...baseData, ...myData];
            
            // 합친 후 다시 날짜순 정렬 (필요 시)
            // checkpoints.value.sort((a, b) => new Date(b.created_at) - new Date(a.created_at));
        });
}

const showCheckpointForm = ref(false);
const checkpointForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: 'Checkpoint Name', type: 'text', value: '', default: '' },
]);
function openCheckpointForm(checkpoint) {
    showCheckpointForm.value = true;
    checkpointForm.value.forEach((field) => {
        field.value = checkpoint[field.key] || field.default;
    });
}
function deleteCheckpoint(checkpoint) {
    Notify.create({
        message: `Are you sure you want to delete checkpoint "${checkpoint.name}"?`,
        color: 'negative',
        actions: [
            { label: 'Cancel', color: 'white', handler: () => { /* do nothing */ } },
            { label: 'Delete', color: 'white', handler: () => {
                api.delete(`/checkpoint/${checkpoint.id}`).then(() => {
                    listCheckpoints();
                    if (selectedCheckpointId.value === checkpoint.id) {
                        selectedCheckpointId.value = null;
                    }
                });
            }}
        ]
    });
}
function saveCheckpoint(form) {
    return api.put(`/checkpoint/${form.id}`, form).then(() => {
        checkpointForm.value.forEach(field => field.value = field.default); // Reset form fields
        listCheckpoints()
    })
}

// function toggleFailureDetection() {
//     const checkpoint = checkpoints.value.at(-1);
//     const isRunning = processStore.isRunning('failure_detection');
//     const action = isRunning ? 'stop' : 'start';
//     const url = `/checkpoint/${checkpoint.id}/:${action}_failure_detection`;

//     api.post(url, {
//         robots: robots.value.filter(r => selectedWorkspace.value.robot_ids.includes(r.id)),
//         sensors: selectedWorkspaceSensors.value,
//         task: selectedWorkspace.value
//     }).then(() => {
//         Notify.create({
//             color: 'positive',
//             message: `Failure detection ${action}ed.`
//         });
//     }).catch((error) => {
//         console.error(`Error ${action}ing failure detection:`, error);
//         Notify.create({
//             color: 'negative',
//             message: `Error ${action}ing failure detection.`
//         });
//     });
// }

const isFailureDetected = ref(false);
const uncertaintyScore = ref(0);

const showCheckpointInfo = ref(false);
const openCheckpointInfoDialog = (checkpoint) => {
    selectedCheckpointId.value = checkpoint.id;
    showCheckpointInfo.value = true;
}; 


const videoContainer = ref(null);
const isCropping = ref(false);
const cropStartPoint = ref({ x: 0, y: 0 });
const cropEndPoint = ref({ x: 0, y: 0 });

const cropAreaStyle = computed(() => {
    if (!isCropping.value) return {};
    const videoEl = videoContainer.value;
    if (!videoEl) return {};

    const left = Math.min(cropStartPoint.value.x, cropEndPoint.value.x);
    const top = Math.min(cropStartPoint.value.y, cropEndPoint.value.y);
    const width = Math.abs(cropStartPoint.value.x - cropEndPoint.value.x);
    const height = Math.abs(cropStartPoint.value.y - cropEndPoint.value.y);

    return {
        left: `${left}px`,
        top: `${top}px`,
        width: `${width}px`,
        height: `${height}px`,
    };
});


function startCrop(event) {
    isCropping.value = true;
    const rect = videoContainer.value.getBoundingClientRect();
    console.log('Video Container Rect:', rect);
    console.log('Mouse Event:', event);
    cropStartPoint.value = {
        x: event.clientX - rect.left,
        y: event.clientY - rect.top,
    };
    cropEndPoint.value = { ...cropStartPoint.value };
}

function doCrop(event) {
    if (isCropping.value) {
        const rect = videoContainer.value.getBoundingClientRect();
        cropEndPoint.value = {
            x: event.clientX - rect.left,
            y: event.clientY - rect.top,
        };
    }
}

function endCrop() {
    isCropping.value = false;
    saveCroppedArea();
}

function cancelCrop() {
    if (isCropping.value) {
        isCropping.value = false;
        cropStartPoint.value = { x: 0, y: 0 };
        cropEndPoint.value = { x: 0, y: 0 };
    }
}

function saveCroppedArea() {
    if (!focused.value.id) return;
    const videoEl = videoContainer.value;
    if (!videoEl) return;
    
    const videoRect = videoEl.querySelector('video').getBoundingClientRect();
    const containerRect = videoEl.getBoundingClientRect();

    console.log(focused.value);

    const scaleX = focused.value.resolution[0] / videoRect.width;
    const scaleY = focused.value.resolution[1] / videoRect.height;

    const x1_rel = Math.min(cropStartPoint.value.x, cropEndPoint.value.x) - (videoRect.left - containerRect.left);
    const y1_rel = Math.min(cropStartPoint.value.y, cropEndPoint.value.y) - (videoRect.top - containerRect.top);
    const x2_rel = Math.max(cropStartPoint.value.x, cropEndPoint.value.x) - (videoRect.left - containerRect.left);
    const y2_rel = Math.max(cropStartPoint.value.y, cropEndPoint.value.y) - (videoRect.top - containerRect.top);
    
    const x1 = Math.round(x1_rel * scaleX);
    const y1 = Math.round(y1_rel * scaleY);
    const x2 = Math.round(x2_rel * scaleX);
    const y2 = Math.round(y2_rel * scaleY);

    const x1_clamped = Math.max(0, Math.min(focused.value.resolution[0], x1));
    const y1_clamped = Math.max(0, Math.min(focused.value.resolution[1], y1));
    const x2_clamped = Math.max(0, Math.min(focused.value.resolution[0], x2));
    const y2_clamped = Math.max(0, Math.min(focused.value.resolution[1], y2));

    if (sensorSettingsMap.value[focused.value.id]) {
        sensorSettingsMap.value[focused.value.id].cropped_area = [x1_clamped, y1_clamped, x2_clamped, y2_clamped];
        updateWorkspace({ sensor_settings: sensorSettingsMap.value });
    }
}

function resetCroppedArea() {
    if (!focused.value.id) return;
    if (sensorSettingsMap.value[focused.value.id]) {
        sensorSettingsMap.value[focused.value.id].cropped_area = [
            0,
            0,
            focused.value.resolution[0],
            focused.value.resolution[1]
        ];
        updateWorkspace({ sensor_settings: sensorSettingsMap.value });
    }
}


onUnmounted(() => {
    robots.value.forEach(robot => {
        if (robot.handler) {
            robot.handler.unSubscribeRobot();
        }
    });
    if (processStore.isRunning('failure_detection') && selectedCheckpointId.value) {
        api.post(`/checkpoint/${selectedCheckpointId.value}/:stop_failure_detection`)
          .catch(err => console.error('Failed to stop failure detection on unmount:', err));
    }
});

onMounted(() => {
    listWorkspaces();
    listSensors();
    // listRobots();
    listAssemblies();
    socket.on('augmentation_complete', (data) => {
        showAugmentationForm.value = false
        listDatasets().then(() => {
            showAugmentationForm.value = false;
            const new_dataset = datasets.value.find(d => d.id === data.dataset_id);
            if (new_dataset) {
                selectedDatasetId.value = new_dataset.id;
            } else {
                console.error(`Could not find the newly augmented dataset with id: ${data.dataset_id}`);
            }
        });
    })

    socket.on('episode_added', (data) => {
        selectedDataset.value?.episodes.push({
            name: data.name,
        });
    });

    socket.on('failure_detection_result', (data) => {
        console.log(data);
        if (data.failure_detected) {
            isFailureDetected.value = true;
        }
        else {
            isFailureDetected.value = false;
        }
        uncertaintyScore.value = Math.round(data.uncertainty_score * 100) / 100;
    });
});
</script>
<style>
.crop-area {
    position: absolute;
    border: 2px dashed red;
    pointer-events: none;
}
.crop-area-saved {
    position: absolute;
    border: 2px solid green;
    pointer-events: none;
}
</style>