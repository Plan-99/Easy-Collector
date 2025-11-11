<template>
    <q-page class="q-pt-lg q-pr-lg full-height column">
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
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
                            :label="`${$t('sensorSetting')} (${sensors.filter(e => selectedWorkspace.sensor_ids.includes(e.id)).length})`"
                        >
                            <q-card class="bg-dark">
                                <q-card-section>
                                    <div
                                        class="q-pa-sm q-px-md q-my-sm border-rounded row"
                                        v-for="sensor in sensors.filter(e => selectedWorkspace.sensor_ids.includes(e.id))" 
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
                                            dense
                                            outlined
                                            dark
                                            bg-color="dark"
                                            label="Width"
                                            class="col"
                                            v-model.number="selectedWorkspace.sensor_img_size[0]"
                                            @change="updateWorkspace({ sensor_img_size: selectedWorkspace.sensor_img_size })"
                                        ></q-input>
                                        <q-input
                                            dense
                                            outlined
                                            dark
                                            bg-color="dark"
                                            label="Height"
                                            class="col"
                                            v-model.number="selectedWorkspace.sensor_img_size[1]"
                                            @change="updateWorkspace({ sensor_img_size: selectedWorkspace.sensor_img_size })"
                                        ></q-input>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>

                        <q-expansion-item
                            icon="adb"
                            :label="`${$t('robotSetting')} (${robots.filter(e => selectedWorkspace.robot_ids.includes(e.id)).length})`"
                        >
                            <q-card class="bg-dark border-rounded">
                            <q-card-section>
                                <div
                                    class="q-pa-sm q-px-md q-my-sm border-rounded row"
                                    v-for="robot in robots.filter(e => selectedWorkspace.robot_ids.includes(e.id))" 
                                    :key="robot.id"
                                    :class="robot.status === 'on' ? 'bg-green-10' : 'bg-grey-8'"
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
                                    </div>
                                    <q-inner-loading :showing="robot.status === 'loading'"></q-inner-loading>
                                </div>
                                <q-btn outline
                                    class="full-width q-mb-sm"
                                    rounded
                                    color="primary"
                                    icon="add"
                                    @click="openRobotForm"
                                ></q-btn>
                            </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
                    <div v-if="focused.id" class="q-pa-md text-white h6">
                        <div>{{ $t(`${focused.device_type}Config`) }} <span class="text-primary">{{ focused.name }}</span></div>
                        <div
                            class="q-pa-sm q-px-md q-mt-sm border-rounded row border-white"
                            v-if="focused.device_type === 'sensor'"
                        >
                            <div class="text-caption">Coming Soon!</div>
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
                                :model-value="selectedDataset && selectedDataset.id === dataset.id"
                                @show="selectedDataset = dataset"
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
                                <q-list bordered separator dark dense v-if="selectedDataset && selectedDataset.id === dataset.id">
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
                                            @click="selectedCheckpointId = checkpoint.id"
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
            <div class="col bg-secondary border-rounded border-white column q-px-sm" v-if="!selectedEpisode.name">
                <div class="col-6 row flex felx-center">
                    <div v-for="sensor in sensors.filter(e => selectedWorkspace.sensor_ids.includes(e.id))" :key="sensor.id" class="col q-py-sm q-px-xs relative-position">
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
                            :resize="[selectedWorkspace.sensor_img_size[0], selectedWorkspace.sensor_img_size[1]]"
                        ></web-rtc-video>
                        <div class="full-height border-white bg-dark border-rounded flex flex-center" v-else>
                            <q-btn round flat icon="play_arrow" text-color="white" size="xl" @click="sensor.handler.startSensor(sensor)"></q-btn>
                        </div>
                        <q-chip color="blue-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px">{{ sensor.name }} sensor</q-chip>
                    </div>
                </div>
                <div class="col-5 row">
                    <div v-for="robot in robots.filter(e => selectedWorkspace.robot_ids.includes(e.id))" :key="robot.id" class="col column q-pa-md relative-position border-rounded border-white text-white cursor-pointer"
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
                <div class="flex flex-center col" v-if="!isRobotSensorAllOn">
                    <div class="text-yellow">Start all sensors and robots to view live data streams.</div>
                </div>
                <div class="col q-py-sm" v-else-if="!selectedCheckpointId">
                    <div class="text-grey bg-dark border-rounded row full-height flex flex-center" v-if="status === 'pending'">
                        <q-select
                            v-model="selectedDataset"
                            dense
                            outlined
                            dark
                            bg-color="dark"
                            label="Select Dataset for Data Collection"
                            style="width: 400px"
                            :options="datasets"
                            option-label="name"
                            option-value="id"
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
                            <div class="text-h6">{{ selectedCheckpoint?.name }}</div>
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
            </div>
            <div v-else-if="selectedEpisode.name && selectedDataset" class="col bg-secondary border-rounded border-white column q-px-sm">
                <div class="col-6 row flex felx-center">
                    <hdf5-viewer
                        :path="`${selectedDataset.id}/${selectedEpisode.name}`"
                        style="width: 100%; height: 100%;"
                        image-class="border-rounded border-white"
                        class="q-gutter-x-sm"
                        @update:robot-states="selectedEpisode.robotStates = $event"
                    ></hdf5-viewer>
                </div>
                <div class="col-5 row q-pa-xs" v-if="selectedEpisode.robotStates">
                    <div v-for="(val, robotId) in selectedEpisode.robotStates" :key="robotId" class="col column q-pa-md relative-position border-rounded border-white text-white cursor-pointer">
                        <div v-for="(j, i) in val.qpos.length" :key="i" class="col flex flex-center q-gutter-x-md">
                            <div class="border-rounded border-white q-px-md q-py-xs text-center">[{{ j }}] {{ val.qpos[i]?.toFixed(4) }}</div>
                            <q-icon name="arrow_forward"></q-icon>
                            <div class="border-rounded border-primary q-px-md q-py-xs text-center text-primary">[{{ j }}] {{ val.qaction[i]?.toFixed(4) }}</div>
                        </div>
                    </div>
                </div>
                <div
                    class="col q-pa-sm bg-dark border-rounded text-white row"
                >
                    <div>
                        <div class="text-h6">Episode: {{ selectedEpisode.name }} </div>
                        <div class="text-caption">Dataset: {{ selectedDataset?.name }} </div>
                    </div>
                    <div>
                        <q-btn
                            icon="close"
                            round
                            flat
                            @click="deselectEpisode"
                        ></q-btn>
                    </div>
                </div>
            </div>
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
            @submit="updateWorkspace"
            :ok-button-label="$t('save')"
        ></form-dialog>
        <form-dialog
            v-model="showRobotForm"
            :title="$t('workspaceRobotFormTitle')"
            :form="robotForm"
            @submit="updateWorkspace"
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

        <data-augmentation-dialog
            v-model="showAugmentationForm"
            :dataset="augmentingDataset"
            :task-id="selectedWorkspaceId"
            v-if="selectedWorkspaceId"
        />
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
        
    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch } from 'vue';
import { api } from 'src/boot/axios';
import FormDialog from 'src/components/v2/FormDialog.vue';
import { useSensor } from '../../composables/useSensor';
import { useRobot } from 'src/composables/useRobot';
import WebRtcVideo from 'src/components/v2/WebRtcVideo.vue';
import Hdf5Viewer from 'src/components/v2/Hdf5Viewer.vue';
import { Notify } from 'quasar';
import { useI18n } from 'vue-i18n';
import DataAugmentationDialog from 'src/components/v2/DataAugmentationDialog.vue';
import { useSocket } from 'src/composables/useSocket.js';
import ProcessConsole from 'src/components/v2/ProcessConsole.vue';
import { useProcessStore } from 'src/stores/processStore';

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
    return workspaces.value.find(w => w.id === selectedWorkspaceId.value) || null;
});

const sensors = ref([]);
const robots = ref([]);

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

function listRobots() {
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.handler = useRobot(robot, () => {
                robot.handler.subscribeRobot(() => {});
            });
        });
    }).catch((error) => {
        console.error('Error fetching robots:', error);
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
    { key: 'sensor_ids', label: 'Sensors', type: 'multiselect', options: computed(() => sensors.value.map(s => ({ label: s.name, value: s.id }))), value: [] }
])

function openSensorForm() {
    if (selectedWorkspace.value) {
        sensorForm.value.find(e => e.key === 'sensor_ids').value = selectedWorkspace.value.sensor_ids;
    }
    showSensorForm.value = true;
}   

function updateWorkspace(form) {
    return api.put(`/task/${selectedWorkspace.value.id}`, form).then(() => {
        listWorkspaces();
    });
}

function updateWorkspaceDeviceSetting(form) {
    return api.put(`/task/${selectedWorkspace.value.id}/device_settings`, form).then(() => {
        listWorkspaces();
    });
}

watch(selectedWorkspaceId, (newVal) => {
    if (!newVal) {
        return;
    }
    focused.value = {};
    listDatasets();
    listCheckpoints();

});

const showRobotForm = ref(false);
const robotForm = ref([
    { key: 'id', value: null },
    { key: 'robot_ids', label: 'Robots', type: 'multiselect', options: computed(() => robots.value.map(r => ({ label: r.name, value: r.id }))), value: [] }
])

function openRobotForm() {
    if (selectedWorkspace.value) {
        robotForm.value.find(e => e.key === 'robot_ids').value = selectedWorkspace.value.robot_ids;
    }
    showRobotForm.value = true;
}

function toggleRobot(robot) {
    if (robot.status === 'on') {
        robot.handler.stopRobot()
    } else {
        robot.handler.startRobot()
    }
}

const focused = ref({});
function focusSensorRobot(device, type) {
    if (device.status !== 'on') {
        return;
    }
    if (focused.value && focused.value.id === device.id && focused.value.device_type === type) {
        focused.value = {};
        return;
    }
    focused.value = device;
    focused.value.device_type = type;
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
const selectedDataset = ref(null)
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

const teleType = ref('leader');

const status = computed(() => {
    if (processStore.isRunning('checkpoint_test')) {
        return 'testing';
    } else if (processStore.isRunning('record_episode')) {
        return 'inferencing';
    } else {
        return 'pending';
    }
});

const collectingProgress = ref(0);
function startDataCollection() {
    if (!selectedDataset.value) {
        Notify.create({
            color: 'negative',
            message: 'Please select a dataset for data collection'
        });
        return;
    }
    showProcessConsole.value = true;
    collectingProgress.value = 0;
    api.post(`/dataset/${selectedDataset.value.id}/:start_collection`, {
        task: selectedWorkspace.value,
        robots: robots.value.filter(e => selectedWorkspace.value.robot_ids.includes(e.id)),
        sensors: sensors.value.filter(e => selectedWorkspace.value.sensor_ids.includes(e.id)),
        tele_type: teleType.value
    }).then(() => {
        Notify.create({
            color: 'positive',
            message: 'Data collection started'
        });
    }).catch((error) => {
        console.error('Error starting data collection:', error);
        Notify.create({
            color: 'negative',
            message: 'Error starting data collection'
        });
    });
}

function stopDataCollection() {
    api.post(`/dataset/${selectedDataset.value.id}/:stop_collection`).then(() => {
        collectingProgress.value = 0;
    })
}

const showProcessConsole = ref(false)

const selectedCheckpointId = ref(null);
const selectedCheckpoint = computed(() => {
    return checkpoints.value.find(c => c.id === selectedCheckpointId.value) || null;
});
const checkpoints = ref([]);
function listCheckpoints() {
    return api.get('/checkpoints', {
        params: {
            where: `task_id,=,${selectedWorkspaceId.value}|status,=,finished`,
            order: 'created_at DESC'
        }
    }).then(response => {
        checkpoints.value = response.data.checkpoints;
    });
}

function startInference() {
    api.post(`/checkpoint/${selectedCheckpointId.value}/:start_test`, {
        task: selectedWorkspace.value,
        policy: selectedCheckpoint.value.policy,
        robots: robots.value.filter(e => selectedWorkspace.value.robot_ids.includes(e.id)),
        sensors: sensors.value.filter(e => selectedWorkspace.value.sensor_ids.includes(e.id)),
    }).then(() => {
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

function stopInference() {
    return api.post(`/checkpoint/${selectedCheckpointId.value}/:stop_test`).catch((error) => {
        console.error('Error stopping test:', error);
        Notify.create({
            color: 'negative',
            message: 'Error stopping test'
        });
    });
}

const isRobotSensorAllOn = computed(() => {
    const selectedRobots = robots.value.filter(r => selectedWorkspace.value?.robot_ids.includes(r.id));
    const selectedSensors = sensors.value.filter(s => selectedWorkspace.value?.sensor_ids.includes(s.id));
    const allRobotsOn = selectedRobots.every(r => r.status === 'on');
    const allSensorsOn = selectedSensors.every(s => s.status === 'on');
    return allRobotsOn && allSensorsOn; 
});

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

onMounted(() => {
    listWorkspaces();
    listSensors();
    listRobots();
    socket.on('augmentation_complete', (data) => {
        Notify.create({
            color: 'positive',
            message: 'Augmentation complete'
        });
        listDatasets().then(() => {
            showAugmentationForm.value = false;
            const new_dataset = datasets.value.find(d => d.id === data.dataset_id);
            if (new_dataset) {
                selectedDataset.value = new_dataset;
            } else {
                console.error(`Could not find the newly augmented dataset with id: ${data.dataset_id}`);
            }
        });
    })

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

    socket.on('episode_added', (data) => {
        selectedDataset.value?.episodes.push({
            name: data.name,
        });
    });
});
</script>