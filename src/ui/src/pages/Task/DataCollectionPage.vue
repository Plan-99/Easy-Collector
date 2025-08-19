<template>
    <div class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="dataset in datasets" :key="dataset.id" style="min-height: 150px;">
                <div class="cursor-pointer text-center"  @click="() => { watchDataset(dataset) }" :style="{ border: watchingDataset && watchingDataset.id === dataset.id ? '1px solid #1976d2' : '' }">
                    <q-icon name="folder" size="100px" color="amber" />
                    <q-menu context-menu>
                        <q-list bordered separator>
                            <q-item clickable v-ripple v-close-popup @click="showDatasetForm = true; datasetForm = dataset">
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
                    <div class="text-h6 text-center">
                        <div>{{ dataset.name }}</div>
                        <div class="text-grey-8 text-body2">{{ dataset.episode_num }}</div> 
                    </div>
                </div>
            </div>
            
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" style="min-height: 150px;">
                <q-btn color="grey-8" class="full-height full-width" outline size="lg" icon="add" @click="showDatasetForm = true"></q-btn>
            </div>
        </div>
        <right-drawer v-model="showRightDrawer" v-if="!datasets.length">
            <div class="row">
                <div class="col text-center q-py-sm cursor-pointer" :class="{ 'bg-grey-4': settingTab !== 'robot', 'text-dark': settingTab !== 'robot' }" @click="settingTab = 'robot'">Robot</div>
                <div class="col text-center q-py-sm cursor-pointer" :class="{ 'bg-grey-4': settingTab !== 'sensor', 'text-dark': settingTab !== 'sensor' }" @click="settingTab = 'sensor'">Sensor</div>
            </div>
            <q-scroll-area v-if="settingTab === 'robot'" class="q-pa-lg" style="height: 800px; width: 600px">
                <q-select
                    v-model="task.robot_ids"
                    label="Robot"
                    emit-value
                    map-options
                    :options="robots"
                    option-label="name"
                    option-value="id"
                    multiple
                    class="q-mb-md"
                    outlined
                    bg-color="grey-2"
                ></q-select>
                <div v-for="robot in robots.filter(e => task.robot_ids.includes(e.id))" :key="robot.id">
                    <div class="row q-col-gutter-sm q-mb-lg">
                        <div class="col">
                            <q-btn color="grey-7 full-width" icon="power_settings_new" v-if="robot.handler.status() === 'off'" @click="robot.handler.startRobot()">
                                {{ robot.name }}
                                <q-tooltip class="bg-primary text-body2">Start robot first to set home pose and end pose</q-tooltip>
                            </q-btn>
                            <q-btn color="orange full-width" icon="power_settings_new" v-if="robot.handler.status() === 'loading'">
                                {{ robot.name }}
                            </q-btn>
                            <q-btn color="green full-width" icon="power_settings_new" v-if="robot.handler.status() === 'on'" @click="robot.handler.stopRobot()">
                                {{ robot.name }}
                            </q-btn>
                        </div>
                    </div>
                    <div v-if="robot.handler.status() === 'on'" class="q-mb-md">
                        <div v-for="type in [{ title: 'Start Pose', id: 'home_pose' }, { title: 'End Pose', id: 'end_pose' }]" :key="type.id"  class="q-mb-md">
                            <div class="row q-col-gutter-sm" v-if="task[type.id][robot.id]">
                                <div class="col-3 row">
                                    <div class="text-h6">{{ type.title }}</div>
                                    <q-space></q-space>
                                    <q-btn flat icon="gamepad" color="primary" @click="openRobotControl(robot, type.id)"></q-btn>
                                </div>
                                <div
                                    v-for="(p, index) in task[type.id][robot.id]"
                                    :key="index"
                                    class="col-3"
                                >
                                    <q-input 
                                        v-model.number="task[type.id][robot.id][index]" 
                                        :label="robot.joint_names[index]" full-width
                                        dense bg-color="grey-2" outlined
                                        type="number"
                                    >
                                    </q-input>
                                </div>
                            </div>
                        </div>
                    </div>
                    
                    <q-separator class="q-my-md" color="grey-6"></q-separator>

                    <div>
                        <q-input
                            v-model="task.episode_len"
                            label="Episode Length"
                            type="number"
                            outlined
                            bg-color="grey-2"
                            class="q-mb-md"
                        ></q-input>
                    </div>
                </div>
            </q-scroll-area>
            <q-scroll-area v-else class="q-pa-lg" style="height: 800px; width: 600px">
                <q-select
                    v-model="task.sensor_ids"
                    label="Sensor"
                    emit-value
                    map-options
                    :options="sensors"
                    option-label="name"
                    option-value="id"
                    multiple
                    class="q-mb-md"
                    outlined
                    bg-color="grey-2"
                ></q-select>
                <div class="row q-col-gutter-sm q-mb-md">
                    <div class="col">
                        <q-input
                            label="Width"
                            v-model.number="task.sensor_img_size[0]"
                            dense
                            outlined
                            bg-color="grey-2"
                            type="number"
                        ></q-input>
                    </div>
                    <div class="col">
                        <q-input
                            label="Width"
                            v-model.number="task.sensor_img_size[1]"
                            dense
                            outlined
                            bg-color="grey-2"
                            type="number"
                        ></q-input>
                    </div>
                </div>
                <div
                    v-for="sensor in sensors.filter(e => task.sensor_ids.includes(e.id))"
                    :key="sensor.id"
                    class="q-mb-md"
                >
                    <div class="row q-col-gutter-sm">
                        <div class="col">
                            <q-btn
                                color="grey-7 full-width"
                                icon="power_settings_new"
                                v-if="sensor.handler.status() === 'off'"
                                @click="sensor.handler.startSensor(sensor)"
                            >
                                {{ sensor.name }}
                                <q-tooltip class="bg-primary text-body2">Start sensor first to set home pose and end pose</q-tooltip>
                            </q-btn>
                            <q-btn
                                color="orange full-width"
                                icon="power_settings_new"
                                v-if="sensor.handler.status() === 'loading'"
                            >
                                {{ sensor.name }}
                            </q-btn>
                            <q-btn
                                color="green full-width"
                                icon="power_settings_new"
                                v-if="sensor.handler.status() === 'on'"
                                @click="sensor.handler.stopSensor(sensor)"
                            >
                                {{ sensor.name }}
                            </q-btn>
                        </div>
                    </div>
                    <div class="q-mt-md flex flex-center">
                        <web-rtc-video
                            v-if="sensor.handler.status() !== 'off'"
                            :process-id="`sensor_${sensor.id}`"
                            style="width: 400px;"
                            :topic="sensor.topic"
                            :resize="[task.sensor_img_size[0], task.sensor_img_size[1]]"
                            :loading="sensor.status !== 'on'"
                        ></web-rtc-video>
                    </div>
                </div>
            </q-scroll-area>
            <div class="absolute-bottom" style="left: 80px; right: 30px; bottom: 20px;">
                <q-btn color="primary full-width" @click="saveTask">Save Setting</q-btn>
            </div>
        </right-drawer>

        <bottom-terminal 
            :tabs="datasets.filter(e => e.onTerminal)" 
            v-model="watchingDataset"
            tab-label="name"
            tab-value="id"
            v-if="datasets.filter(e => e.onTerminal).length > 0 && watchingDataset"
            @update:model-value="watchDataset($event)"
            @close="datasets.forEach(e => e.onTerminal = false)"
        >
            <template v-for="dataset in datasets.filter(e => e.onTerminal)" :key="dataset.id" v-slot:[dataset.id]>
                <div class="row q-pa-xs">
                    <div class="col-4">
                        <q-btn class="full-width bg-white" outline color="primary" @click="showDataCollectionDialog = true">Add Data</q-btn>
                        <q-scroll-area class="bg-grey-2" style="height: 355px; ">
                            <q-list dense separator v-if="watchingFileList.length > 0">
                                <q-item 
                                    clickable v-ripple
                                    v-for="(file, index) in watchingFileList" 
                                    :key="index" 
                                    @click="() => { watchFile(file, watchingFile) }"
                                    :class="{ 'bg-grey-8': watchingFile && watchingFile.name === file.name, 'text-white': watchingFile && watchingFile.name === file.name }"
                                >
                                    <q-item-section>{{ file.name }}</q-item-section>
                                    <q-item-section side>
                                        <q-icon class="text-white" name="close" v-if="watchingFile && watchingFile.name === file.name" @click="deleteFile(file)"></q-icon>
                                    </q-item-section>
                                    <q-separator></q-separator>
                                </q-item>
                                <q-separator></q-separator>
                            </q-list>
                            <div v-else class="text-center text-grey-6 q-pa-md">
                                No files found in this dataset.
                            </div>
                        </q-scroll-area>
                    </div>
                    
                    <div v-if="watchingFile && !showAugmentationForm" class="col q-pl-md">
                        <hdf5-viewer
                            :path="`${watchingDataset.id}/${watchingFile.name}`"
                            style="height: 380px;"
                        ></hdf5-viewer>
                    </div>
                </div>
                
            </template>
        </bottom-terminal>
        <q-dialog v-model="showDatasetForm">
            <q-card style="min-width: 350px">
                <q-card-section class="q-pt-none">
                    <q-card-section>
                        <div class="text-h6 text-center">Dataset</div>
                    </q-card-section>
                    <q-input
                        dense
                        v-model="datasetForm.name"
                        label="Dataset Name"
                        autofocus
                        class="q-mb-md"
                    />
                </q-card-section>

                <q-card-actions align="center" class="text-primary">
                    <q-btn flat label="Save" v-close-popup @click="saveDataset" />
                    <q-btn flat color="grey-7" label="Close" v-close-popup @click="datasetForm = {}" />
                </q-card-actions>
            </q-card>
        </q-dialog>
        <q-dialog
            v-model="showRobotControlDialog"
            persistent
        >
            <q-card style="min-width: 500px">
                <q-card-section class="q-pt-none">
                    <q-card-section>
                        <div class="text-h6 text-center">{{ controlledRobot ? controlledRobot.name : 'Robot Control' }}</div>
                    </q-card-section>
                    <q-card-section class="text-center q-mb-md">
                        <div
                            v-for="(joint, i) in controlledRobot.joint_names" :key="joint"
                        >
                            <div class="text-caption">{{ joint }}</div>
                            <q-slider
                                v-model="controlledRobot.joint_pos[i]"
                                :min="controlledRobot.joint_lower_bounds[i]"
                                :max="controlledRobot.joint_upper_bounds[i]"
                                :step="0.0001"
                                label
                                switch-label-side
                                thumb-size="1px"
                                color="red"
                                @update:model-value="controlledRobot.handler.moveRobot(i, controlledRobot.joint_pos[i])"
                            />
                        </div>
                    </q-card-section>
                </q-card-section>

                <q-card-actions align="center" class="text-primary">
                    <q-btn flat label="Save" v-close-popup @click="savePose" />
                    <q-btn flat color="grey-7" label="Close" v-close-popup @click="closeRobotControl" />
                </q-card-actions>
            </q-card>
        </q-dialog>
        <q-dialog
            maximized
            v-model="showDataCollectionDialog"
            @before-hide="onHideDataCollectionDialog"
        >
            <div class="bg-dark column text-white">
                <div class="absolute-top-right" @click="showDataCollectionDialog = false">
                    <q-icon name="close" class="cursor-pointer q-pa-lg" size="lg" style="z-index: 1000;" v-close-popup></q-icon>
                </div>
                <div class="row col">
                    <div
                        v-for="sensor in sensors.filter(e => task.sensor_ids.includes(e.id))"
                        :key="sensor.id"
                        class="full-width full-height col"
                    >
                        <web-rtc-video
                            :process-id="`sensor_${sensor.id}`"
                            style="width: 400px;"
                            :topic="sensor.topic"
                            class="full-width full-height"
                            v-if="sensor.handler.status() !== 'off'"
                            :resize="[task.sensor_img_size[0], task.sensor_img_size[1]]"
                            :loading="sensor.status !== 'on'"
                        ></web-rtc-video>
                        <div
                            class="full-width full-height text-center q-pa-md bg-grey-8 flex flex-center"
                            style="border-right: 1px solid #ffffff;"
                            v-else
                        >
                            <q-btn round flat icon="play_arrow" size="xl" @click="sensor.handler.startSensor(sensor)"></q-btn>
                        </div>
                    </div>
                </div>
                <div class="row col bg-grey-9">
                    <div class="col-2 q-pa-md">
                        <q-select
                            v-model="watchingDataset"
                            :options="datasets"
                            option-label="name"
                            option-value="id"
                            label="Select Dataset"
                            emit-value
                            map-options
                            class="q-mb-md"
                            outlined
                            bg-color="grey-2"
                        ></q-select>
                        <div v-for="robot in robots.filter(e => task.robot_ids.includes(e.id))" :key="robot.id" class="q-mb-md">
                            <q-btn color="grey-7 full-width" icon="power_settings_new" v-if="robot.handler.status() === 'off'" @click="robot.handler.startRobot()">
                                {{ robot.name }}
                                <q-tooltip class="bg-primary text-body2">Start robot first to set home pose and end pose</q-tooltip>
                            </q-btn>
                            <q-btn color="orange full-width" icon="power_settings_new" v-if="robot.handler.status() === 'loading'">
                                {{ robot.name }}
                            </q-btn>
                            <q-btn color="green full-width" icon="power_settings_new" v-if="robot.handler.status() === 'on'" @click="robot.handler.stopRobot()">
                                {{ robot.name }}
                            </q-btn>
                        </div>
                    </div>
                    <div class="col-10 q-pa-md">
                        <div v-if="
                            robots.filter(e => task.robot_ids.includes(e.id)).find((e) => e.handler.status() === 'off' || e.handler.status() === 'loading') 
                            || sensors.filter(e => task.sensor_ids.includes(e.id)).find((e) => e.handler.status() === 'off' || e.handler.status() === 'loading')"
                            class="flex flex-center full-height"
                            style="border: 1px solid #ffffff;"
                        >
                            Start all robots to collect data
                        </div>
                        <div v-else class="text-center">
                            <div v-if="!dataCollecting">Select Teleoperation Method</div>
                            <div class="q-col-gutter-sm" v-if="!dataCollecting">
                                <q-radio dark v-model="teleoperationMethod" val="leader" label="Leader Robot" />
                                <q-radio dark v-model="teleoperationMethod" val="keyboard" label="Keyboard" disable/>
                            </div>
                            <div v-if="dataCollecting" class="q-mb-md">
                                <q-linear-progress size="25px" instant-feedback :value="collectingProgress" color="accent">
                                    <div class="absolute-full flex flex-center">
                                        <q-badge color="white" text-color="accent" :label="`${Number(collectingProgress * 100).toFixed(0)}%`" />
                                    </div>
                                </q-linear-progress>
                            </div>
                            <div style="border: 1px solid #ffffff;" class="flex flex-center q-pa-lg"
                                v-if="teleoperationMethod === 'leader' && robots.filter(e => task.robot_ids.includes(e.id)).find((e) => !e.leader_robot_preset)"
                            >
                                Go Robot page and set leader robot preset.
                            </div>
                            <div v-else>
                                <q-btn class="full-width bg-white" outline color="primary" @click="startDataCollection" v-if="!dataCollecting">Start Data Collection</q-btn>
                                <q-btn 
                                    class="full-width bg-white" 
                                    outline 
                                    color="orange-8" 
                                    @click="stopDataCollection" 
                                    v-else
                                >Stop</q-btn>
                            
                                <process-console
                                    process="record_episode"
                                    class="full-width q-mt-md"
                                    style="border: 1px solid #ffffff;"                                
                                >
                                </process-console> 
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </q-dialog>
        <data-augmentation-dialog
            v-model="showAugmentationForm"
            :dataset="augmentingDataset"
            :task-id="taskId"
            @hide="watchingFile = null;"
        />
    </div>
</template>

<script setup>
import { onMounted, ref } from 'vue';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';
import BottomTerminal from 'src/components/BottomTerminal.vue';
import RightDrawer from 'src/components/RightDrawer.vue';
import { useRobot } from 'src/composables/useRobot.js';
import { useSensor } from 'src/composables/useSensor.js';
import WebRtcVideo from 'src/components/WebRtcVideo.vue';
import { useSocket } from 'src/composables/useSocket.js';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import Hdf5Viewer from 'src/components/Hdf5Viewer.vue';
import DataAugmentationDialog from 'src/components/DataAugmentationDialog.vue';

const route = useRoute();
const { socket } = useSocket();

const taskId = route.params.id; // Assuming the task_id is passed as a route parameter


const datasets = ref([]);
const datasetForm = ref({});
const showDatasetForm = ref(false);

function listDatasets() {
    return api.get('/datasets', {
        params: {
            task_id: taskId
        }
    }).then((response) => {
        // Filter datasets by task_id from the route
        datasets.value = response.data.datasets || []
        datasets.value.forEach(dataset => {
            dataset.onTerminal = false;
        });
        if (datasets.value.length === 0) {
            showRightDrawer.value = true;
        }
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
    });
}

function getDatasetByID(dataset_id) {
    console.log(datasets.value)
    return datasets.value.find(e => e.id === dataset_id);
}

function saveDataset() {
    if (!datasetForm.value.name) {
        Notify.create({
            color: 'negative',
            message: 'Please fill the dataset name'
        })
        return;
    }
    
    // Add task_id to the datasetForm before saving
    datasetForm.value.task_id = taskId;

    if (datasetForm.value.id) {
        // Update existing dataset
        return api.put(`/dataset/${datasetForm.value.id}`, {
            name: datasetForm.value.name,
            task_id: taskId // Ensure task_id is sent for update if needed by backend
        }).then(() => {
            datasetForm.value = {};
            listDatasets();
        }).catch((error) => {
            console.error('Error updating dataset:', error);
            Notify.create({
                color: 'negative',
                message: 'Error updating dataset'
            });
        });
    } else {
        // Create new dataset
        return api.post(`/dataset`, {
            name: datasetForm.value.name,
            task_id: taskId
        }).then(() => {
            datasetForm.value = {};
            listDatasets();
        }).catch((error) => {
            console.error('Error creating dataset:', error);
            Notify.create({
                color: 'negative',
                message: 'Error creating dataset'
            });
        });
    }
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

const task = ref(null);
function getTask() {
    return api.get(`/tasks/${taskId}`).then((response) => {
        task.value = response.data.task;
    }).catch((error) => {
        console.error('Error fetching task:', error);
    });
}

const robots = ref([]);
function listRobots() {
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.handler = useRobot(robot);
        });
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

const sensors = ref([]);
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

const showRightDrawer = ref(false);
const settingTab = ref('robot');

const showRobotControlDialog = ref(false);
const controlledRobot = ref(null);
const controllingPoseType = ref(null);

function openRobotControl(robot, poseType) {
    controlledRobot.value = robot;
    controlledRobot.value.joint_pos = []
    controllingPoseType.value = poseType;
    showRobotControlDialog.value = true;
    
    controlledRobot.value.handler.subscribeRobot((msg) => {
        controlledRobot.value.joint_pos = msg.position;
        controlledRobot.value.handler.unSubscribeRobot();
    });
    controlledRobot.value.handler.publishRobot();
}

function closeRobotControl() {
    showRobotControlDialog.value = false;
    controlledRobot.value = null;
    controllingPoseType.value = null;
}

function savePose() {
    if (!controlledRobot.value || !controllingPoseType.value) return;
    const pose = controlledRobot.value.joint_pos;
    if (controllingPoseType.value === 'home_pose') {
        task.value.home_pose[controlledRobot.value.id] = pose;
    } else if (controllingPoseType.value === 'end_pose') {
        task.value.end_pose[controlledRobot.value.id] = pose;
    }
    closeRobotControl();
}

function saveTask() {
    return api.put(`/task/${taskId}`, task.value).then(() => {
        Notify.create({
            color: 'positive',
            message: 'Task saved successfully'
        });
    }).catch((error) => {
        console.error('Error saving task:', error);
        Notify.create({
            color: 'negative',
            message: 'Error saving task'
        });
    });
}

const watchingDataset = ref(null);
const watchingFileList = ref([]);
function watchDataset(dataset) {
    // if (watchingDataset.value && watchingDataset.value.id === dataset.id) {
    //     return;
    // }
    watchingFileList.value = [];
    watchingFile.value = null;
    watchingDataset.value = dataset;
    if (!dataset) return;

    dataset.onTerminal = true;
    api.get(`/datasets/${dataset.id}`).then((response) => {
        watchingFileList.value = response.data.files || [];
    }).catch((error) => {
        console.error('Error fetching dataset files:', error);
    });
}

const watchingFile = ref(null);
function watchFile(newFile) {
    watchingFile.value = newFile;
}

const showDataCollectionDialog = ref(false);
const teleoperationMethod = ref('leader');

const dataCollecting = ref(false);
const collectingProgress = ref(0);
function startDataCollection() {
    collectingProgress.value = 0;
    api.post(`/dataset/${watchingDataset.value.id}/:start_collection`, {
        task: task.value,
        robots: robots.value.filter(e => task.value.robot_ids.includes(e.id)),
        sensors: sensors.value.filter(e => task.value.sensor_ids.includes(e.id)),
        tele_type: teleoperationMethod.value
    }).then(() => {
        dataCollecting.value = true;
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
    api.post(`/dataset/${watchingDataset.value.id}/:stop_collection`).then(() => {
        dataCollecting.value = false;
        Notify.create({
            color: 'positive',
            message: 'Data collection stopped'
        });
        collectingProgress.value = 0;
    }).catch((error) => {
        console.error('Error stopping data collection:', error);
        Notify.create({
            color: 'negative',
            message: 'Error stopping data collection'
        });
    });
}

function onHideDataCollectionDialog() {
    stopDataCollection();
    watchDataset(watchingDataset.value);
}

function deleteFile(file) {
    api.delete(`/dataset/${watchingDataset.value.id}/${file.name}`).then(() => {
        watchDataset(watchingDataset.value);
    })
}

const showAugmentationForm = ref(false);
const augmentingDataset = ref(null);
function openAugmentationForm(dataset) {
    if (!dataset.episode_num) {
        Notify.create({
            color: 'negative',
            message: 'Dataset must have at least one episode to augment.'
        });
        return;
    }
    augmentingDataset.value = dataset;
    showAugmentationForm.value = true;
}

onMounted(() => {
    listDatasets();
    getTask();
    listRobots();
    listSensors();

    socket.on('augmentation_complete', (data) => {
        Notify.create({
            color: 'positive',
            message: 'Augmentation complete'
        });
        listDatasets().then(() => {
            showAugmentationForm.value = false;
            const new_dataset = getDatasetByID(data.dataset_id);
            if (new_dataset) {
                watchDataset(new_dataset);
            } else {
                console.error(`Could not find the newly augmented dataset with id: ${data.dataset_id}`);
            }
        });
    })

    socket.on('record_episode_progress', (data) => {
        collectingProgress.value = data.progress;
    });
})

</script>
