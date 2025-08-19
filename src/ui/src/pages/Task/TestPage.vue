<template>
    <div class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="checkpoint in checkpoints" :key="checkpoint.id" style="min-height: 150px;">
                <div class="cursor-pointer text-center"  @click="() => { watchCheckpoint(checkpoint) }" :style="{ border: watchingCheckpoint && watchingCheckpoint.id === checkpoint.id ? '1px solid #1976d2' : '' }">
                    <q-icon name="folder" size="100px" color="grey">
                        <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click="showCheckpointForm = true; checkpointForm = { ...checkpoint }">
                                    <q-item-section>Edit Checkpoint</q-item-section>
                                    <q-item-section side>
                                        <q-icon name="edit" size="xs" />
                                    </q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup class="text-negative" @click="deleteCheckpoint(checkpoint)">
                                    <q-item-section>Delete Checkpoint</q-item-section>
                                    <q-item-section side>
                                        <q-icon color="negative" name="delete" size="xs" />
                                    </q-item-section>
                                </q-item>
                            </q-list>
                        </q-menu>
                    </q-icon>
                    <div class="text-h6 text-center">
                        <div>{{ checkpoint.name }}</div>
                    </div>
                </div>
            </div>
        </div>

        <bottom-terminal
            :tabs="checkpoints.filter(e => e.onTerminal)"
            v-model="watchingCheckpoint"
            tab-label="name"
            tab-value="id"
            v-if="checkpoints.filter(e => e.onTerminal).length > 0 && watchingCheckpoint"
            @update:model-value="watchCheckpoint($event)"
            @close-tab="closeCheckpointTab"
        >
            <template v-for="checkpoint in checkpoints.filter(e => e.onTerminal)" :key="checkpoint.id" v-slot:[checkpoint.id]>
                <div class="q-pa-md">
                    <q-btn color="primary" @click="showTestDialog = true" class="full-width q-mb-sm">Start Test</q-btn>
                    <div class="row q-col-gutter-sm" v-if="watchingCheckpoint" >
                        <div class="col-4">
                            <div class="rounded-borders" style="border: 1px solid rgba(0,0,0,0.12);">
                                <div class="row q-pa-sm bg-grey-2 text-weight-bold">
                                    <div class="col-6">Policy Parameters</div>
                                    <div class="col-6">Value</div>
                                </div>
                                <q-scroll-area style="height: 400px;">
                                    <div v-for="(value, key) in { model: watchingCheckpoint.policy.type, ...watchingCheckpoint.policy.settings }" :key="key">
                                        <q-separator />
                                        <div class="row q-pa-sm">
                                            <div class="col-6" style="word-wrap: break-word;">{{ key }}</div>
                                            <div class="col-6" style="word-wrap: break-word;">{{ value }}</div>
                                        </div>
                                    </div>
                                </q-scroll-area>
                            </div>
                        </div>
                        <div class="col-4">
                            <div class="rounded-borders" style="border: 1px solid rgba(0,0,0,0.12);">
                                <div class="row q-pa-sm bg-grey-2 text-weight-bold">
                                    <div class="col-6">Train Parameters</div>
                                    <div class="col-6">Value</div>
                                </div>
                                <q-scroll-area style="height: 400px;">
                                    <div v-for="(value, key) in { finetuned_from: watchingCheckpoint.load_model?.name, ...watchingCheckpoint.train_settings }" :key="key">
                                        <q-separator />
                                        <div class="row q-pa-sm">
                                            <div class="col-6" style="word-wrap: break-word;">{{ key }}</div>
                                            <div class="col-6" style="word-wrap: break-word;">{{ value }}</div>
                                        </div>
                                    </div>
                                </q-scroll-area>
                            </div>
                        </div>
                        <div class="col-4 q-pa-md text-h6">
                            <div class="text-bold">Datasets: </div>
                            <div class="q-ml-lg">
                                <div v-for="(info, id) in watchingCheckpoint.dataset_info" :key="id">
                                    <div class="text-grey-8 text-body1">
                                        <span class="text-black">{{ info.name }}</span> (Number of Episodes: {{ info.episode_num }})
                                    </div>
                                </div>
                            </div>
                            <div class="text-bold">Loss: <span class="text-body1">{{ watchingCheckpoint.loss?.toFixed(5) }}</span></div>
                            <div class="text-bold">Best Epoch: <span class="text-body1">{{ watchingCheckpoint.best_epoch }}</span></div>
                        </div>
                    </div>
                    <div v-else class="text-center text-grey-6 q-pa-md">
                        Loading policy settings...
                    </div>
                </div>
            </template>
        </bottom-terminal>

        <q-dialog v-model="showCheckpointForm">
            <q-card style="min-width: 350px;">
                <q-card-section>
                    <div class="text-h6">Edit Checkpoint</div>
                </q-card-section>

                <q-card-section class="q-pt-none">
                    <q-input dense v-model="checkpointForm.name" label="Checkpoint Name" autofocus @keyup.enter="editCheckpoint" />
                </q-card-section>

                <q-card-actions align="right" class="text-primary">
                    <q-btn flat label="Cancel" v-close-popup />
                    <q-btn flat label="Save" @click="editCheckpoint" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <q-dialog
            maximized
            v-model="showTestDialog"
            @before-hide="onHideTestDialog"
        >
            <div class="bg-dark column text-white">
                <div class="absolute-top-right" @click="showTestDialog = false">
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
                        <div>
                            <q-input dense outlined label="Timesteps per episode" class="bg-white" v-model="timesteps"></q-input>
                        </div>
                    </div>
                    <div class="col-10 q-pa-md">
                        <div v-if="
                            robots.filter(e => task.robot_ids.includes(e.id)).find((e) => e.handler.status() === 'off' || e.handler.status() === 'loading') 
                            || sensors.filter(e => task.sensor_ids.includes(e.id)).find((e) => e.handler.status() === 'off' || e.handler.status() === 'loading')"
                            class="flex flex-center full-height"
                            style="border: 1px solid #ffffff;"
                        >
                            Start all robots and sensors to start test
                        </div>
                        <div v-else class="text-center">
                            <div v-if="!testing">
                                <q-btn class="full-width bg-white" outline color="primary" @click="startTest">Start Test</q-btn>
                            </div>
                            <div v-else>
                                <div class="q-mb-md">
                                    <q-linear-progress size="25px" instant-feedback :value="testingProgress" color="accent">
                                        <div class="absolute-full flex flex-center">
                                            <q-badge color="white" text-color="accent" :label="`${Number(testingProgress * 100).toFixed(0)}%`" />
                                        </div>
                                    </q-linear-progress>
                                </div>
                                <q-btn 
                                    class="full-width bg-white" 
                                    outline 
                                    color="orange-8" 
                                    @click="stopTest" 
                                >Stop</q-btn>
                            </div>
                            <process-console
                                process="checkpoint_test"
                                class="full-width q-mt-md"
                                style="border: 1px solid #ffffff;"                                
                            >
                            </process-console> 
                        </div>
                    </div>
                </div>
            </div>
        </q-dialog>
    </div>
</template>

<script setup>
import { onMounted, ref } from 'vue';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';
import BottomTerminal from 'src/components/BottomTerminal.vue';
import { useRobot } from 'src/composables/useRobot.js';
import { useSensor } from 'src/composables/useSensor.js';
import WebRtcVideo from 'src/components/WebRtcVideo.vue';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Notify } from 'quasar';
import { useSocket } from 'src/composables/useSocket.js';

const route = useRoute();
const { socket } = useSocket();

const taskId = Number(route.params.id);

const checkpoints = ref([]);
const policies = ref([]);
const watchingCheckpoint = ref(null);
const activePolicy = ref(null);

const showCheckpointForm = ref(false);
const checkpointForm = ref({
    id: null,
    name: '',
});

const showTestDialog = ref(false);
const testing = ref(false);

const task = ref(null);
const robots = ref([]);
const sensors = ref([]);

const testingProgress = ref(0);

function listCheckpoints() {
    return api.get('/checkpoints', {
        params: {
            where: `task_id,=,${taskId}|is_training,=,0`
        }
    }).then((response) => {
        checkpoints.value = response.data.checkpoints
    })
}

function editCheckpoint() {
    api.put(`/checkpoint/${checkpointForm.value.id}`, {
        name: checkpointForm.value.name
    }).then(() => {
        Notify.create({
            color: 'positive',
            message: 'Checkpoint updated successfully'
        });
        listCheckpoints();
    }).catch((error) => {
        console.error('Error updating checkpoint:', error);
        Notify.create({
            color: 'negative',
            message: 'Failed to update checkpoint'
        });
    });
}

function deleteCheckpoint(checkpoint) {
    api.delete(`/checkpoint/${checkpoint.id}`).then(() => {
        Notify.create({
            color: 'positive',
            message: 'Checkpoint deleted successfully'
        });
        listCheckpoints();
    }).catch((error) => {
        console.error('Error deleting checkpoint:', error);
        Notify.create({
            color: 'negative',
            message: 'Failed to delete checkpoint'
        });
    });
}

function listPolicies() {
    return api.get('/policies').then((response) => {
        policies.value = response.data.policies;
    });
}

function getTask() {
    return api.get(`/tasks/${taskId}`).then((response) => {
        task.value = response.data.task;
        timesteps.value = task.value.episode_len;
    }).catch((error) => {
        console.error('Error fetching task:', error);
    });
}

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

function watchCheckpoint(checkpoint) {
    if (!checkpoint) {
        watchingCheckpoint.value = null;
        return;
    }

    watchingCheckpoint.value = checkpoint;
    checkpoint.onTerminal = true;

    if (checkpoint.policy_id) {
        activePolicy.value = policies.value.find(p => p.id === checkpoint.policy_id) || null;
    } else {
        activePolicy.value = null;
    }
}

function closeCheckpointTab(checkpoint) {
    const c = checkpoints.value.find(c => c.id === checkpoint.id);
    if (c) {
        c.onTerminal = false;
    }
    if (watchingCheckpoint.value && watchingCheckpoint.value.id === checkpoint.id) {
        watchingCheckpoint.value = null;
    }
}

const timesteps = ref(100)
function startTest() {
    api.post(`/checkpoint/${watchingCheckpoint.value.id}/:start_test`, {
        timesteps: timesteps.value,
        task: task.value,
        policy: activePolicy.value,
        robots: robots.value.filter(e => task.value.robot_ids.includes(e.id)),
        sensors: sensors.value.filter(e => task.value.sensor_ids.includes(e.id)),
    }).then(() => {
        testing.value = true;
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

function stopTest() {
    api.post(`/checkpoint/${watchingCheckpoint.value.id}/:stop_test`).then(() => {
        testing.value = false;
        Notify.create({
            color: 'positive',
            message: 'Test stopped'
        });
        testingProgress.value = 0;
    }).catch((error) => {
        console.error('Error stopping test:', error);
        Notify.create({
            color: 'negative',
            message: 'Error stopping test'
        });
    });
}

function onHideTestDialog() {
    if (testing.value) {
        stopTest();
    }
}

onMounted(() => {
    listCheckpoints();
    listPolicies();
    getTask();
    listRobots();
    listSensors();

    socket.on('checkpoint_test_progress', (data) => {
        testingProgress.value = data.progress;
        console.log('Checkpoint test progress:', testingProgress.value);
    });

    socket.on('stop_process', (data) => {
        if (data.id === 'checkpoint_test') {
            testing.value = false;
            testingProgress.value = 0;
            Notify.create({
                color: 'positive',
                message: 'Test stopped'
            });
        }
    });

});
</script>