<template>
    <q-page class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="task in tasks" :key="task.id">
                <q-card class="cursor-pointer full-height"  @click="$router.push(`/tasks/${task.id}/data_collection`)">
                    <div class="cursor-pointer relative-position" style="height: 180px; display: flex; align-items: center; justify-content: center;">
                        <div style="font-size: 80px;">
                            {{ task.image || '🤖' }}
                        </div>
                        <div class="absolute-bottom text-h6 q-pa-sm" style="background-color: rgba(0,0,0,0.5); color: white; width: 100%;">
                            <div>{{ task.name }}</div>
                            <q-space></q-space>
                            <q-icon v-if="task.status === 'running'" color="positive" name="play_arrow" size="sm"></q-icon>
                            <q-icon v-if="task.status === 'stopped'" name="stop" size="sm"></q-icon>
                        </div>
                        <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click.stop="openEditTaskDialog(task)">
                                    <q-item-section>Edit Task</q-item-section>
                                    <q-item-section side>
                                        <q-icon name="edit" size="xs" />
                                    </q-item-section>
                                </q-item>
                                <q-item clickable v-ripple class="text-negative" @click.stop="deleteTask(task)">
                                    <q-item-section>Delete Task</q-item-section>
                                    <q-item-section side>
                                        <q-icon color="negative" name="delete" size="xs" />
                                    </q-item-section>
                                </q-item>
                            </q-list>
                        </q-menu>
                    </div>
                </q-card>
            </div>
            <div class="col-6 col-sm-4 col-md-3 col-lg-2">
                <q-btn color="grey-8" class="full-height full-width" outline size="lg" icon="add" @click="openNewTaskDialog"></q-btn>
            </div>
        </div>
        <q-dialog v-model="showTaskForm" persistent>
            <q-card style="min-width: 500px">
                <q-card-section class="q-pt-none">
                    <q-card-section>
                        <div class="text-h6 text-center">Task</div>
                    </q-card-section>
                    <div class="text-center q-mb-md">
                        <q-btn flat :label="taskForm.image" style="font-size: 30px;">
                            <q-popup-proxy>
                                <EmojiPicker :native="true" @select="onSelectEmoji" disable-skin-tones />
                            </q-popup-proxy>
                        </q-btn>
                    </div>
                    <q-input dense v-model="taskForm.name" label="Task Name" autofocus class="q-mb-md" />
                    <q-select dense v-model="taskForm.robot_ids" :options="robots" label="Robot" class="q-mb-md" multiple map-options emit-value option-label="name" option-value="id" />
                    <q-select dense v-model="taskForm.sensor_ids" :options="sensors" label="Sensors" class="q-mb-md" multiple map-options emit-value option-label="name" option-value="id" />
                    <!-- <div v-if="taskForm.robot_id">
                        <div class="q-mb-md">
                            <div class="text-caption">Home Pose</div>
                            <div class="row q-gutter-sm">
                                <q-input dense v-for="(joint, i) in robots.find(e => e.id === taskForm.robot_id).joint_names" :key="i" v-model.number="taskForm.home_pose[i]" :label="joint" class="col" type="number" />
                            </div>
                        </div>
                        <div class="q-mb-md">
                            <div class="text-caption">End Pose</div>
                            <div class="row q-gutter-sm">
                                <q-input dense v-for="(joint, i) in robots.find(e => e.id === taskForm.robot_id).joint_names" :key="i" v-model.number="taskForm.end_pose[i]" :label="joint" class="col" type="number" />
                            </div>
                        </div>
                    </div> -->
                    <!-- <q-input dense v-model="taskForm.episode_len" label="Episode Length" type="number" class="q-mb-md" /> -->
                </q-card-section>

                <q-card-actions align="center" class="text-primary">
                    <q-btn flat label="Save" v-close-popup @click="saveTask" />
                    <q-btn flat color="grey-7" label="Close" v-close-popup @click="taskForm = { ...defaultTaskForm }" />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { onMounted, ref } from 'vue';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';
import EmojiPicker from 'vue3-emoji-picker';
import 'vue3-emoji-picker/css';

const tasks = ref([]);
const robots = ref([]);
const sensors = ref([]);

const showTaskForm = ref(false);

function onSelectEmoji(emoji) {
  taskForm.value.image = emoji.i;
}

const defaultTaskForm = {
    name: '',
    robot_ids: [],
    sensor_ids: [],
    home_pose: {},
    end_pose: {},
    episode_len: 100,
    image: '🤖',
};
const taskForm = ref({ ...defaultTaskForm });

function listTasks() {
    return api.get('/tasks').then((response) => {
        tasks.value = response.data.tasks || [];
    }).catch((error) => {
        console.error('Error fetching tasks:', error);
    });
}

function listRobots() {
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

function listSensors() {
    return api.get('/sensors').then((response) => {
        sensors.value = response.data.sensors || [];
    }).catch((error) => {
        console.error('Error fetching sensors:', error);
    });
}

function saveTask() {
    taskForm.value.robot_ids.forEach((robotId) => {
        const robot = robots.value.find(r => r.id === robotId);
        const jointNames = robot ? robot.joint_names : [];
        taskForm.value.home_pose[robot.id] = Array(jointNames.length).fill(0);
        taskForm.value.end_pose[robot.id] = Array(jointNames.length).fill(0);
    });
    if (!taskForm.value.name || !taskForm.value.robot_ids.length || !taskForm.value.sensor_ids.length) {
        Notify.create({
            color: 'negative',
            message: 'Please fill the form'
        });
        return;
    }

    if (taskForm.value.id) {
        return api.put(`/task/${taskForm.value.id}`, taskForm.value).then(() => {
            listTasks();
            taskForm.value = { ...defaultTaskForm };
        });
    } else {
        return api.post('/task', taskForm.value).then(() => {
            listTasks();
            taskForm.value = { ...defaultTaskForm };
        });
    }
}

function deleteTask(task) {
    return api.delete(`/task/${task.id}`).then(() => {
        listTasks();
    });
}

function openNewTaskDialog() {
    taskForm.value = { ...defaultTaskForm };
    showTaskForm.value = true;
}

function openEditTaskDialog(task) {
    const form = { ...task };
    taskForm.value = form;
    showTaskForm.value = true;
}

onMounted(() => {
    listTasks();
    listRobots();
    listSensors();
});
</script>''