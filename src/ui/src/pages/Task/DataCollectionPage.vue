<template>
    <div class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="dataset in datasets" :key="dataset.id">
                <div class="cursor-pointer text-center">
                    <q-icon name="folder" size="100px" color="amber" />
                    <q-menu context-menu>
                        <q-list bordered separator>
                            <q-item clickable v-ripple v-close-popup @click="showDatasetForm = true; datasetForm = dataset">
                                <q-item-section>Edit Dataset</q-item-section>
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
                    </div>
                </div>
            </div>
            
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" style="min-height: 220px;">
                <q-btn color="grey-8" class="full-height full-width" outline size="lg" icon="add" @click="showDatasetForm = true"></q-btn>
            </div>
        </div>

        <div class="absolute-bottom bg-grey-4">
            <q-separator />
            <div class="q-pa-md row q-gutter-x-md">
                <div class="col">
                  
                </div>
            </div>
        </div>

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
      </div>
</template>

<script setup>
import { onMounted, ref } from 'vue';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';

const route = useRoute();
const taskId = route.params.id; // Assuming the task_id is passed as a route parameter

const datasets = ref([]);
const datasetForm = ref({});
const showDatasetForm = ref(false);

function listDatasets() {
    return api.get('/datasets').then((response) => {
        // Filter datasets by task_id from the route
        datasets.value = response.data.datasets.filter(dataset => dataset.task_id == taskId) || [];
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
    });
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
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

const sensors = ref([]);
function listSensors() {
    return api.get('/sensors').then((response) => {
        sensors.value = response.data.sensors || [];
    }).catch((error) => {
        console.error('Error fetching sensors:', error);
    });
}

onMounted(() => {
    listDatasets();
    getTask();
    listRobots();
    listSensors();
});
</script>

<style scoped>
/* Add any specific styles for this page here */
</style>