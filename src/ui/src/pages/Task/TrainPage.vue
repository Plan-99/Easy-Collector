<template>
    <q-page class="q-pa-md">
        <q-stepper
            v-model="step"
            ref="stepper"
            color="primary"
            animated
        >
            <q-step
                :name="1"
                title="Select Datasets"
                icon="collections_bookmark"
                :done="step > 1"
            >
                <div class="text-h6 q-mb-md">Select datasets for training</div>
                <div class="row q-col-gutter-md">
                    <div class="col-6">
                        <q-card>
                            <q-card-section>
                                <div class="text-subtitle1">Available Datasets</div>
                            </q-card-section>
                            <q-separator />
                            <q-list bordered>
                                <q-item v-for="dataset in availableDatasets" :key="dataset.id">
                                    <q-item-section>{{ dataset.name }}</q-item-section>
                                    <q-item-section side>
                                        <q-btn icon="add" size="sm" flat round @click="selectDataset(dataset)" />
                                    </q-item-section>
                                </q-item>
                                <q-item v-if="!availableDatasets.length">
                                    <q-item-section class="text-grey">No available datasets</q-item-section>
                                </q-item>
                            </q-list>
                        </q-card>
                    </div>
                    <div class="col-6">
                        <q-card>
                            <q-card-section>
                                <div class="text-subtitle1">Selected Datasets</div>
                            </q-card-section>
                            <q-separator />
                            <q-list bordered>
                                <q-item v-for="dataset in selectedDatasets" :key="dataset.id">
                                    <q-item-section>{{ dataset.name }}</q-item-section>
                                    <q-item-section side>
                                        <q-btn icon="remove" size="sm" flat round @click="deselectDataset(dataset)" />
                                    </q-item-section>
                                </q-item>
                                <q-item v-if="!selectedDatasets.length">
                                    <q-item-section class="text-grey">No datasets selected</q-item-section>
                                </q-item>
                            </q-list>
                        </q-card>
                    </div>
                </div>
            </q-step>

            <q-step
                :name="2"
                title="Select Policy"
                icon="policy"
                :done="step > 2"
            >
                <div class="text-h6">Select a policy</div>
                <!-- Placeholder for policy selection -->
                <p>Here you will be able to select the policy for training.</p>
            </q-step>

            <q-step
                :name="3"
                title="Train Model"
                icon="model_training"
            >
                <div class="text-h6">Start Training</div>
                <!-- Placeholder for training controls and status -->
                <p>Click the button to start the training process. Progress will be displayed below.</p>
                <q-btn color="primary" label="Start Training" @click="startTraining" />
            </q-step>

            <template v-slot:navigation>
                <q-stepper-navigation class="text-center">
                    <q-btn v-if="step > 1" flat color="primary" @click="$refs.stepper.previous()" label="Back" class="q-mr-sm" />
                    <q-btn @click="nextStep" color="primary" :label="step === 3 ? 'Finish' : 'Continue'" />
                </q-stepper-navigation>
            </template>
        </q-stepper>
    </q-page>
</template>

<script setup>
import { ref, onMounted } from 'vue';
import { useQuasar } from 'quasar';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';

const $q = useQuasar();
const route = useRoute();
const step = ref(1);
const stepper = ref(null);

const taskId = route.params.id;
const availableDatasets = ref([]);
const selectedDatasets = ref([]);

function listDatasets() {
    api.get('/datasets').then((response) => {
        availableDatasets.value = response.data.datasets.filter(dataset => dataset.task_id == taskId) || [];
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
        $q.notify({
            color: 'negative',
            message: 'Failed to load datasets.'
        });
    });
}

function selectDataset(dataset) {
    selectedDatasets.value.push(dataset);
    availableDatasets.value = availableDatasets.value.filter(d => d.id !== dataset.id);
}

function deselectDataset(dataset) {
    availableDatasets.value.push(dataset);
    selectedDatasets.value = selectedDatasets.value.filter(d => d.id !== dataset.id);
    // sort by id
    availableDatasets.value.sort((a, b) => a.id - b.id);
}

onMounted(() => {
    listDatasets();
});

function nextStep() {
    if (step.value === 1 && selectedDatasets.value.length === 0) {
        $q.notify({
            color: 'warning',
            message: 'Please select at least one dataset to continue.',
            icon: 'warning'
        });
        return;
    }
    if (step.value < 3) {
        stepper.value.next();
    } else {
        // Handle finish action
        $q.notify({
            color: 'positive',
            message: 'Training setup complete!',
            icon: 'check'
        });
    }
}

function startTraining() {
    $q.notify({
        color: 'info',
        message: 'Starting training process...',
        icon: 'play_arrow'
    });
    // Placeholder for actual training logic
}
</script>

<style scoped>
/* Add any specific styles for this page here */
</style>
