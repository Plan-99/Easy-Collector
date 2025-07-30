<template>
    <div class="q-pa-md">
        <q-stepper v-model="step" ref="stepper" color="primary" animated>
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
                                    <q-item-section avatar>
                                        <q-icon name="folder" size="24px" color="amber" />
                                    </q-item-section>
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
                                    <q-item-section avatar>
                                        <q-icon name="folder" size="24px" color="amber" />
                                    </q-item-section>
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
                <div class="text-h6 q-mb-md">Configure Policy</div>
                <div class="row q-col-gutter-md">
                    <div class="col-12">
                        <q-select
                            outlined
                            v-model="selectedCheckpoint"
                            :options="checkpointOptions"
                            label="Load Model from Checkpoint"
                            emit-value
                            map-options
                            clearable
                            @clear="handleCheckpointClear"
                        >
                            <template v-slot:option="scope">
                                <q-item v-bind="scope.itemProps">
                                    <q-item-section>
                                        <q-item-label>{{ scope.opt.label }}</q-item-label>
                                    </q-item-section>
                                    <q-item-section side v-if="scope.opt.value">
                                        <q-btn icon="delete" size="sm" flat round @click.stop="deleteCheckpoint(scope.opt.value)" />
                                    </q-item-section>
                                </q-item>
                            </template>
                        </q-select>
                    </div>
                    <div class="col-12">
                        <q-select
                            outlined
                            v-model="selectedPolicy"
                            :options="policyOptions"
                            label="Select Policy"
                            emit-value
                            map-options
                            :disable="!!selectedCheckpoint"
                        >
                            <template v-slot:option="scope">
                                <q-item v-if="scope.opt.value === 'new'" v-bind="scope.itemProps" @click="handleCreateNewPolicy">
                                    <q-item-section>
                                        <q-item-label class="text-grey">Create New Policy +</q-item-label>
                                    </q-item-section>
                                </q-item>
                                <q-item v-else v-bind="scope.itemProps">
                                     <q-item-section>
                                        <q-item-label>{{ scope.opt.label }}</q-item-label>
                                    </q-item-section>
                                    <q-item-section side>
                                        <q-btn icon="delete" size="sm" flat round @click.stop="deletePolicy(scope.opt.value)" />
                                    </q-item-section>
                                </q-item>
                            </template>
                        </q-select>
                    </div>
                </div>

                <q-card class="q-mt-md" v-if="policyForm.name !== undefined">
                    <q-card-section>
                        <div class="text-h6">{{ formTitle }}</div>
                    </q-card-section>
                    <q-card-section>
                        <q-form class="q-col-gutter-md row">
                            <div class="col-4">
                                <q-input dense outlined v-model="policyForm.name" label="Policy Name" :readonly="isNameReadonly" />
                            </div>
                            <div class="col-4">
                                <q-select dense outlined v-model="policyForm.type" :options="policyTypes" label="Policy Type" :readonly="isTypeReadonly" @update:model-value="handlePolicyTypeChange" />
                            </div>
                            <div v-for="(config, key) in policyForm.settings" :key="key" class="col-4">
                                <q-select
                                    v-if="config.type === 'select'"
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :options="config.options"
                                    :label="config.label"
                                    :readonly="isSettingsReadonly"
                                    :disable="isSettingsReadonly"
                                    emit-value
                                    map-options
                                />
                                <q-input
                                    dense
                                    outlined
                                    v-model.number="config.value"
                                    :label="config.label"
                                    :readonly="isSettingsReadonly"
                                    :disable="isSettingsReadonly"
                                    v-else-if="config.type === 'number'"
                                    type="number"
                                />
                                <q-input
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :label="config.label"
                                    :readonly="isSettingsReadonly"
                                    :disable="isSettingsReadonly"
                                    v-else
                                    type="text"
                                />
                            </div>
                        </q-form>
                    </q-card-section>
                    <q-card-actions align="right" v-if="showFormButtons">
                        <q-btn flat label="Save" color="primary" @click="savePolicy" v-if="selectedPolicy === 'new'" />
                        <q-btn flat label="Save As..." color="secondary" @click="showSaveAsDialog = true" v-if="selectedPolicy && selectedPolicy !== 'new'" />
                    </q-card-actions>
                </q-card>
            </q-step>

            <q-step
                :name="3"
                title="Train Model"
                icon="model_training"
            >
                <div class="q-gutter-y-md">
                    <div class="row q-col-gutter-md">
                        <q-input dense outlined v-model="newCheckpointName" class="col-12" label="Checkpoint Name" :disable="isTraining" />

                        <q-input dense outlined v-model.number="trainingForm.numEpochs" class="col" label="Num Epochs" type="number" :disable="isTraining" />
                        <q-input dense outlined v-model.number="trainingForm.batchSize" class="col" label="Batch Size" :disable="isTraining" />
                        <q-input dense outlined v-model="trainingForm.learning_rate" class="col" label="Learning Rate" type="number" :disable="isTraining" />
                        <q-input dense outlined v-model="trainingForm.lr_backbone" class="col" label="Backbone Learning Rate" type="number" :disab />
                    </div>
                    <div class="text-center">
                        <q-btn v-if="!isTraining" color="primary" label="Start Training" @click="startTraining" :loading="isLoadingApi" :disable="isTraining" />
                        <q-btn v-else color="negative" label="Stop Training" @click="stopTraining" />

                    </div>

                    <q-linear-progress
                        v-if="isTraining"
                        instant-feedback
                        :value="trainingProgress"
                        size="20px"
                        color="primary"
                    >
                        <div class="absolute-full flex flex-center">
                            <q-badge color="white" text-color="primary" :label="`${Number(trainingProgress * 100).toFixed(2)}%`" />
                        </div>
                    </q-linear-progress>


                    <process-console
                        v-if="step === 3"
                        process="train_task"
                    />
                </div>
                
            </q-step>

            <template v-slot:navigation>
                <q-stepper-navigation class="text-center">
                    <q-btn v-if="step > 1 && !isTraining" flat color="primary" @click="$refs.stepper.previous()" label="Back" class="q-mr-sm" />
                    <q-btn @click="nextStep" color="primary" :label="'Continue'" v-show="step !== 3" />
                </q-stepper-navigation>
            </template>
        </q-stepper>

        <q-dialog v-model="showSaveAsDialog">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">Save Policy As</div>
                </q-card-section>
                <q-card-section class="q-pt-none">
                    <q-input dense v-model="newPolicyName" label="New Policy Name" autofocus />
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat label="Cancel" v-close-popup />
                    <q-btn flat label="Save" color="primary" @click="savePolicyAs" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </div>
</template>

<script setup>
import { ref, onMounted, computed, watch } from 'vue';
import { useQuasar, Notify } from 'quasar';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { useSocket } from 'src/composables/useSocket';
import { useProcessStore } from 'src/stores/processStore';

const processStore = useProcessStore();

const $q = useQuasar();
const route = useRoute();
const step = ref(1);
const stepper = ref(null);
const taskId = Number(route.params.id);

// Step 1: Dataset Selection
const availableDatasets = ref([]);
const selectedDatasets = ref([]);

// Step 2: Policy Selection
const checkpoints = ref([]);
const policies = ref([]);
const selectedCheckpoint = ref(null);
const selectedPolicy = ref(null);
const policyForm = ref({});
const trainingForm = ref({
    numEpochs: 1000,
    batchSize: 20,
    learning_rate: 1e-5,
    lr_backbone: 1e-6
});
// const numEpochsForm = ref(1000);
// const batchSizeForm = ref(32);
const showSaveAsDialog = ref(false);
const newPolicyName = ref('');
const newCheckpointName = ref('');

const taskVal = ref({});

// Step 3: Training
const isLoadingApi = ref(false); // For API call loading state
const { socket } = useSocket();

const POLICY_CONFIGS = {
    'ACT': {
        'chunk_size': { 'label': 'Chunk Size', 'value': 32, 'type': 'number' },
        'kl_weight': { 'label': 'KL Weight', 'value': 10.0, 'type': 'number' },
        'hidden_dim': { 'label': 'Hidden Dimension', 'value': 256, 'type': 'number' },
        'dim_feedforward': { 'label': 'Feedforward Dimension', 'value': 2048, 'type': 'number' },
        'backbone': { 'label': 'Backbone', 'value': 'resnet18', 'type': 'select', 'options': ['resnet18'] },
        'enc_layers': { 'label': 'Encoder Layers', 'value': 4, 'type': 'number' },
        'dec_layers': { 'label': 'Decoder Layers', 'value': 6, 'type': 'number' },
        'nheads': { 'label': 'Number of Heads', 'value': 8, 'type': 'number' },
        'position_embedding': { 'label': 'Position Embedding', 'value': 'sine', 'type': 'select', 'options': ['sine'] },
    }
};
const policyTypes = Object.keys(POLICY_CONFIGS);

// --- Computed Properties for Step 2 ---
const checkpointOptions = computed(() => {
    return [
        { label: 'Start without checkpoint', value: null },
        ...checkpoints.value.filter(c => c.task_id === taskId).map(c => ({ label: c.name, value: c.id }))
    ];
});

const policyOptions = computed(() => {
    const options = policies.value.map(p => ({ label: p.name, value: p.id }));
    options.push({ label: 'Create New Policy +', value: 'new' });
    return options;
});

const formTitle = computed(() => {
    if (selectedCheckpoint.value) return 'Finetune Policy';
    if (selectedPolicy.value === 'new') return 'Create New Policy';
    if (selectedPolicy.value) return 'Edit Policy';
    return '';
});

const isNameReadonly = computed(() => !!selectedCheckpoint.value || (selectedPolicy.value && selectedPolicy.value !== 'new'));
const isTypeReadonly = computed(() => !!selectedCheckpoint.value || (selectedPolicy.value && selectedPolicy.value !== 'new'));
const isSettingsReadonly = computed(() => !!selectedCheckpoint.value);
const showFormButtons = computed(() => selectedPolicy.value && !selectedCheckpoint.value);
const isTraining = computed(() => processStore.isRunning('train_task'));

function validatePolicyForm() {
    const form = policyForm.value;

    if (!form.name || form.name.trim() === '') {
        Notify.create({ color: 'negative', message: 'Policy Name is required.' });
        return false;
    }
    if (!form.type) {
        Notify.create({ color: 'negative', message: 'Policy Type is required.' });
        return false;
    }

    if (form.settings) {
        for (const key in form.settings) {
            const setting = form.settings[key];
            const value = setting.value;
            if (value === null || value === undefined || (typeof value === 'string' && value.trim() === '')) {
                Notify.create({ color: 'negative', message: `Setting '${setting.label}' is required.` });
                return false;
            }
        }
    }
    return true;
}

// --- Watchers for Step 2 ---
watch(selectedCheckpoint, (newVal) => {
    if (newVal) {
        const checkpoint = checkpoints.value.find(c => c.id === newVal);
        if (checkpoint && checkpoint.policy) {
            const policy = checkpoint.policy;
            const newSettings = {};
            const config = POLICY_CONFIGS[policy.type];
            if (config) {
                for (const key in config) {
                    newSettings[key] = { ...config[key] }; // Copy complete config
                    if (policy.settings[key] !== undefined) {
                        newSettings[key].value = policy.settings[key]; // Override value
                    }
                }
            }
            selectedPolicy.value = policy.id;
            policyForm.value = {
                ...JSON.parse(JSON.stringify(policy)),
                name: policy.name,
                settings: newSettings,
            };
        }
    } else {
       if (policyForm.value.id && policies.value.some(p => p.id === policyForm.value.id)) {
            // do nothing
       } else {
            resetPolicyForm();
       }
    }
});

watch(() => policyForm.value.name, (newVal) => {
    newCheckpointName.value = `${newVal}_${selectedCheckpoint.value ? 'finetuned' : ''}_${new Date().toISOString()}`;
});

watch(selectedPolicy, (newVal) => {
    if (selectedCheckpoint.value) return; // Don't react if a checkpoint is loaded

    if (newVal && newVal !== 'new') {
        const policy = policies.value.find(p => p.id === newVal);
        if (policy) {
            const newSettings = {};
            const config = POLICY_CONFIGS[policy.type];
            if (config) {
                for (const key in config) {
                    newSettings[key] = { ...config[key] }; // Copy complete config
                    if (policy.settings[key] !== undefined) {
                        newSettings[key].value = policy.settings[key]; // Override value
                    }
                }
            }
            policyForm.value = {
                ...JSON.parse(JSON.stringify(policy)),
                settings: newSettings
            };
        }
    } else if (newVal === 'new') {
        policyForm.value = {
            name: '',
            type: null,
            settings: {}
        };
    } else {
        resetPolicyForm();
    }
});


// --- Methods for Step 1 ---
function listDatasets() {
    api.get('/datasets').then((response) => {
        availableDatasets.value = response.data.datasets.filter(dataset => dataset.task_id == taskId) || [];
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
        Notify.create({ color: 'negative', message: 'Failed to load datasets.' });
    });
}

function selectDataset(dataset) {
    selectedDatasets.value.push(dataset);
    availableDatasets.value = availableDatasets.value.filter(d => d.id !== dataset.id);
}

function deselectDataset(dataset) {
    availableDatasets.value.push(dataset);
    selectedDatasets.value = selectedDatasets.value.filter(d => d.id !== dataset.id);
    availableDatasets.value.sort((a, b) => a.id - b.id);
}

// --- Methods for Step 2 ---
function listCheckpoints() {
    return api.get('/checkpoints').then(response => {
        checkpoints.value = response.data.checkpoints.map(c => {
            const policy = policies.value.find(p => p.id === c.policy_id);
            return {...c, policy: policy};
        })
    });
}

function listPolicies() {
    return api.get('/policies').then(response => {
        policies.value = response.data.policies;
    })
}

function handleCheckpointClear() {
    selectedCheckpoint.value = null;
    resetPolicyForm();
}

function handleCreateNewPolicy() {
    selectedPolicy.value = 'new';
}

function handlePolicyTypeChange(type) {
    if (selectedPolicy.value === 'new') {
        policyForm.value.settings = JSON.parse(JSON.stringify(POLICY_CONFIGS[type]));
    }
}

function resetPolicyForm() {
    selectedPolicy.value = null;
    policyForm.value = {};
}

function getPolicyPayload() {
    const policyToSave = JSON.parse(JSON.stringify(policyForm.value));
    const simpleSettings = {};
    if (policyToSave.settings) {
        for (const key in policyToSave.settings) {
            simpleSettings[key] = policyToSave.settings[key].value;
        }
    }
    policyToSave.settings = simpleSettings;
    return policyToSave;
}

function deletePolicy(policyId) {
    $q.notify({
        message: 'Are you sure you want to delete this policy?',
        color: 'negative',
        icon: 'warning',
        position: 'top',
        timeout: 0, // persistent
        actions: [
            {
                label: 'Confirm',
                color: 'white',
                handler: async () => {
                    try {
                        await api.delete(`/policy/${policyId}`);
                        await listPolicies();
                        if (selectedPolicy.value === policyId) {
                            resetPolicyForm();
                        }
                        Notify.create({ color: 'positive', message: 'Policy deleted successfully!' });
                    } catch (error) {
                        Notify.create({ color: 'negative', message: error + ': Failed to delete policy.' });
                    }
                }
            },
            { label: 'Cancel', color: 'white', handler: () => {} }
        ]
    });
}


async function savePolicy() {
    if (!validatePolicyForm()) {
        return; // Stop if validation fails
    }

    const payload = getPolicyPayload();
    if (selectedPolicy.value === 'new') { // Create new policy
        try {
            console.log(payload)
            const response = await api.post('/policy', payload);
            await listPolicies();
            selectedPolicy.value = response.data.id; // Assuming API returns the new ID
            Notify.create({ color: 'positive', message: 'Policy created successfully!' });
        } catch (error) {
            Notify.create({ color: 'negative', message: `${error}: Failed to create policy.` });
        }
    } else { // Update existing policy
        try {
            await api.put(`/policy/${payload.id}`, payload);
            await listPolicies();
            Notify.create({ color: 'positive', message: 'Policy updated successfully!' });
        } catch (error) {
            Notify.create({ color: 'negative', message: `${error}: Failed to update policy.` });
        }
    }
}

async function savePolicyAs() {
    if (!newPolicyName.value) {
        Notify.create({ color: 'warning', message: 'Please enter a name for the new policy.' });
        return;
    }
    const newPolicy = {
        ...getPolicyPayload(),
        name: newPolicyName.value,
    };
    delete newPolicy.id; // Remove id to create a new entry

    try {
        const response = await api.post('/policy', newPolicy);
        await listPolicies();
        selectedPolicy.value = response.data.id;
        newPolicyName.value = '';
        showSaveAsDialog.value = false;
        Notify.create({ color: 'positive', message: 'Policy saved as new entry.' });
    } catch (error) {
        Notify.create({ color: 'negative', message: `${error}: Failed to save policy.` });
    }
}


// --- Stepper Navigation ---
function nextStep() {
    if (step.value === 1 && selectedDatasets.value.length === 0) {
        Notify.create({ color: 'negative', message: 'Please select at least one dataset.' });
        return;
    }
    if (step.value === 2 && selectedPolicy.value) {
        if (selectedPolicy.value === 'new') {
            console.log(selectedPolicy.value)
            Notify.create({ color: 'negative', message: 'Please save a new policy.' });
            return;
        }
    }
    // It's good practice to also validate before proceeding
    if (step.value === 2 && policyForm.value.name !== undefined) {
        if (!validatePolicyForm()) {
            return; // Stop if validation fails
        }
    }

    if (step.value === 2 && !selectedPolicy.value) {
        Notify.create({ color: 'negative', message: 'Please select a policy.' });
        return;
    }

    if (step.value < 3) {
        stepper.value.next();
    } else {
        Notify.create({ color: 'positive', message: 'Training setup complete!' });
    }
}

function deleteCheckpoint(checkpointId) {
    $q.notify({
        message: 'Are you sure you want to delete this checkpoint?',
        color: 'negative',
        icon: 'warning',
        position: 'top',
        timeout: 0, // persistent
        actions: [
            {
                label: 'Confirm',
                color: 'white',
                handler: async () => {
                    try {
                        await api.delete(`/checkpoint/${checkpointId}`);
                        await listCheckpoints();
                        if (selectedCheckpoint.value === checkpointId) {
                            resetCheckpointForm();
                        }
                        Notify.create({ color: 'positive', message: 'Checkpoint deleted successfully!' });
                    } catch (error) {
                        Notify.create({ color: 'negative', message: `${error}: Failed to delete checkpoint.` });
                    }
                }
            },
            { label: 'Cancel', color: 'white', handler: () => {} }
        ]
    });
}

function resetCheckpointForm() {
    selectedCheckpoint.value = null;
}

async function startTraining() {
    console.log(selectedCheckpoint.value)
    isLoadingApi.value = true;
    try {
        api.post('/task:start_training', {
            task_id: taskId,
            policy_id: selectedPolicy.value,
            load_model_id: selectedCheckpoint.value,
            num_epochs: trainingForm.value.numEpochs,
            batch_size: trainingForm.value.batchSize,
            learning_rate: trainingForm.value.learning_rate,
            lr_backbone: trainingForm.value.lr_backbone,
            dataset_info: Object.fromEntries(selectedDatasets.value.map(d => [d.id, { episode_num: d.episode_num }])),
            name: newCheckpointName.value,
        }).then(() => {
            getTrainingCheckpoint()
        });


        // The UI state will now be updated by the 'start_process' socket event.
        // We can anticipate the training state to be true.
    } catch (error) {
        Notify.create({ color: 'negative', message: `${error}: Failed to start training: ` });
    } finally {
        listCheckpoints();
        isLoadingApi.value = false;
    }
}

async function stopTraining() {
    if (!isTraining.value) return;
    try {
        await api.post('/task:stop_training');
    } catch (error) {
        Notify.create({ color: 'negative', message: `${error}: Failed to stop training.` });
    }
}

const trainingCheckpoint = ref(null);
function getTrainingCheckpoint() {
    return api.get(`/checkpoints`, {
        params: {
            where: `task_id,=,${taskId}|is_training,=,1`
        }
    }).then(response => {
        trainingCheckpoint.value = response.data.checkpoints.at(-1) || null;
        newCheckpointName.value = trainingCheckpoint.value.name || '';
        trainingForm.value.numEpochs = trainingCheckpoint.value.num_epochs || 1000;
        trainingForm.value.batchSize = trainingCheckpoint.value.batch_size || 32;
        trainingForm.value.learning_rate = trainingCheckpoint.value.learning_rate || 1e-5;
        trainingForm.value.lr_backbone = trainingCheckpoint.value.lr_backbone || 1e-6;
    })
}

const trainingProgress = ref(0);
onMounted(async () => {
    listDatasets();
    await listPolicies(); // Wait for policies to load first
    listCheckpoints()
    const taskResponse = await api.get(`/tasks/${taskId}`);
    taskVal.value = taskResponse.data.task;

    if (isTraining.value) {
        step.value = 3;
        getTrainingCheckpoint()
    }

    socket.on('stop_process', (data) => {
        if (data.id === 'train_task') {

            api.get(`/checkpoint/${trainingCheckpoint.value.id}/:check_create_successed`).then(response => {
                if (response.data.check_create_successed) {
                    Notify.create({ color: 'positive', message: 'Training completed successfully.' });
                } else {
                    console.log(response.data);
                    Notify.create({ color: 'negative', message: 'Training failed.' });
                }
            }).catch(error => {
                Notify.create({ color: 'negative', message: `Error stopping training: ${error}` });
            });
        }
    });

    socket.on('log_train_task', (data) => {
        if (data.log.includes('Epoch ')) {
            const epochMatch = data.log.replace('Epoch ', '');
            trainingProgress.value = eval(epochMatch);
        }
    });
});
</script>

<style scoped>
/* Add any specific styles for this page here */
</style>