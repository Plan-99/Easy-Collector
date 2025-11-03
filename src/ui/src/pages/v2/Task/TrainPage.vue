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
                                    v-if="key === 'pretrained_backbone_weights'"
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :options="pretrained_backbone_weight_options"
                                    :label="config.label"
                                    class="full-height"
                                />
                                <q-select
                                    v-else-if="config.type === 'select'"
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :options="config.options"
                                    :label="config.label"
                                    :readonly="isSettingsReadonly"
                                    :disable="isSettingsReadonly"
                                    emit-value
                                    map-options
                                    class="full-height"
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
                                    class="full-height"

                                />
                                <q-btn-toggle
                                    v-else-if="config.type === 'boolean'"
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :options="[{ label: config.label, value: true }, { label: 'No', value: false }]"
                                    :label="config.label"
                                    :readonly="isSettingsReadonly"
                                    :disable="isSettingsReadonly"
                                    spread
                                    class="full-height"
                                ></q-btn-toggle>
                                <q-input
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :label="config.label"
                                    :readonly="isSettingsReadonly"
                                    :disable="isSettingsReadonly"
                                    v-else
                                    type="text"
                                    class="full-height"
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
                    <q-form class="q-col-gutter-md row">
                        <div class="col-12">
                            <q-input dense outlined v-model="newCheckpointName" label="Checkpoint Name" />
                        </div>

                        <div v-for="(config, key) in trainingForm" :key="key" class="col-4">
                            <q-select
                                v-if="config.type === 'select'"
                                dense
                                outlined
                                v-model="config.value"
                                :options="config.options"
                                :label="config.label"
                                emit-value
                                map-options
                                class="full-height"
                            />
                            <q-input
                                dense
                                outlined
                                v-model.number="config.value"
                                :label="config.label"
                                v-else-if="config.type === 'number'"
                                type="number"
                                class="full-height"
                                step="any"
                            />
                            <q-btn-toggle
                                v-else-if="config.type === 'boolean'"
                                dense
                                outlined
                                v-model="config.value"
                                :options="[{ label: config.label, value: true }, { label: 'No', value: false }]"
                                :label="config.label"
                                spread
                                class="full-height"
                            ></q-btn-toggle>
                            <q-input
                                dense
                                outlined
                                v-model="config.value"
                                :label="config.label"
                                v-else
                                type="text"
                                class="full-height"
                            />
                        </div>
                    </q-form>
                    <div class="text-center">
                        <q-btn color="primary" label="Start Training" @click="createCheckpoint" />
                    </div>

                </div>
                
            </q-step>

            <template v-slot:navigation>
                <q-stepper-navigation class="text-center">
                    <q-btn v-if="step > 1" flat color="primary" @click="$refs.stepper.previous()" label="Back" class="q-mr-sm" />
                    <q-btn @click="nextStep" color="primary" :label="'Continue'" v-show="step !== 3" />
                </q-stepper-navigation>
            </template>
        </q-stepper>

        <q-page-sticky position="bottom-right" :offset="[18, 18]">
            <q-btn
                push
                round
                size="xl"
                :color="trainingCheckpoint.id === cp.id ? 'deep-orange' : 'grey-5'"
                :icon="trainingCheckpoint.id === cp.id ? 'model_training' : 'pending'"
                @click="watchCheckpoint(cp)"
                v-for="cp in unfinishedCheckpoints.toReversed()"
                :key="cp.id"
                class="q-ml-sm"
            />

        </q-page-sticky>

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
        <TrainingDialog v-model="showTrainingDialog" :checkpoint="watchingCheckpoint" :isTraining="watchingIsTraining" @hide="unwatchCheckpoint" @cpRemoved="getUnfinishedCheckpoint" />
    </div>
</template>

<script setup>
import { ref, onMounted, computed, watch } from 'vue';
import { useQuasar, Notify } from 'quasar';
import { useRoute } from 'vue-router';
import { api } from 'src/boot/axios';
import { POLICY_CONFIGS, TRAIN_CONFIGS } from 'src/configs/modelConfigs';
// import { useTraining } from 'src/composables/useTraining';
import TrainingDialog from 'src/components/TrainingDialog.vue';
import { useSocket } from 'src/composables/useSocket';
import { useProcessStore } from 'src/stores/processStore';


const processStore = useProcessStore();


const route = useRoute();
const taskId = Number(route.params.id);
// const { unfinishedCheckpoints, getUnfinishedCheckpoint, startTraining } = useTraining();

const $q = useQuasar();
const step = ref(1);
const stepper = ref(null);

// Step 1: Dataset Selection
const availableDatasets = ref([]);
const selectedDatasets = ref([]);

// Step 2: Policy Selection
const checkpoints = ref([]);
const policies = ref([]);
const selectedCheckpoint = ref(null);
const selectedPolicy = ref(null);
const policyForm = ref({});
const showSaveAsDialog = ref(false);
const newPolicyName = ref('');
const newCheckpointName = ref('');

const taskVal = ref({});

// Step 3: Training
const trainingForm = ref({});

const policyTypes = Object.keys(POLICY_CONFIGS);

const { socket } = useSocket();

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
            if (setting.nullable) {
                continue;
            }
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

const pretrained_backbone_weight_options = computed(() => {
    if (!policyForm.value.type || !POLICY_CONFIGS[policyForm.value.type]['pretrained_backbone_weight']) return [];
    if (!policyForm.value.settings || !policyForm.value.settings.vision_backbone) return [];
    const backbone = policyForm.value.settings.vision_backbone.value;
    return POLICY_CONFIGS[policyForm.value.type].pretrained_backbone_weights.options[backbone] || [];
});

watch(() => policyForm.value.settings?.vision_backbone?.value, (newVal) => {
    if (!policyForm.value.type || !POLICY_CONFIGS[policyForm.value.type]['pretrained_backbone_weight']) return;
    if (newVal && policyForm.value.type) {
        policyForm.value.settings.pretrained_backbone_weights.options = pretrained_backbone_weight_options.value;
        if (!policyForm.value.settings.pretrained_backbone_weights.value) {
            policyForm.value.settings.pretrained_backbone_weights.value = pretrained_backbone_weight_options.value[0] || null;
        }
        if (!pretrained_backbone_weight_options.value.includes(policyForm.value.settings.pretrained_backbone_weights.value)) {
            policyForm.value.settings.pretrained_backbone_weights.value = pretrained_backbone_weight_options.value[0] || null;
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
    return api.get('/checkpoints', {
        params: {
            where: `task_id,=,${taskId}|status,=,finished`,
            order: 'created_at DESC'
        }
    }).then(response => {
        checkpoints.value = response.data.checkpoints.map(c => {
            const policy = policies.value.find(p => p.id === c.policy_id);
            return {...c, policy: policy};
        })
    });
}

function listPolicies() {
    return api.get('/policies').then(response => {
        policies.value = response.data.policies.filter(policy => policy.is_vla !== 1) || [];
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

    if (step.value === 2) {
        const policyType = policyForm.value.type;
        const commonConfigs = JSON.parse(JSON.stringify(TRAIN_CONFIGS.Common));
        const policySpecificConfigs = policyType && TRAIN_CONFIGS[policyType] ? JSON.parse(JSON.stringify(TRAIN_CONFIGS[policyType])) : {};
        trainingForm.value = { ...commonConfigs, ...policySpecificConfigs };
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

function getTrainingPayload() {
    const trainingParams = {};
    if (trainingForm.value) {
        for (const key in trainingForm.value) {
            trainingParams[key] = trainingForm.value[key].value;
        }
    }
    return trainingParams;
}

const showTrainingDialog = ref(false);
const watchingCheckpoint = ref(null);
function createCheckpoint() {
    const trainingPayload = getTrainingPayload();
    api.post('/checkpoint', {
        task_id: taskId,
        policy_id: selectedPolicy.value,
        load_model_id: selectedCheckpoint.value,
        dataset_info: Object.fromEntries(selectedDatasets.value.map(d => [d.id, { episode_num: d.episode_num }])),
        name: newCheckpointName.value,
        train_settings: trainingPayload
    }).then((res) => {
        const checkpointId = res.data.id;
        getUnfinishedCheckpoint().then(() => {
            watchCheckpoint(unfinishedCheckpoints.value.find(cp => cp.id === checkpointId));
            startTraining(watchingCheckpoint.value.id);
        });
    }).catch(error => {
        Notify.create({ color: 'negative', message: `${error}: Failed to start training: ` });
    }).finally(() => {
        listCheckpoints();
    });
}

function startTraining(checkpoint_id) {
    api.post('/task:start_training', { checkpoint_id }).catch(error => {
        Notify.create({ color: 'negative', message: `Error starting training: ${error}` });
    })
}

const trainingCheckpoint = computed(() => {
    if (!unfinishedCheckpoints.value.length) return {};
    return unfinishedCheckpoints.value.find(cp => cp.status === 'training') || unfinishedCheckpoints.value.at();
});

const isTraining = computed(() => {
    return processStore.processIds.includes('train_task')
})

const watchingIsTraining = computed(() => {
    return watchingCheckpoint.value && trainingCheckpoint.value && watchingCheckpoint.value.id === trainingCheckpoint.value.id && isTraining.value;
}); 

const unfinishedCheckpoints = ref([]);
function getUnfinishedCheckpoint() {
    return api.get(`/checkpoints`, {
        params: {
            where: `status,!=,finished`,
        }
    }).then(response => {
        unfinishedCheckpoints.value = response.data.checkpoints
    })
}

function watchCheckpoint(checkpoint) {
    watchingCheckpoint.value = checkpoint;
    showTrainingDialog.value = true;
}

function unwatchCheckpoint() {
    watchingCheckpoint.value = null;
    showTrainingDialog.value = false;
}

function removeTrainingCheckpoint() {
    unfinishedCheckpoints.value = unfinishedCheckpoints.value.filter(cp => cp.id !== watchingCheckpoint.value?.id);
    unwatchCheckpoint();
}


onMounted(async () => {
    listDatasets();
    await listPolicies(); // Wait for policies to load first
    await listCheckpoints();
    await getUnfinishedCheckpoint();
    const taskResponse = await api.get(`/tasks/${taskId}`);
    taskVal.value = taskResponse.data.task;

    socket.on('stop_process', (data) => {
        if (data.id === 'train_task') {
            api.get(`/checkpoint/${trainingCheckpoint.value.id}/:check_create_successed`).then(response => {
                if (response.data.check_create_successed) {
                    Notify.create({ color: 'positive', message: 'Training completed successfully.' });
                } else {
                    Notify.create({ color: 'negative', message: 'Training failed.' });
                }
                removeTrainingCheckpoint()
            }).catch(error => {
                Notify.create({ color: 'negative', message: `Error stopping training: ${error}` });
            });
        }
    });

    socket.on('start_process', (data) => {
        if (data.id === 'train_task') {
            getUnfinishedCheckpoint()
        }
    });

    // if (isTraining.value) {
    //     step.value = 3;
    //     await getTrainingCheckpoint();
    //     if (trainingCheckpoint.value) {
    //         const policy = policies.value.find(p => p.id === trainingCheckpoint.value.policy_id);
    //         if (policy) {
    //             const policyType = policy.type;
    //             const commonConfigs = JSON.parse(JSON.stringify(TRAIN_CONFIGS.Common));
    //             const policySpecificConfigs = policyType && TRAIN_CONFIGS[policyType] ? JSON.parse(JSON.stringify(TRAIN_CONFIGS[policyType])) : {};
    //             const currentTrainingForm = { ...commonConfigs, ...policySpecificConfigs };

    //             // Populate with values from the checkpoint
    //             if (trainingCheckpoint.value.train_settings) {
    //                 for (const key in currentTrainingForm) {
    //                     if (trainingCheckpoint.value.train_settings[key] !== undefined) {
    //                         currentTrainingForm[key].value = trainingCheckpoint.value.train_settings[key];
    //                     }
    //                 }
    //             }
    //             trainingForm.value = currentTrainingForm;
    //         }
    //     }
    // }
});
</script>

<style scoped>
/* Add any specific styles for this page here */
</style>