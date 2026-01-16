<template>
    <q-page class="q-pt-lg full-height column q-pr-lg">
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-lg">{{ $t('trainIntroTitle') }}</div>
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
                    </q-select>
                </div>
                <div class="text-body text-white">{{ $t('trainIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('trainIntroBody2') }}</div>
            </div>
        </div>
        <div class="col q-mb-lg border-rounded border-grey text-grey flex-center flex text-h6" v-if="!selectedWorkspaceId">
            Select Workspace First
        </div>
        <q-stepper v-model="step" ref="stepper" animated dark v-else flat class="col" active-color="accent" done-color="accent" >
            <q-step
                :name="1"
                title="Select Datasets"
                icon="collections_bookmark"
                :done="step > 1"
                class="border-white border-rounded q-mt-lg bg-secondary col"
            >
                <div class="text-h6 q-mb-md">Select datasets for training</div>
                <q-scroll-area style="height: 420px">                
                    <div class="row q-col-gutter-md">
                        <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="dataset in availableDatasets" :key="dataset.id">
                            <q-card
                                class="q-pa-md bg-dark border-rounded border-white text-white"
                                :class="selectedDatasetIds.includes(dataset.id) ? 'border-primary' : ''"
                            >
                                <div class="absolute-top-right">
                                    <q-checkbox
                                        v-model="selectedDatasetIds"
                                        :val="dataset.id"
                                        dark
                                    />
                                </div>
                                <q-card-section class="q-pa-none q-mt-sm text-center">
                                    <q-img
                                        src="images/folder-icon.png"
                                        class="cursor-pointer"
                                        fit="contain"
                                        width="80px"
                                    >
                                        <div style="background: none;" class="absolute-full flex flex-center text-h6 text-primary q-mt-sm">
                                            {{ dataset.episodes.length }}
                                        </div>
                                    </q-img>
                                    <div class="text-bold q-mt-md">{{ dataset.name }}</div>
                                </q-card-section>
                            </q-card>
                        </div>
                    </div>
                </q-scroll-area>
                <q-stepper-navigation align="right" class="q-mt-md">
                    <q-btn @click="nextStep" color="primary" outline :label="'Continue'"/>
                </q-stepper-navigation>
            </q-step>

            <q-step
                :name="2"
                title="Select Policy"
                icon="policy"
                :done="step > 2"
                class="border-white border-rounded q-mt-lg bg-secondary col"
            >
                <div class="text-h6 q-mb-md">Configure Policy</div>
                <div class="row">
                    <q-scroll-area class="col-12" style="height: 420px">                
                        <div class="row q-col-gutter-md">
                            <div class="col-6">
                                <q-select
                                    outlined
                                    v-model="selectedCheckpoint"
                                    :options="checkpointOptions"
                                    label="Load Model from Checkpoint"
                                    emit-value
                                    map-options
                                    clearable
                                    dark
                                    dense
                                    bg-color="dark"
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
                            <div class="col-6">
                                <q-select
                                    outlined
                                    v-model="selectedPolicy"
                                    :options="policyOptions"
                                    label="Select Policy"
                                    emit-value
                                    map-options
                                    :disable="!!selectedCheckpoint"
                                    dark
                                    dense
                                    bg-color="dark"
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
                        <q-card class="q-mt-md border-rounded bg-dark" v-if="policyForm.name !== undefined">
                            <q-card-section class="text-h6 text-center">
                                {{ formTitle }}
                            </q-card-section>
                            <q-card-section>
                                <q-form class="q-col-gutter-md row">
                                    <div class="col-6">
                                        <q-input dense outlined v-model="policyForm.name" class="bg-secondary" dark label="Policy Name" :readonly="isNameReadonly" />
                                    </div>
                                    <div class="col-6">
                                        <q-select dense outlined v-model="policyForm.type" :options="policyTypes" dark class="bg-secondary" label="Policy Type" :readonly="isTypeReadonly" @update:model-value="handlePolicyTypeChange" />
                                    </div>
                                    <div v-for="(config, key) in policyForm.settings" :key="key" class="col-4">
                                        <q-select
                                            v-if="key === 'pretrained_backbone_weights'"
                                            dense
                                            outlined
                                            v-model="config.value"
                                            :options="pretrained_backbone_weight_options"
                                            :label="config.label"
                                            class="full-height bg-secondary"
                                            dark
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
                                            class="full-height bg-secondary"
                                            dark
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
                                            class="full-height bg-secondary"
                                            dark
                                        />
                                        <q-btn-toggle
                                            v-else-if="config.type === 'boolean'"
                                            dense
                                            v-model="config.value"
                                            :options="[{ label: config.label, value: true }, { label: 'No', value: false }]"
                                            :label="config.label"
                                            :readonly="isSettingsReadonly"
                                            :disable="isSettingsReadonly"
                                            spread
                                            color="secondary"
                                            toggle-color="primary"
                                            toggle-text-color="dark"
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
                                            class="full-height bg-secondary"
                                            dark
                                        />
                                    </div>
                                </q-form>
                            </q-card-section>
                        </q-card>
                    </q-scroll-area>
                </div>
                <q-stepper-navigation align="right" class="q-mt-md q-gutter-x-md">
                    <q-btn @click="step = 1" color="grey" outline :label="'Back'"/>
                    <q-btn @click="savePolicy" color="primary" outline :label="'Save'" v-if="selectedPolicy === 'new'" />
                    <q-btn @click="showSaveAsDialog = true" color="primary" outline :label="'Save As'" v-if="isPolicyFormChanged && selectedPolicy !== 'new'"/>
                    <q-btn @click="nextStep" color="primary" outline :label="'Continue'" v-if="!isPolicyFormChanged && selectedPolicy !== 'new'" />
                </q-stepper-navigation>
            </q-step>

            <q-step
                :name="3"
                title="Train Model"
                icon="model_training"
                class="border-white border-rounded q-mt-lg bg-secondary col"

            >
                <div class="text-h6 q-mb-md">Training Configuration</div>
                <q-scroll-area class="col-12" style="height: 420px">
                    <div class="q-gutter-y-md">
                        <q-form class="q-col-gutter-md row">
                            <div class="col-12">
                                <q-input dense outlined v-model="newCheckpointName" label="Checkpoint Name" 
                                    class="bg-dark" dark
                                />
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
                                    class="full-height bg-dark"
                                    dark
                                />
                                <q-input
                                    dense
                                    outlined
                                    v-model.number="config.value"
                                    :label="config.label"
                                    v-else-if="config.type === 'number'"
                                    type="number"
                                    class="full-height bg-dark"
                                    step="any"
                                    dark
                                />
                                <q-btn-toggle
                                    v-else-if="config.type === 'boolean'"
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :options="[{ label: config.label, value: true }, { label: 'No', value: false }]"
                                    :label="config.label"
                                    spread
                                    class="full-height bg-dark"
                                ></q-btn-toggle>
                                <q-input
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :label="config.label"
                                    v-else
                                    type="text"
                                    dark
                                    class="full-height bg-dark"
                                />
                            </div>
                        </q-form>
                    </div>
                </q-scroll-area>
                <q-stepper-navigation align="right" class="q-mt-md q-gutter-x-md">
                    <q-btn @click="step = 2" color="grey" outline :label="'Back'"/>
                    <q-btn @click="createCheckpoint" color="primary" outline :label="'Start Training'"/>
                </q-stepper-navigation>
            </q-step>
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
            <q-card style="min-width: 350px" dark>
                <q-card-section>
                    <div class="text-h6">Save Policy As</div>
                </q-card-section>
                <q-card-section class="q-pt-none">
                    <q-input dense outlined v-model="newPolicyName" dark label="New Policy Name" autofocus />
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat label="Cancel" v-close-popup />
                    <q-btn flat label="Save" color="primary" @click="savePolicyAs" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>
        <TrainingDialog v-model="showTrainingDialog" :checkpoint="watchingCheckpoint" :isTraining="watchingIsTraining" @hide="unwatchCheckpoint" @cpRemoved="getUnfinishedCheckpoint" />
    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch } from 'vue';
import { useQuasar, Notify } from 'quasar';
import { api } from 'src/boot/axios';
import { POLICY_CONFIGS, TRAIN_CONFIGS } from 'src/configs/modelConfigs';
// import { useTraining } from 'src/composables/useTraining';
import TrainingDialog from 'src/components/v2/TrainingDialog.vue';
import { useSocket } from 'src/composables/useSocket';
import { useProcessStore } from 'src/stores/processStore';

const processStore = useProcessStore();

const $q = useQuasar();
const step = ref(1);
const stepper = ref(null);

// Step 1: Dataset Selection
const availableDatasets = ref([]);
const selectedDatasetIds = ref([]);

// Step 2: Policy Selection
const checkpoints = ref([]);
const policies = ref([]);
const selectedCheckpoint = ref(null);
const selectedPolicy = ref(null);
const policyForm = ref({});
const showSaveAsDialog = ref(false);
const newPolicyName = ref('');
const newCheckpointName = ref('');

// Step 3: Training
const trainingForm = ref({});

const policyTypes = Object.keys(POLICY_CONFIGS);

const { socket } = useSocket();

const selectedWorkspaceId = ref(null);
const workspaces = ref([]);
watch(selectedWorkspaceId, (newVal) => {
    if (newVal) {
        listDatasets();
        listPolicies();
        listCheckpoints();
        getUnfinishedCheckpoint();
    }
});

function listWorkspaces() {
    return api.get('/tasks').then((response) => {
        workspaces.value = response.data.tasks;
    });
}

// const selectedWorkpace = computed(() => {
//     return workspaces.value.find(w => w.id === selectedWorkspaceId.value);
// });
// --- Computed Properties for Step 2 ---
const checkpointOptions = computed(() => {
    return [
        { label: 'Start without checkpoint', value: null },
        ...checkpoints.value.filter(c => c.task_id === selectedWorkspaceId.value).map(c => ({ label: c.name, value: c.id }))
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
        availableDatasets.value = response.data.datasets.filter(dataset => dataset.task_id == selectedWorkspaceId.value) || [];
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
        Notify.create({ color: 'negative', message: 'Failed to load datasets.' });
    });
}

// --- Methods for Step 2 ---
function listCheckpoints() {
    return api.get('/checkpoints', {
        params: {
            where: `task_id,=,${selectedWorkspaceId.value}|status,=,finished`,
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
            console.log(response);
            selectedPolicy.value = response.data.data.id; // Assuming API returns the new ID
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
        selectedPolicy.value = response.data.data.id;
        newPolicyName.value = '';
        showSaveAsDialog.value = false;
        Notify.create({ color: 'positive', message: 'Policy saved as new entry.' });
    } catch (error) {
        Notify.create({ color: 'negative', message: `${error}: Failed to save policy.` });
    }
}


// --- Stepper Navigation ---
function nextStep() {
    if (step.value === 1 && selectedDatasetIds.value.length === 0) {
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
        task_id: selectedWorkspaceId.value,
        policy_id: selectedPolicy.value,
        load_model_id: selectedCheckpoint.value,
        dataset_info: Object.fromEntries(selectedDatasetIds.value.map(d => [d, { episode_num: availableDatasets.value.find(ds => ds.id === d).episodes.length }])),
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

const isPolicyFormChanged = computed(() => {
    if (!policyForm.value.id) return true; // New policy, no changes yet
    const originalPolicy = policies.value.find(p => p.id === policyForm.value.id);
    if (!originalPolicy) return false;

    if (originalPolicy.name !== policyForm.value.name) return true;
    if (originalPolicy.type !== policyForm.value.type) return true;

    for (const key in policyForm.value.settings) {
        if (originalPolicy.settings[key] !== policyForm.value.settings[key].value) {
            return true;
        }
    }
    return false;
});


onMounted(async () => {
    listWorkspaces();
    // listDatasets();
    // await listPolicies(); // Wait for policies to load first
    // await listCheckpoints();
    // await getUnfinishedCheckpoint();
    // const taskResponse = await api.get(`/tasks/${selectedWorkspaceId.value}`);
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