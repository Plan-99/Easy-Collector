<template>
    <q-page class="q-pt-md full-height column q-pr-md">
        <div class="border-rounded bg-secondary q-pa-md q-mb-md row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-md">{{ $t('trainIntroTitle') }}</div>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="selectedWorkspaceId"
                        :options="workspaces"
                        :label="$t('workspaceSelectLabel')"
                        style="width: 400px"
                        class="q-ml-md"
                        map-options
                        emit-value
                        option-label="name"
                        option-value="id"
                        :disable="pageLoading"
                    >
                    </q-select>
                </div>
                <div class="text-body text-white">{{ $t('trainIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('trainIntroBody2') }}</div>
            </div>
        </div>

        <TutorialHint class="q-mb-md" :text="$t('tutorialTrainIntro')" />

        <div class="col q-mb-md border-rounded border-grey flex-center flex column" v-if="pageLoading">
            <q-spinner-gears size="50px" color="primary" class="q-mb-md" />
            <div class="text-h6 text-grey">{{ $t('plannerInitializing') }}</div>
        </div>
        <div class="col q-mb-md border-rounded border-grey text-grey flex-center flex text-h6" v-else-if="!selectedWorkspaceId">
            {{ $t('selectWorkspaceFirst') }}
        </div>
        <q-stepper v-model="step" ref="stepper" animated dark v-else flat class="col" active-color="accent" done-color="accent" >
            <q-step
                :name="1"
                :title="$t('trainStep1Title')"
                icon="collections_bookmark"
                :done="step > 1"
                class="border-white border-rounded q-mt-md bg-secondary col"
            >
                <div class="text-h6 q-mb-md">{{ $t('trainStep1Heading') }}</div>
                <TutorialHint step="1" class="q-mb-md" :text="$t('tutorialTrainStep1')" />
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
                    <q-btn @click="nextStep" color="primary" outline :label="$t('continueBtn')"/>
                </q-stepper-navigation>
            </q-step>

            <q-step
                :name="2"
                :title="$t('trainStep2Title')"
                icon="policy"
                :done="step > 2"
                class="border-white border-rounded q-mt-md bg-secondary col"
            >
                <div class="text-h6 q-mb-md">{{ $t('trainStep2Heading') }}</div>
                <TutorialHint step="2" class="q-mb-md" :text="$t('tutorialTrainStep2')" />
                <div class="row">
                    <q-scroll-area class="col-12" style="height: 420px">                
                        <div class="row q-col-gutter-md">
                            <div class="col-6">
                                <q-select
                                    outlined
                                    v-model="selectedCheckpoint"
                                    :options="checkpointOptions"
                                    :label="$t('trainPolicyLoadFromCheckpoint')"
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
                                    :label="$t('trainPolicySelect')"
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
                                                <q-item-label class="text-grey">{{ $t('trainPolicyCreateNew') }}</q-item-label>
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
                                        <q-input dense outlined v-model="policyForm.name" class="bg-secondary" dark :label="$t('trainPolicyName')" :readonly="isNameReadonly" />
                                    </div>
                                    <div class="col-6">
                                        <q-select dense outlined v-model="policyForm.type" :options="policyTypes" dark class="bg-secondary" :label="$t('trainPolicyType')" :readonly="isTypeReadonly" @update:model-value="handlePolicyTypeChange" />
                                    </div>
                                    <div
                                        v-for="(config, key) in policyForm.settings"
                                        :key="key"
                                        class="col-4"
                                        v-show="!config.showIf || (policyForm.settings[config.showIf] && policyForm.settings[config.showIf].value)"
                                    >
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
                                        <q-select
                                            v-else-if="config.type === 'multiselect'"
                                            dense
                                            outlined
                                            v-model="config.value"
                                            :options="config.options"
                                            :label="config.label"
                                            :readonly="isSettingsReadonly"
                                            :disable="isSettingsReadonly"
                                            emit-value
                                            map-options
                                            multiple
                                            use-chips
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
                                            :options="[{ label: config.label, value: true }, { label: $t('no'), value: false }]"
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
                                            v-else-if="config.type === 'password'"
                                            dense
                                            outlined
                                            v-model="config.value"
                                            :label="config.label"
                                            :readonly="isSettingsReadonly"
                                            :disable="isSettingsReadonly"
                                            :type="passwordVisibility[key] ? 'text' : 'password'"
                                            class="full-height bg-secondary"
                                            dark
                                            autocomplete="off"
                                        >
                                            <template v-slot:append>
                                                <q-icon
                                                    :name="passwordVisibility[key] ? 'visibility' : 'visibility_off'"
                                                    class="cursor-pointer"
                                                    @click="passwordVisibility[key] = !passwordVisibility[key]"
                                                />
                                            </template>
                                        </q-input>
                                        <q-input
                                            v-else-if="config.type === 'array'"
                                            dense
                                            outlined
                                            :model-value="Array.isArray(config.value) ? config.value.join(', ') : config.value"
                                            :label="config.label + ' (comma-separated)'"
                                            :readonly="isSettingsReadonly"
                                            :disable="isSettingsReadonly"
                                            type="text"
                                            class="full-height bg-secondary"
                                            dark
                                            @update:model-value="(v) => {
                                                const trimmed = String(v).trim()
                                                if (!trimmed) { config.value = []; return }
                                                config.value = trimmed.split(',').map(s => {
                                                    const n = Number(s.trim())
                                                    return isNaN(n) ? s.trim() : n
                                                }).filter(x => x !== '')
                                            }"
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
                    <q-btn @click="step = 1" color="grey" outline :label="$t('back')"/>
                    <q-btn @click="savePolicy" color="primary" outline :label="$t('save')" v-if="selectedPolicy === 'new'" />
                    <q-btn @click="showSaveAsDialog = true" color="primary" outline :label="$t('saveAs')" v-if="isPolicyFormChanged && selectedPolicy !== 'new'"/>
                    <q-btn @click="nextStep" color="primary" outline :label="$t('continueBtn')" v-if="!isPolicyFormChanged && selectedPolicy !== 'new'" />
                </q-stepper-navigation>
            </q-step>

            <q-step
                :name="3"
                :title="$t('trainStep3Title')"
                icon="model_training"
                class="border-white border-rounded q-mt-md bg-secondary col"

            >
                <div class="text-h6 q-mb-md">{{ $t('trainStep3Heading') }}</div>
                <TutorialHint step="3" class="q-mb-md" :text="$t('tutorialTrainStep3')" />
                <q-scroll-area class="col-12" style="height: 420px">
                    <div class="q-gutter-y-md">
                        <!-- Training Server -->
                        <q-card class="bg-dark q-pa-md q-mb-md border-rounded">
                            <div class="row items-center q-gutter-x-md">
                                <q-input
                                    dense
                                    outlined
                                    v-model="remoteServerUrl"
                                    :label="$t('trainServerUrl')"
                                    :placeholder="$t('trainServerUrlPlaceholder')"
                                    class="col bg-secondary"
                                    dark
                                >
                                    <template v-slot:append>
                                        <q-btn
                                            flat
                                            dense
                                            :icon="serverStatus === 'connected' ? 'check_circle' : serverStatus === 'checking' ? 'hourglass_empty' : 'wifi_off'"
                                            :color="serverStatus === 'connected' ? 'positive' : serverStatus === 'checking' ? 'warning' : 'negative'"
                                            @click="checkServerHealth"
                                            :loading="serverStatus === 'checking'"
                                        />
                                    </template>
                                </q-input>
                                <q-badge v-if="serverGpuAvailable" color="positive" :label="$t('trainServerGpuAvailable')" class="q-ml-sm" />
                            </div>
                        </q-card>

                        <q-form class="q-col-gutter-md row">
                            <div class="col-12">
                                <q-input dense outlined v-model="newCheckpointName" :label="$t('trainCheckpointName')"
                                    class="bg-dark" dark
                                />
                            </div>

                            <div v-for="(config, key) in trainingForm" :key="key" class="col-4" v-show="!config.showIf || (trainingForm[config.showIf] && trainingForm[config.showIf].value)">
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
                                    :options="[{ label: config.label, value: true }, { label: $t('no'), value: false }]"
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
                    <q-btn @click="step = 2" color="grey" outline :label="$t('back')"/>
                    <q-btn
                        @click="createCheckpoint"
                        color="primary"
                        outline
                        :label="$t('trainStartTraining')"
                        :loading="startingTraining"
                        :disable="startingTraining"
                    />
                </q-stepper-navigation>
            </q-step>
        </q-stepper>

        <TrainingQueuePanel ref="queuePanel" @watch="watchCheckpoint" />

        <q-dialog v-model="showSaveAsDialog">
            <q-card style="min-width: 350px" dark>
                <q-card-section>
                    <div class="text-h6">{{ $t('trainSavePolicyAs') }}</div>
                </q-card-section>
                <q-card-section class="q-pt-none">
                    <q-input dense outlined v-model="newPolicyName" dark :label="$t('trainNewPolicyName')" autofocus />
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat :label="$t('cancel')" v-close-popup />
                    <q-btn flat :label="$t('save')" color="primary" @click="savePolicyAs" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>
        <TrainingDialog v-model="showTrainingDialog" :checkpoint="watchingCheckpoint" :isTraining="watchingIsTraining" @hide="unwatchCheckpoint" @cpRemoved="queuePanel?.refresh()" />
    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch } from 'vue';
import { useQuasar, Notify } from 'quasar';
import { api } from 'src/boot/axios';
import { POLICY_CONFIGS, TRAIN_CONFIGS } from 'src/configs/modelConfigs';
// import { useTraining } from 'src/composables/useTraining';
import TrainingDialog from 'src/components/v2/TrainingDialog.vue';
import TrainingQueuePanel from 'src/components/v2/TrainingQueuePanel.vue';
import TutorialHint from 'src/components/v2/TutorialHint.vue';
import { useSocket } from 'src/composables/useSocket';
import { useI18n } from 'vue-i18n';

const { t } = useI18n();

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
const passwordVisibility = ref({});
const showSaveAsDialog = ref(false);
const newPolicyName = ref('');
const newCheckpointName = ref('');

// Step 3: Training
const trainingForm = ref({});

// Remote Training Server
// 사용자가 한 번이라도 명시 입력했으면 localStorage 값을 그대로 쓰고, 없으면
// 백엔드에 default-url을 물어 (로컬 training_server 살아있으면 localhost,
// 아니면 공용 원격) 응답을 기본값으로 채워준다.
const remoteServerUrl = ref(localStorage.getItem('remoteTrainingServerUrl') || '');
const serverStatus = ref('unknown'); // 'unknown', 'checking', 'connected', 'error'
const serverGpuAvailable = ref(false);

if (!remoteServerUrl.value) {
    api.get('/remote-train/default-url')
        .then((res) => {
            if (!remoteServerUrl.value && res?.data?.default_url) {
                remoteServerUrl.value = res.data.default_url;
            }
        })
        .catch(() => {
            // 백엔드 미응답 시 공용 원격을 안전한 기본값으로
            if (!remoteServerUrl.value) {
                remoteServerUrl.value = 'http://easytrainer.training_server.com';
            }
        });
}

watch(remoteServerUrl, (newVal) => {
    localStorage.setItem('remoteTrainingServerUrl', newVal);
    serverStatus.value = 'unknown';
});

function checkServerHealth() {
    if (!remoteServerUrl.value) {
        Notify.create({ color: 'negative', message: t('trainEnterServerUrl') });
        return;
    }
    serverStatus.value = 'checking';
    api.post('/remote-train/health', { server_url: remoteServerUrl.value })
        .then((res) => {
            serverStatus.value = 'connected';
            serverGpuAvailable.value = res.data.server?.gpu_available || false;
            Notify.create({ color: 'positive', message: t('trainServerConnected') });
        })
        .catch(() => {
            serverStatus.value = 'error';
            serverGpuAvailable.value = false;
            Notify.create({ color: 'negative', message: t('trainServerCannotConnect') });
        });
}

const policyTypes = Object.keys(POLICY_CONFIGS);

const { socket } = useSocket();

const selectedWorkspaceId = ref(null);
const pageLoading = ref(true);
const startingTraining = ref(false);
const workspaces = ref([]);
watch(selectedWorkspaceId, (newVal) => {
    if (newVal) {
        listDatasets();
        listPolicies();
        listCheckpoints();
        // 큐 상태는 TrainingQueuePanel이 자체 fetch + socketio로 갱신.
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
        { label: t('trainPolicyStartWithoutCheckpoint'), value: null },
        ...checkpoints.value.filter(c => c.task_id === selectedWorkspaceId.value).map(c => ({ label: c.name, value: c.id }))
    ];
});

const policyOptions = computed(() => {
    const options = policies.value.map(p => ({ label: p.name, value: p.id }));
    options.push({ label: t('trainPolicyCreateNew'), value: 'new' });
    return options;
});

const formTitle = computed(() => {
    if (selectedCheckpoint.value) return t('trainPolicyFormFinetune');
    if (selectedPolicy.value === 'new') return t('trainPolicyFormCreate');
    if (selectedPolicy.value) return t('trainPolicyFormEdit');
    return '';
});

const isNameReadonly = computed(() => !!selectedCheckpoint.value || (selectedPolicy.value && selectedPolicy.value !== 'new'));
const isTypeReadonly = computed(() => !!selectedCheckpoint.value || (selectedPolicy.value && selectedPolicy.value !== 'new'));
const isSettingsReadonly = computed(() => !!selectedCheckpoint.value);

function validatePolicyForm() {
    const form = policyForm.value;

    if (!form.name || form.name.trim() === '') {
        Notify.create({ color: 'negative', message: t('trainPolicyNameRequired') });
        return false;
    }
    if (!form.type) {
        Notify.create({ color: 'negative', message: t('trainPolicyTypeRequired') });
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
                Notify.create({ color: 'negative', message: t('trainSettingRequired', { label: setting.label }) });
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
        Notify.create({ color: 'negative', message: t('trainLoadDatasetsFailed') });
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
        message: t('trainConfirmDeletePolicy'),
        color: 'negative',
        icon: 'warning',
        position: 'top',
        timeout: 0, // persistent
        actions: [
            {
                label: t('confirm'),
                color: 'white',
                handler: async () => {
                    try {
                        await api.delete(`/policy/${policyId}`);
                        await listPolicies();
                        if (selectedPolicy.value === policyId) {
                            resetPolicyForm();
                        }
                        Notify.create({ color: 'positive', message: t('trainPolicyDeleted') });
                    } catch (error) {
                        Notify.create({ color: 'negative', message: t('trainPolicyDeleteFailed', { error }) });
                    }
                }
            },
            { label: t('cancel'), color: 'white', handler: () => {} }
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
            Notify.create({ color: 'positive', message: t('trainPolicyCreated') });
        } catch (error) {
            Notify.create({ color: 'negative', message: t('trainPolicyCreateFailed', { error }) });
        }
    } else { // Update existing policy
        try {
            await api.put(`/policy/${payload.id}`, payload);
            await listPolicies();
            Notify.create({ color: 'positive', message: t('trainPolicyUpdated') });
        } catch (error) {
            Notify.create({ color: 'negative', message: t('trainPolicyUpdateFailed', { error }) });
        }
    }
}

async function savePolicyAs() {
    if (!newPolicyName.value) {
        Notify.create({ color: 'warning', message: t('trainPolicyEnterName') });
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
        Notify.create({ color: 'positive', message: t('trainPolicySavedAsNew') });
    } catch (error) {
        Notify.create({ color: 'negative', message: t('trainPolicySaveFailed', { error }) });
    }
}


// --- Stepper Navigation ---
function nextStep() {
    if (step.value === 1 && selectedDatasetIds.value.length === 0) {
        Notify.create({ color: 'negative', message: t('trainSelectAtLeastOneDataset') });
        return;
    }
    if (step.value === 2 && selectedPolicy.value) {
        if (selectedPolicy.value === 'new') {
            Notify.create({ color: 'negative', message: t('trainSaveNewPolicyFirst') });
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
        Notify.create({ color: 'negative', message: t('trainSelectPolicyFirst') });
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
        Notify.create({ color: 'positive', message: t('trainSetupComplete') });
    }
}

function deleteCheckpoint(checkpointId) {
    $q.notify({
        message: t('trainConfirmDeleteCheckpoint'),
        color: 'negative',
        icon: 'warning',
        position: 'top',
        timeout: 0, // persistent
        actions: [
            {
                label: t('confirm'),
                color: 'white',
                handler: async () => {
                    try {
                        await api.delete(`/checkpoint/${checkpointId}`);
                        await listCheckpoints();
                        if (selectedCheckpoint.value === checkpointId) {
                            resetCheckpointForm();
                        }
                        Notify.create({ color: 'positive', message: t('trainCheckpointDeleted') });
                    } catch (error) {
                        Notify.create({ color: 'negative', message: t('trainCheckpointDeleteFailed', { error }) });
                    }
                }
            },
            { label: t('cancel'), color: 'white', handler: () => {} }
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
const queuePanel = ref(null);

function createCheckpoint() {
    if (startingTraining.value) return;
    if (serverStatus.value !== 'connected') {
        Notify.create({ color: 'negative', message: t('trainConnectToServerFirst') });
        return;
    }

    startingTraining.value = true;
    const trainingPayload = getTrainingPayload();
    let createdCheckpointId = null;
    api.post('/checkpoint', {
        task_id: selectedWorkspaceId.value,
        policy_id: selectedPolicy.value,
        load_model_id: selectedCheckpoint.value,
        dataset_info: Object.fromEntries(selectedDatasetIds.value.map(d => [d, { episode_num: availableDatasets.value.find(ds => ds.id === d).episodes.length }])),
        name: newCheckpointName.value,
        train_settings: trainingPayload
    }).then((res) => {
        createdCheckpointId = res.data.id;
        // 큐에 추가 — 이미 다른 학습이 돌고 있으면 자동으로 기다린다.
        return enqueueTraining(createdCheckpointId);
    }).then(async () => {
        // 큐 패널 즉시 갱신 후, 방금 enqueue한 체크포인트의 최신 dict를
        // 가져와서 학습 다이얼로그를 자동으로 열어준다 (이전 UX 유지).
        await queuePanel.value?.refresh();
        try {
            const cpRes = await api.get(`/checkpoint/${createdCheckpointId}`);
            if (cpRes?.data?.checkpoint) {
                watchCheckpoint(cpRes.data.checkpoint);
            }
        } catch {
            // 다이얼로그 자동 오픈 실패는 비치명 — 사용자가 큐 패널에서 클릭 가능.
        }
    }).catch(error => {
        Notify.create({ color: 'negative', message: t('trainStartFailed', { error }) });
    }).finally(() => {
        listCheckpoints();
        startingTraining.value = false;
    });
}

function enqueueTraining(checkpoint_id) {
    return api.post('/train/queue', {
        server_url: remoteServerUrl.value,
        checkpoint_id: checkpoint_id,
    }).catch(error => {
        Notify.create({ color: 'negative', message: t('trainRemoteStartFailed', { error }) });
        throw error;
    });
}

// 시청 중인 체크포인트가 현재 running 상태인지 — TrainingDialog의 stop 버튼
// 활성/비활성 결정 + 그래프/progress 영역 v-if에 사용.
// status 필드만 보면 단순하고 reactive에도 안전.
const watchingIsTraining = computed(() => {
    return watchingCheckpoint.value?.status === 'running';
});

function watchCheckpoint(checkpoint) {
    watchingCheckpoint.value = checkpoint;
    showTrainingDialog.value = true;
}

// 큐 패널의 snapshot이 갱신될 때, 다이얼로그가 열려 있는 체크포인트의 fresh
// dict로 watchingCheckpoint를 동기화. status가 queued→running→finished 등으로
// 바뀌어도 다이얼로그 UI(progress/그래프 영역)와 버튼 라벨이 즉시 따라간다.
// 종료 상태에서 active 큐에 없으면 백엔드에 직접 GET해서 최신 status 반영.
watch(
    () => queuePanel.value?.queue,
    async (queue) => {
        if (!queue || !watchingCheckpoint.value) return;
        const id = watchingCheckpoint.value.id;
        const fresh =
            (queue.running && queue.running.id === id && queue.running) ||
            (queue.queued || []).find((c) => c.id === id) ||
            (queue.recent || []).find((c) => c.id === id);
        if (fresh) {
            watchingCheckpoint.value = fresh;
            return;
        }
        // active/recent 어디에도 없는데 다이얼로그는 열려 있는 상황 — 종료된 지
        // 오래되어 recent에서 밀려난 케이스. 직접 fetch해서 status를 갱신.
        try {
            const res = await api.get(`/checkpoint/${id}`);
            if (res?.data?.checkpoint) {
                watchingCheckpoint.value = res.data.checkpoint;
            }
        } catch {
            // 삭제된 경우 등 — 다이얼로그는 그대로 두고 사용자가 닫게 한다.
        }
    },
    { deep: true },
);

function unwatchCheckpoint() {
    watchingCheckpoint.value = null;
    showTrainingDialog.value = false;
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
    pageLoading.value = true;
    try {
        await listWorkspaces();
    } finally {
        pageLoading.value = false;
    }
    // listDatasets();
    // await listPolicies(); // Wait for policies to load first
    // await listCheckpoints();
    // await getUnfinishedCheckpoint();
    // const taskResponse = await api.get(`/tasks/${selectedWorkspaceId.value}`);
    // 학습 큐 변경은 TrainingQueuePanel이 직접 'train_queue_changed' 이벤트를 listen해
    // 자체 갱신한다. 여기서는 학습 종료 시 가벼운 알림만 처리.
    socket.on('train_queue_changed', () => {
        listCheckpoints();
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