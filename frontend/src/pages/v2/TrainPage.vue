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
                                    <div class="text-bold q-mt-md ellipsis">{{ dataset.name }}</div>
                                </q-card-section>
                                <q-slide-transition>
                                    <q-card-section
                                        v-if="selectedDatasetIds.includes(dataset.id)"
                                        class="q-pa-none q-mt-md"
                                    >
                                        <div class="row items-center justify-between q-mb-xs">
                                            <div class="text-caption text-grey-5">
                                                {{ $t('trainDatasetWeight') }}
                                            </div>
                                            <div class="text-caption text-primary text-weight-bold">
                                                ×{{ getWeightLabel(getDatasetWeight(dataset.id)) }}
                                            </div>
                                        </div>
                                        <q-btn-toggle
                                            :model-value="getDatasetWeight(dataset.id)"
                                            @update:model-value="setDatasetWeight(dataset.id, $event)"
                                            :options="weightOptions"
                                            spread
                                            no-caps
                                            unelevated
                                            dense
                                            size="sm"
                                            toggle-color="primary"
                                            color="dark"
                                            text-color="grey-4"
                                            class="border-grey border-rounded"
                                        >
                                            <q-tooltip>{{ $t('trainDatasetWeightTooltip') }}</q-tooltip>
                                        </q-btn-toggle>
                                    </q-card-section>
                                </q-slide-transition>
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
                <div class="row q-col-gutter-md">
                    <div class="col-12 col-md-5">
                        <div class="row items-center q-mb-xs no-wrap">
                            <div class="text-subtitle2 text-grey-4 ellipsis">{{ $t('trainPolicyLoadFromCheckpoint') }}</div>
                            <q-space />
                            <q-btn
                                dense
                                no-caps
                                size="sm"
                                icon="auto_awesome"
                                :label="$t('checkpointGraphZeroBase')"
                                :color="selectedCheckpoint ? 'grey-7' : 'primary'"
                                :outline="!!selectedCheckpoint"
                                @click="selectZeroBase"
                            />
                        </div>
                        <CheckpointGraph
                            :checkpoints="checkpoints"
                            :selected-id="selectedCheckpoint"
                            height="384px"
                            :empty-text="$t('checkpointGraphEmpty')"
                            @node-click="onGraphNodeClick"
                        >
                            <template #menu="{ checkpoint }">
                                <q-item
                                    v-if="checkpoint.status === 'finished'"
                                    clickable
                                    v-ripple
                                    v-close-popup
                                    @click="onGraphNodeClick(checkpoint)"
                                >
                                    <q-item-section>{{ $t('checkpointFinetuneFromHere') }}</q-item-section>
                                    <q-item-section side><q-icon name="model_training" size="xs" /></q-item-section>
                                </q-item>
                                <q-item
                                    v-if="checkpoint.status === 'running' || checkpoint.status === 'queued'"
                                    clickable
                                    v-ripple
                                    v-close-popup
                                    @click="watchCheckpoint(checkpoint)"
                                >
                                    <q-item-section>{{ $t('checkpointWatchTraining') }}</q-item-section>
                                    <q-item-section side><q-icon name="visibility" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup class="text-negative" @click="deleteCheckpoint(checkpoint.id)">
                                    <q-item-section>{{ $t('workspaceCheckpointDelete') }}</q-item-section>
                                    <q-item-section side><q-icon color="negative" name="delete" size="xs" /></q-item-section>
                                </q-item>
                            </template>
                        </CheckpointGraph>
                    </div>
                    <div class="col-12 col-md-7">
                        <q-scroll-area style="height: 420px">
                            <div class="row q-col-gutter-md">
                                <div class="col-12">
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
                                        v-show="(!config.showIf || (policyForm.settings[config.showIf] && policyForm.settings[config.showIf].value)) && (!config.showIfBackbone || (policyForm.settings.vision_backbone && policyForm.settings.vision_backbone.value && String(policyForm.settings.vision_backbone.value).startsWith(config.showIfBackbone))) && (!config.hideIfBackbone || !(policyForm.settings.vision_backbone && policyForm.settings.vision_backbone.value && String(policyForm.settings.vision_backbone.value).startsWith(config.hideIfBackbone)))"
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
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-select>
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
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-select>
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
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-select>
                                        <q-select
                                            v-else-if="config.type === 'wrist_sensor_select'"
                                            dense
                                            outlined
                                            v-model="config.value"
                                            :options="wristSensorOptions"
                                            :label="config.label"
                                            :readonly="isSettingsReadonly"
                                            :disable="isSettingsReadonly"
                                            emit-value
                                            map-options
                                            multiple
                                            use-chips
                                            class="full-height bg-secondary"
                                            dark
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-select>
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
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-input>
                                        <div v-else-if="config.type === 'boolean'" class="row items-center no-wrap">
                                            <q-btn-toggle
                                                dense
                                                v-model="config.value"
                                                :options="[{ label: config.label, value: true }, { label: $t('no'), value: false }]"
                                                :readonly="isSettingsReadonly"
                                                :disable="isSettingsReadonly"
                                                spread
                                                color="secondary"
                                                toggle-color="primary"
                                                toggle-text-color="dark"
                                                class="col full-height"
                                            />
                                            <HyperparamHelp :policy-type="policyForm.type" :param-key="key" class="q-ml-xs" />
                                        </div>
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
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
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
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }} (comma-separated)</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-input>
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
                                        >
                                            <template v-slot:label>
                                                <span>{{ config.label }}</span>
                                                <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                            </template>
                                        </q-input>
                                    </div>
                                </q-form>
                            </q-card-section>
                        </q-card>
                        </q-scroll-area>
                    </div>
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

                        <!-- GPU 예상 사용량 / 남은 GPU 메모리 -->
                        <q-card class="bg-dark q-pa-md q-mb-md border-rounded">
                            <div class="row items-center q-gutter-x-sm">
                                <q-icon name="memory" color="deep-orange" size="sm" />
                                <div class="text-subtitle2">{{ $t('trainGpuEstimateTitle') }}</div>
                                <q-space />
                                <q-btn flat dense round size="sm" icon="refresh" :loading="gpuEstimate.loading" @click="refreshGpuEstimate" />
                            </div>
                            <div class="row items-center q-gutter-x-xl q-mt-sm">
                                <div>
                                    <div class="text-caption text-grey-5">{{ $t('trainGpuEstimated') }}</div>
                                    <div class="text-h6">~{{ gpuEstimate.estimated_mib ?? '—' }} MiB</div>
                                </div>
                                <div>
                                    <div class="text-caption text-grey-5">{{ $t('trainGpuFree') }}</div>
                                    <div class="text-h6">
                                        <!-- 학습 서버 연결이 확인되기 전엔 남은 GPU 를 표시하지 않는다 (- / -). -->
                                        <!-- 표시값은 진행 중인 학습 예약량을 차감한 available_mib (새 학습이 실제 쓸 수 있는 자리). -->
                                        <span v-if="serverStatus !== 'connected'" class="text-grey-5">- / -</span>
                                        <span v-else-if="gpuEstimate.available_mib != null">{{ gpuEstimate.available_mib }} / {{ gpuEstimate.total_mib }} MiB</span>
                                        <span v-else class="text-grey-5 text-body2">{{ $t('trainGpuUnknown') }}</span>
                                    </div>
                                </div>
                                <q-space />
                                <!-- 여유/대기 칩도 연결 확인 후에만 (남은 GPU 를 알 때만 의미 있음). -->
                                <template v-if="serverStatus === 'connected'">
                                    <q-chip v-if="gpuEstimate.fits === true" color="positive" text-color="white" icon="check_circle" dense>{{ $t('trainGpuFits') }}</q-chip>
                                    <q-chip v-else-if="gpuEstimate.fits === false" color="orange" text-color="white" icon="schedule" dense>{{ $t('trainGpuWillQueue') }}</q-chip>
                                </template>
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
                                >
                                    <template v-slot:label>
                                        <span>{{ config.label }}</span>
                                        <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                    </template>
                                </q-select>
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
                                >
                                    <template v-slot:label>
                                        <span>{{ config.label }}</span>
                                        <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                    </template>
                                </q-input>
                                <div v-else-if="config.type === 'boolean'" class="row items-center no-wrap">
                                    <q-btn-toggle
                                        dense
                                        outlined
                                        v-model="config.value"
                                        :options="[{ label: config.label, value: true }, { label: $t('no'), value: false }]"
                                        spread
                                        class="col full-height bg-dark"
                                    />
                                    <HyperparamHelp :policy-type="policyForm.type" :param-key="key" class="q-ml-xs" />
                                </div>
                                <q-select
                                    v-else-if="config.type === 'wrist_sensor_select'"
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :options="wristSensorOptions"
                                    :label="config.label"
                                    emit-value
                                    map-options
                                    multiple
                                    use-chips
                                    class="full-height bg-dark"
                                    dark
                                >
                                    <template v-slot:label>
                                        <span>{{ config.label }}</span>
                                        <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                    </template>
                                </q-select>
                                <q-input
                                    dense
                                    outlined
                                    v-model="config.value"
                                    :label="config.label"
                                    v-else
                                    type="text"
                                    dark
                                    class="full-height bg-dark"
                                >
                                    <template v-slot:label>
                                        <span>{{ config.label }}</span>
                                        <HyperparamHelp :policy-type="policyForm.type" :param-key="key" />
                                    </template>
                                </q-input>
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
        <TrainingDialog v-model="showTrainingDialog" :checkpoint="watchingCheckpoint" :isTraining="watchingIsTraining" :detail-loading="watchingDetailLoading" @hide="unwatchCheckpoint" @cpRemoved="queuePanel?.refresh()" />
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
import HyperparamHelp from 'src/components/v2/HyperparamHelp.vue';
import CheckpointGraph from 'src/components/v2/CheckpointGraph.vue';
import { useSocket } from 'src/composables/useSocket';
import { useI18n } from 'vue-i18n';

const { t } = useI18n();

const $q = useQuasar();
const step = ref(1);
const stepper = ref(null);

// Step 1: Dataset Selection
const availableDatasets = ref([]);
const selectedDatasetIds = ref([]);
// 데이터셋별 샘플링 비중 — key: dataset.id, value: 0.5 | 1 | 2 | 3 | 5.
// 기본값 1 (균등). 사용자가 카드에서 ×N 으로 바꿀 때 여기 들어감.
// 체크 해제했다가 다시 선택해도 값이 유지되도록 별도 dict 로 관리.
const datasetWeights = ref({});
const weightOptions = [
    { label: '×½', value: 0.5 },
    { label: '×1', value: 1 },
    { label: '×2', value: 2 },
    { label: '×3', value: 3 },
    { label: '×5', value: 5 },
];
function getDatasetWeight(id) {
    return datasetWeights.value[id] ?? 1;
}
function setDatasetWeight(id, v) {
    datasetWeights.value = { ...datasetWeights.value, [id]: v };
}
function getWeightLabel(v) {
    return v === 0.5 ? '½' : String(v);
}

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
            // 연결되면 학습 서버 남은 GPU(예상 사용량 카드)도 즉시 갱신.
            refreshGpuEstimate();
        })
        .catch(() => {
            serverStatus.value = 'error';
            serverGpuAvailable.value = false;
            Notify.create({ color: 'negative', message: t('trainServerCannotConnect') });
            // 연결 실패 시에도 카드의 남은 GPU 를 최신(측정 불가)으로 반영.
            refreshGpuEstimate();
        });
}

const policyTypes = Object.keys(POLICY_CONFIGS);

// ── GPU 예상 사용량 / 학습서버 GPU 여유 ────────────────────────────────────
// 현재 파라미터로 학습 시 예상 VRAM 과 대상 서버의 남은 VRAM 을 보여준다.
// 예상 > 여유면 곧바로 동시 실행되지 않고 대기열에서 순차 실행됨을 안내.
const gpuEstimate = ref({ estimated_mib: null, free_mib: null, total_mib: null, available_mib: null, running_count: 0, fits: null, loading: false });
let gpuEstimateTimer = null;
function refreshGpuEstimate() {
    const policyType = policyForm.value?.type;
    if (!policyType) return;
    const train_settings = {};
    if (trainingForm.value) {
        for (const key in trainingForm.value) train_settings[key] = trainingForm.value[key].value;
    }
    gpuEstimate.value.loading = true;
    api.post('/train/gpu-estimate', {
        policy_type: policyType,
        train_settings,
        server_url: remoteServerUrl.value,
    })
        .then((res) => {
            const d = res.data || {};
            gpuEstimate.value = {
                estimated_mib: d.estimated_mib ?? null,
                free_mib: d.free_mib ?? null,
                total_mib: d.total_mib ?? null,
                available_mib: d.available_mib ?? d.free_mib ?? null,
                running_count: d.running_count ?? 0,
                fits: d.fits ?? null,
                loading: false,
            };
        })
        .catch(() => {
            gpuEstimate.value.loading = false;
        });
}
function scheduleGpuEstimate() {
    if (gpuEstimateTimer) clearTimeout(gpuEstimateTimer);
    gpuEstimateTimer = setTimeout(refreshGpuEstimate, 400);
}
// 파라미터/서버/정책 변화 시 디바운스 재계산.
watch(
    () => [trainingForm.value, remoteServerUrl.value, policyForm.value?.type],
    scheduleGpuEstimate,
    { deep: true },
);
// step 3(학습 파라미터)에 들어오면 즉시 1회 계산.
watch(step, (s) => {
    if (s === 3) refreshGpuEstimate();
});

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
    // 드롭다운 용 — light 만 받음. 선택 시 ``ensureWorkspaceDetail`` 가 해당
    // workspace 에 sensors / assembly 를 lazy 로 채워넣어 wristSensorOptions 등
    // 풀 필드 의존 computed 가 동작.
    return api.get('/tasks').then((response) => {
        workspaces.value = response.data.tasks || [];
    });
}

// 선택된 workspace 의 풀 detail 을 lazy fetch + 원래 객체에 in-place merge.
// 이미 sensors 가 있으면 skip — 같은 페이지 세션에서 한 번만 가져옴.
async function ensureWorkspaceDetail(id) {
    const i = workspaces.value.findIndex(w => w.id === id);
    if (i < 0) return;
    if (workspaces.value[i].sensors) return;   // 이미 detail 들어있음
    try {
        const res = await api.get(`/tasks/${id}`);
        const detail = res.data?.task;
        if (detail) workspaces.value[i] = { ...workspaces.value[i], ...detail };
    } catch (e) {
        console.error('ensureWorkspaceDetail:', e);
    }
}
watch(selectedWorkspaceId, (v) => { if (v) ensureWorkspaceDetail(v); });

// wrist_sensor_select 필드용 옵션. 현재 선택된 workspace의 sensors 목록.
const wristSensorOptions = computed(() => {
    const w = workspaces.value.find(w => w.id === selectedWorkspaceId.value);
    return (w?.sensors || []).map(s => ({ label: `${s.name} (id=${s.id})`, value: s.id }));
});

// --- Step 2: 베이스 체크포인트 그래프 ---
// 그래프 노드 클릭: finished 면 파인튜닝 베이스로 선택(selectedCheckpoint watch가 정책 로드),
// 그 외(running/queued/failed) 면 학습 진행 다이얼로그를 연다.
function onGraphNodeClick(checkpoint) {
    if (checkpoint.status === 'finished') {
        selectedCheckpoint.value = checkpoint.id;
    } else {
        watchCheckpoint(checkpoint);
    }
}
// "처음부터 학습 (새 브랜치)" — 베이스 없이 zero-base 로 시작.
function selectZeroBase() {
    handleCheckpointClear();
}

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
                        let _val = policy.settings[key];
                        // legacy coercion: wrist_sensor_ids는 옛 schema에서 "2,3" 문자열
                        // 이었음. 새 multiselect는 [int, int] 배열을 기대.
                        if (key === 'wrist_sensor_ids' && typeof _val === 'string') {
                            _val = _val.split(',').map(s => parseInt(s.trim(), 10)).filter(n => !isNaN(n))
                        }
                        newSettings[key].value = _val; // Override value
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
    if (!policyForm.value.type || !POLICY_CONFIGS[policyForm.value.type]['pretrained_backbone_weights']) return [];
    if (!policyForm.value.settings || !policyForm.value.settings.vision_backbone) return [];
    const backbone = policyForm.value.settings.vision_backbone.value;
    return POLICY_CONFIGS[policyForm.value.type].pretrained_backbone_weights.options[backbone] || [];
});

watch(() => policyForm.value.settings?.vision_backbone?.value, (newVal) => {
    if (!policyForm.value.type || !POLICY_CONFIGS[policyForm.value.type]['pretrained_backbone_weights']) return;
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
                        let _val = policy.settings[key];
                        // legacy coercion: wrist_sensor_ids는 옛 schema에서 "2,3" 문자열
                        // 이었음. 새 multiselect는 [int, int] 배열을 기대.
                        if (key === 'wrist_sensor_ids' && typeof _val === 'string') {
                            _val = _val.split(',').map(s => parseInt(s.trim(), 10)).filter(n => !isNaN(n))
                        }
                        newSettings[key].value = _val; // Override value
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
        // 응답이 일시적으로 비거나 깨져도(동시 요청 시 dev 서버) 크래시하지 않도록 방어.
        availableDatasets.value = (response.data?.datasets || []).filter(dataset => dataset.task_id == selectedWorkspaceId.value);
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
        Notify.create({ color: 'negative', message: t('trainLoadDatasetsFailed') });
    });
}

// --- Methods for Step 2 ---
function listCheckpoints() {
    // 그래프는 전체 상태(finished/running/queued/failed)를 계보로 보여준다.
    // 파인튜닝 베이스 선택은 onGraphNodeClick에서 finished 만 허용.
    return api.get('/checkpoints', {
        params: {
            where: `task_id,=,${selectedWorkspaceId.value}`,
            order: 'created_at DESC'
        }
    }).then(response => {
        checkpoints.value = (response.data?.checkpoints || []).map(c => {
            const policy = policies.value.find(p => p.id === c.policy_id);
            return {...c, policy: policy || c.policy};
        })
    }).catch((error) => {
        console.error('Error fetching checkpoints:', error);
    });
}

function listPolicies() {
    return api.get('/policies').then(response => {
        policies.value = (response.data?.policies || []).filter(policy => policy.is_vla !== 1);
    }).catch((error) => {
        console.error('Error fetching policies:', error);
    });
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
const watchingDetailLoading = ref(false);
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
        dataset_info: Object.fromEntries(selectedDatasetIds.value.map(d => [d, {
            episode_num: availableDatasets.value.find(ds => ds.id === d).episodes.length,
            weight: getDatasetWeight(d),
        }])),
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

async function watchCheckpoint(checkpoint) {
    // 큐 스냅샷은 경량(id/name/status)만 담는다. 다이얼로그를 즉시 열고(경량 정보로
    // 헤더·상태 표시), 상세는 백그라운드로 조회해 채운다 — 조회 중엔 다이얼로그
    // 내부에 스피너가 뜬다(detailLoading).
    watchingCheckpoint.value = checkpoint;
    showTrainingDialog.value = true;
    watchingDetailLoading.value = true;
    try {
        const res = await api.get(`/checkpoint/${checkpoint.id}`);
        if (res?.data?.checkpoint) watchingCheckpoint.value = res.data.checkpoint;
    } catch {
        // 조회 실패 — 경량 정보로 유지
    } finally {
        watchingDetailLoading.value = false;
    }
}

// 큐 패널의 (경량) snapshot이 갱신될 때, 다이얼로그가 열린 체크포인트의 status가
// 바뀌었으면(queued→running→finished 등) 상세를 다시 조회해 다이얼로그를 최신
// full dict로 갱신한다. status가 그대로면 재조회하지 않는다(불필요한 요청 방지).
watch(
    () => queuePanel.value?.queue,
    async (queue) => {
        if (!queue || !watchingCheckpoint.value) return;
        const id = watchingCheckpoint.value.id;
        const curStatus = watchingCheckpoint.value.status;
        const light =
            (queue.running_list || []).find((c) => c.id === id) ||
            (queue.queued || []).find((c) => c.id === id) ||
            (queue.recent || []).find((c) => c.id === id);
        const liveStatus = light?.status;
        if (liveStatus) {
            if (liveStatus === curStatus) return; // 변화 없음
        } else {
            // 큐에서 사라짐 — 이미 종료 상태로 알고 있으면 더 조회 안 함.
            if (['finished', 'failed', 'canceled'].includes(curStatus)) return;
        }
        // status 변화/큐 이탈 → 상세 재조회로 다이얼로그 갱신.
        try {
            const res = await api.get(`/checkpoint/${id}`);
            if (res?.data?.checkpoint) watchingCheckpoint.value = res.data.checkpoint;
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
        const saved = originalPolicy.settings[key];
        // 저장된 policy 가 이 키를 안 가짐 = policy 저장 후 추가된 config 필드.
        // 사용자가 바꾼 게 아니라 schema 확장이므로 "변경"으로 치지 않는다
        // (안 그러면 기존 policy 가 전부 Save As 로 떠 Continue 가 안 보임).
        if (saved === undefined) continue;
        const current = policyForm.value.settings[key].value;
        // 배열/객체 값(obs_state_keys 등)은 참조 비교(!==)가 항상 true 라 JSON 비교.
        const changed = (typeof saved === 'object' || typeof current === 'object')
            ? JSON.stringify(saved) !== JSON.stringify(current)
            : saved !== current;
        if (changed) return true;
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