<template>
    <q-page class="q-pt-lg q-pr-lg full-height column">
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <!-- <div class="float-right">
                    <q-toggle
                        class="text-white"
                        label="Failure Detector"
                        :model-value="processStore.isRunning('failure_detection')"
                        @update:model-value="toggleFailureDetection()"
                        dense
                    ></q-toggle>
                    <div class="row q-mt-sm" v-if="processStore.isRunning('failure_detection')">
                        <div class="text-white">Uncertainty Score:</div>
                        <div
                            class="q-ml-sm"
                            :class="isFailureDetected ? 'text-red text-bold' : 'text-green text-bold'"
                        >{{ uncertaintyScore }}</div>
                    </div>
                </div> -->
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-lg">{{ $t('workspaceIntroTitle') }}</div>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="selectedWorkspaceId"
                        :options="workspaces"
                        :label="$t('workspaceSelectLabel')"
                        style="width: 400px"
                        class="q-ml-lg"
                        map-options
                        emit-value
                        option-label="name"
                        option-value="id"
                    >
                        <template v-slot:option="scope">
                            <q-item :props="scope.props" v-if="scope.opt.id === 'new'" v-bind="scope.itemProps" @click="openCreateWorkspaceForm">
                                <q-item-section>
                                    <q-item-label class="text-yellow">{{ scope.opt.name }}</q-item-label>
                                </q-item-section>
                            </q-item>
                            <q-item :props="scope.props" v-bind="scope.itemProps" v-else>
                                <q-item-section>
                                    <q-item-label>{{ scope.opt.name }}</q-item-label>
                                </q-item-section>
                                <q-item-section side>
                                    <div class="row">
                                        <q-btn icon="edit" size="sm" flat round @click.stop="openEditWorkspaceForm(scope.opt)" />
                                        <q-btn icon="delete" size="sm" color="red" flat round @click.stop="deleteWorkspace(scope.opt)" />
                                    </div>
                                </q-item-section>
                            </q-item>   
                        </template>
                    </q-select>
                </div>
                <div class="text-body text-white">{{ $t('workspaceIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('workspaceIntroBody2') }}</div>
            </div>
        </div>

        <TutorialHint class="q-mb-md" :text="$t('tutorialWorkspaceIntro')" />

        <div class="col q-mb-lg border-rounded border-grey text-grey flex-center flex text-h6" v-if="!selectedWorkspaceId">
            {{ $t('selectWorkspaceFirst') }}
        </div>
        <div class="col row q-mb-lg" v-else>
            <div class="col-3 bg-secondary q-mr-lg border-rounded q-pa-sm" v-if="status === 'pending'">
                <q-tabs
                    dense
                    v-model="selectedTab"
                    :breakpoint="0"
                    class="text-white"
                    active-color="primary"
                >
                    <q-tab name="setting" :label="$t('workspaceTabSetting')"></q-tab>
                    <q-tab name="data" :label="$t('workspaceTabData')"></q-tab>
                    <q-tab name="inference" :label="$t('workspaceTabInference')"></q-tab>
                </q-tabs>
                <div v-if="selectedTab === 'setting'" class="q-pt-md q-px-sm text-white">
                    <TutorialHint class="q-mb-sm" :text="$t('tutorialWorkspaceSetting')" />
                    <q-list dark bordered separator class="border-rounded bg-dark" >
                        <q-expansion-item
                            icon="camera"
                            :label="`${$t('sensorSetting')} (${selectedSensors.length})`"
                        >
                            <q-card class="bg-dark">
                                <q-card-section>
                                    <div
                                        class="q-pa-sm q-px-md q-my-sm border-rounded row"
                                        v-for="sensor in selectedSensors" 
                                        :key="sensor.id"
                                        :class="sensor.status === 'on' ? 'bg-green-10' : 'bg-grey-8'"
                                    >
                                        <div>{{ sensor.name }}</div>
                                        <q-space />
                                        <q-toggle
                                            :model-value="sensor.status === 'on'"
                                            dense
                                            color="positive"
                                            @click="toggleSensor(sensor)"
                                            v-if="sensor.type !== 'custom'"
                                        ></q-toggle>
                                        <div v-else>
                                            <div class="text-caption text-grey" v-if="sensor.status === 'off'">{{ $t('topicOff') }}</div>
                                            <div class="text-caption text-positive" v-else-if="sensor.status === 'on'">{{ $t('topicOn') }}</div>
                                        </div>
                                        <q-inner-loading :showing="sensor.status === 'loading'"></q-inner-loading>
                                    </div>
                                    <q-btn outline
                                        class="full-width"
                                        rounded
                                        color="primary"
                                        icon="add"
                                        @click="openSensorForm"
                                    ></q-btn>
                                    <div class="row q-gutter-x-sm q-mt-md q-mb-sm">
                                        <q-input
                                            dense outlined dark bg-color="dark"
                                            :label="$t('workspaceCommonWidth')"
                                            class="col"
                                            v-model.number="commonSensorResolution.width"
                                        ></q-input>

                                        <q-input
                                            dense outlined dark bg-color="dark"
                                            :label="$t('workspaceCommonHeight')"
                                            class="col"
                                            v-model.number="commonSensorResolution.height"
                                        ></q-input>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                        <q-expansion-item
                            icon="adb"
                            :label="`${$t('robotSetting')}`"
                        >
                            <q-card class="bg-dark border-rounded">
                            <q-card-section>
                                <q-select
                                    dense
                                    outlined
                                    dark
                                    bg-color="dark"
                                    v-model="selectedWorkspace.assembly_id"
                                    :options="assemblies"
                                    :label="$t('workspaceRobotAssembly')"
                                    style="width: 100%"
                                    map-options
                                    emit-value
                                    option-label="name"
                                    option-value="id"
                                    @update:model-value="updateWorkspace({ assembly_id: selectedWorkspace.assembly_id })"
                                ></q-select>
                                <div
                                    class="q-pa-sm q-px-md q-my-sm border-rounded row"
                                    v-for="robot in robots" 
                                    :key="robot.id"
                                    :class="robot.status === 'on' ? 'bg-green-10' : (robot.status === 'error' ? 'bg-red-10' : 'bg-grey-8')"
                                >
                                    <div>{{ robot.name }}</div>
                                    <q-space />
                                    <q-toggle
                                        :model-value="robot.status === 'on'"
                                        dense
                                        color="positive"
                                        @click="toggleRobot(robot)"
                                        v-if="robot.type !== 'custom'"
                                    ></q-toggle>
                                    <div v-else>
                                        <div class="text-caption text-grey" v-if="robot.status === 'off'">{{ $t('topicOff') }}</div>
                                        <div class="text-caption text-positive" v-else-if="robot.status === 'on'">{{ $t('topicOn') }}</div>
                                        <div class="text-caption text-negative" v-else-if="robot.status === 'error'">{{ $t('statusError') }}</div>
                                    </div>
                                    <q-inner-loading :showing="robot.status === 'loading'"></q-inner-loading>
                                </div>
                            </q-card-section>
                            </q-card>
                        </q-expansion-item>
                        <q-expansion-item
                            icon="cleaning_services"
                            :label="`${$t('taskSetting')}`"
                        >
                            <q-card class="bg-dark border-rounded">
                                <q-card-section>
                                    <div class="row q-gutter-x-sm">
                                        <q-input
                                            dense
                                            outlined
                                            dark
                                            bg-color="dark"
                                            :label="$t('workspaceEpisodeLength')"
                                            class="col"
                                            v-model.number="selectedWorkspace.episode_len"
                                            @change="updateWorkspace({ episode_len: selectedWorkspace.episode_len })"
                                        ></q-input>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
                    <div v-if="focused.id" class="q-mt-md text-white h6">
                        <div>{{ $t(`${focused.device_type} Config`) }} <span class="text-primary">{{ focused.name }}</span></div>
                        <TutorialHint class="q-mt-sm" :text="$t('tutorialWorkspaceSensorConfig')" v-if="focused.device_type === 'sensor'" />
                        <TutorialHint class="q-mt-sm" :text="$t('tutorialWorkspaceRobotConfig')" v-else-if="focused.device_type === 'robot'" />
                        <div
                            class="q-pa-sm q-px-md q-mt-sm border-rounded row border-white"
                            v-if="focused.device_type === 'sensor'"
                        >
                            <div class="text-caption col-12">
                                {{ $t('workspaceCropHint') }}
                                <q-btn size="xs" outline color="pink-3" icon="sync" @click="resetCroppedArea()" class="q-ml-sm" />
                                <!-- <q-btn size="xs" outline color="primary" icon="save" @click="saveCroppedArea" class="q-ml-sm" /> -->
                            </div>
                            <div
                                class="relative-position"
                                style="min-width: 200px; min-height: 200px; border-color: blue;"
                                @mousedown="startCrop"
                                @mousemove="doCrop"
                                @mouseup="endCrop"
                                @mouseleave="cancelCrop"
                                ref="videoContainer"
                            >
                                <web-rtc-video
                                    :process-id="`sensor_${focused.id}`"
                                    :topic="focused.read_topic"
                                    :msg-type="focused.read_topic_msg"
                                    class="border-rounded"
                                    :key="focused.id"
                                    :loading="focused.status !== 'on'"
                                    v-if="focused.status !== 'off'"
                                    show_original_video
                                ></web-rtc-video>
                                <div
                                    class="crop-area"
                                    :style="cropAreaStyle"
                                    v-if="cropAreaStyle.width"
                                ></div>
                            </div>
                            <div>
                                <div class="row justify-between">
                                    <div class="text-caption">{{ $t('workspaceCroppedAreaLabel') }}</div>
                                    <div>
                                        {{ selectedWorkspace.sensor_cropped_area[focused.id] ? `${selectedWorkspace.sensor_cropped_area[focused.id][2] - selectedWorkspace.sensor_cropped_area[focused.id][0]} x ${selectedWorkspace.sensor_cropped_area[focused.id][3] - selectedWorkspace.sensor_cropped_area[focused.id][1]}` : $t('workspaceCroppedAreaNotSet') }}
                                    </div>
                                </div>
                                <div class="row q-gutter-x-sm">
                                    <q-input
                                        v-for="(val, index) in ['x1', 'y1', 'x2', 'y2']"
                                        :key="index"
                                        dense
                                        outlined
                                        dark
                                        bg-color="dark"
                                        class="col"
                                        v-model.number="selectedWorkspace.sensor_cropped_area[focused.id][index]"
                                        @update:model-value="updateWorkspaceDeviceSetting({ 
                                            device_type: 'sensors',
                                            key: 'cropped_area',
                                            setting: selectedWorkspace.sensor_cropped_area,
                                        });"
                                    ></q-input>
                                </div>
                                
                            </div>
                            <div class="q-mt-sm">
                                <div class="text-caption q-mb-xs">{{ $t('workspaceRotateLabel') }}</div>
                                <q-select
                                    dense
                                    outlined
                                    dark
                                    bg-color="dark"
                                    v-model="selectedWorkspace.sensor_rotate[focused.id]"
                                    :options="[0, 90, 180, 270]"
                                    :label="$t('workspaceRotationDegrees')"
                                    @update:model-value="updateWorkspaceDeviceSetting({
                                        device_type: 'sensors',
                                        key: 'rotate',
                                        setting: selectedWorkspace.sensor_rotate,
                                    });"
                                ></q-select>
                            </div>
                            <div class="q-mt-md q-pa-sm border-rounded bg-grey-10">
                                <div class="row items-center q-mb-xs">
                                    <q-icon name="auto_fix_high" class="q-mr-xs" />
                                    <div class="text-caption text-bold">{{ $t('workspaceSam3Title') }}</div>
                                    <q-space />
                                    <q-toggle
                                        :model-value="sam3Cfg.enabled"
                                        dense
                                        color="positive"
                                        @update:model-value="(v) => updateSam3({ enabled: v })"
                                    />
                                </div>
                                <div v-if="sam3Cfg.enabled">
                                    <div class="text-caption q-mb-xs">{{ $t('workspaceSam3Mode') }}</div>
                                    <q-select
                                        dense outlined dark bg-color="dark"
                                        :model-value="sam3Cfg.mode"
                                        :options="[
                                            { label: $t('workspaceSam3ModeBackground'), value: 'background' },
                                            { label: $t('workspaceSam3ModeObject'), value: 'object' },
                                        ]"
                                        emit-value map-options
                                        @update:model-value="(v) => updateSam3({ mode: v })"
                                    />
                                    <div class="row items-center q-mt-sm q-gutter-x-sm">
                                        <div class="text-caption">{{ $t('workspaceSam3Color') }}</div>
                                        <div
                                            class="border-rounded"
                                            :style="`width: 28px; height: 20px; background: rgb(${sam3Cfg.color.join(',')}); border: 1px solid #888;`"
                                        ></div>
                                        <q-btn dense flat size="xs" icon="colorize">
                                            <q-popup-proxy>
                                                <q-color
                                                    :model-value="sam3ColorHex"
                                                    @change="(v) => updateSam3({ color: hexToRgb(v) })"
                                                    no-header no-footer default-view="palette"
                                                />
                                            </q-popup-proxy>
                                        </q-btn>
                                    </div>
                                    <div class="text-caption q-mt-sm q-mb-xs">{{ $t('workspaceSam3TextPrompts') }}</div>
                                    <div
                                        v-for="(prompt, idx) in sam3Cfg.text_prompts"
                                        :key="`p-${idx}`"
                                        class="row items-center q-mb-xs q-gutter-x-xs"
                                    >
                                        <q-input
                                            dense outlined dark bg-color="dark"
                                            class="col"
                                            :model-value="prompt"
                                            :placeholder="$t('workspaceSam3TextPromptPlaceholder')"
                                            @update:model-value="(v) => updateSam3TextPrompt(idx, v)"
                                        />
                                        <q-btn dense flat size="sm" icon="close" color="negative" @click="removeSam3TextPrompt(idx)" />
                                    </div>
                                    <q-btn
                                        outline rounded dense size="sm"
                                        class="full-width q-mb-sm"
                                        color="primary"
                                        icon="add"
                                        :label="$t('workspaceSam3AddText')"
                                        @click="addSam3TextPrompt()"
                                    />
                                    <div class="text-caption q-mb-xs">{{ $t('workspaceSam3Boxes') }} ({{ sam3Cfg.boxes.length }})</div>
                                    <div
                                        v-for="(box, idx) in sam3Cfg.boxes"
                                        :key="`b-${idx}`"
                                        class="row items-center q-mb-xs"
                                    >
                                        <div class="text-caption col">[{{ box.join(', ') }}]</div>
                                        <q-btn dense flat size="sm" icon="close" color="negative" @click="removeSam3Box(idx)" />
                                    </div>
                                    <q-btn
                                        outline rounded dense size="sm"
                                        class="full-width q-mb-sm"
                                        :color="sam3DrawingBox ? 'orange' : 'primary'"
                                        :icon="sam3DrawingBox ? 'cancel' : 'crop_free'"
                                        :label="sam3DrawingBox ? $t('workspaceSam3CancelDraw') : $t('workspaceSam3DrawBox')"
                                        @click="toggleSam3DrawBox()"
                                    />
                                    <q-btn
                                        outline rounded dense size="sm"
                                        class="full-width"
                                        color="positive"
                                        icon="play_arrow"
                                        :loading="sam3PreviewLoading"
                                        :label="$t('workspaceSam3TestPreview')"
                                        @click="testSam3Preview()"
                                    />
                                    <div v-if="sam3PreviewImg" class="q-mt-sm">
                                        <q-img :src="sam3PreviewImg" />
                                    </div>
                                </div>
                            </div>
                        </div>
                        <div
                            class="q-pa-sm q-px-md q-mt-sm border-rounded border-white"
                            v-else-if="focused.device_type === 'robot'"
                        >
                            <div class="text-caption q-mb-xs row q-gutter-x-sm">
                                <div>{{ $t('workspaceHomePoseLabel') }}</div>
                                <q-space></q-space>
                                <q-btn size="xs" outline color="pink-3" icon="sync" @click="selectedWorkspace.home_pose[focused.id] = [...focused.jointState]" />
                                <q-btn size="xs" outline color="primary" icon="save" @click="updateWorkspaceDeviceSetting({
                                    device_type: 'robots',
                                    key: 'home_pose',
                                    setting: selectedWorkspace.home_pose,
                                })" />
                            </div>
                            <div class="row q-col-gutter-sm">
                                <q-input
                                    dense
                                    outlined
                                    dark
                                    bg-color="dark"
                                    :label="val"
                                    class="col-6"
                                    v-model.number="selectedWorkspace.home_pose[focused.id][index]"
                                    v-for="(val, index) in focused.joint_names"
                                    :key="index"
                                ></q-input>
                            </div>
                        </div>
                    </div>
                </div>
                <div
                    v-else-if="selectedTab === 'data'"
                    class="q-pt-md q-px-sm text-white"
                    style="height: 90%"
                >
                    <TutorialHint class="q-mb-sm" :text="$t('tutorialWorkspaceData')" />
                    <q-btn
                        outline
                        class="full-width q-mb-sm"
                        rounded
                        color="primary bg-dark"
                        icon="add"
                        :label="$t('workspaceAddDatasetFolder')"
                        @click="openAddDatasetForm"
                    ></q-btn>
                    <q-scroll-area class="full-height">
                        <q-list bordered separator class="border-rounded bg-dark" dark>
                            <q-expansion-item
                                expand-separator
                                icon="folder"
                                :label="`${dataset.name} (${dataset.id})`"
                                :caption="`${dataset.episodes.length} episodes`"
                                v-for="dataset in datasets"
                                :key="dataset.id"
                                dark
                                :model-value="selectedDatasetId === dataset.id"
                                @show="selectedDatasetId = dataset.id"
                                header-class="text-white"
                            >
                                <template v-slot:header>
                                    <q-icon name="folder" class="q-mr-lg" size="lg"></q-icon>
                                    <div class="col">
                                        <div>{{ dataset.name }} ({{ dataset.id }})</div>
                                        <div class="text-caption">{{ dataset.episodes.length }} episodes</div>
                                         <q-menu context-menu>
                                            <q-list bordered separator>
                                                <q-item clickable v-ripple v-close-popup @click="openEditDatasetForm(dataset)">
                                                    <q-item-section>{{ $t('workspaceDatasetEdit') }}</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon name="edit" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                                <q-item clickable v-ripple v-close-popup
                                                    @click="openAugmentationForm(dataset)">
                                                    <q-item-section>{{ $t('workspaceDatasetAugment') }}</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon name="edit" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                                <q-item clickable v-ripple v-close-popup @click="openMergeDatasetForm(dataset)">
                                                    <q-item-section>{{ $t('workspaceDatasetMerge') }}</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon name="merge_type" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                                <q-item clickable v-ripple class="text-negative" @click="deleteDataset(dataset)">
                                                    <q-item-section>{{ $t('workspaceDatasetDelete') }}</q-item-section>
                                                    <q-item-section side>
                                                        <q-icon color="negative" name="delete" size="xs" />
                                                    </q-item-section>
                                                </q-item>
                                            </q-list>
                                        </q-menu>
                                    </div>
                                    <q-item-section side v-if="!checkDatasetCompatibility(dataset)">
                                        <q-icon name="warning" color="negative" size="24px" />
                                    </q-item-section>
                                    <q-space class="col"></q-space>
                                </template>
                                <q-list bordered separator dark dense v-if="selectedDatasetId === dataset.id">
                                    <q-item
                                        v-for="episode in dataset.episodes"
                                        :key="episode.name"
                                        clickable
                                        v-ripple
                                        @click="selectEpisode(episode)"
                                        :class="{ 'bg-primary': selectedEpisode.name === episode.name && selectedDataset && selectedDataset.id === dataset.id }"
                                    >
                                        <q-item-section>
                                            <q-item-label class="q-ml-xl">{{ episode.name }}</q-item-label>
                                        </q-item-section>
                                        <q-item-section side>
                                            <q-btn
                                                icon="close"
                                                size="sm"
                                                flat
                                                round
                                                @click.stop="deleteFile(dataset.id, episode)"
                                            />
                                        </q-item-section>
                                    </q-item>
                                </q-list>
                            </q-expansion-item>
                        </q-list>
                    </q-scroll-area>
                </div>
                <div v-else-if="selectedTab === 'inference'" style="height: 90%" class="q-pt-md q-px-sm text-white">
                    <TutorialHint class="q-mb-sm" :text="$t('tutorialWorkspaceInference')" />
                    <q-scroll-area class="full-height">
                        <div class="row q-col-gutter-md">
                        <div class="col-6" v-for="checkpoint in checkpoints" :key="checkpoint.id">
                            <q-card
                                class="q-pa-md bg-dark border-rounded border-white text-white"
                                :class="selectedCheckpointId === checkpoint.id ? 'border-primary' : ''"
                                @click="selectedCheckpointId = checkpoint.id"
                            >
                                <q-menu context-menu>
                                    <q-list bordered separator>
                                        <q-item
                                            clickable
                                            v-ripple
                                            v-close-popup
                                            @click="openCheckpointInfoDialog(checkpoint)"
                                        >
                                            <q-item-section>{{ $t('workspaceCheckpointShow') }}</q-item-section>
                                            <q-item-section side>
                                                <q-icon name="add" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="openCheckpointForm(checkpoint)">
                                            <q-item-section>{{ $t('workspaceCheckpointEdit') }}</q-item-section>
                                            <q-item-section side>
                                                <q-icon name="edit" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="generateOodFeatures(checkpoint)">
                                            <q-item-section>{{ $t('workspaceCheckpointOod') }}</q-item-section>
                                            <q-item-section side>
                                                <q-icon name="analytics" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple v-close-popup @click="exportCheckpoint(checkpoint)">
                                            <q-item-section>{{ $t('workspaceCheckpointExport') }}</q-item-section>
                                            <q-item-section side>
                                                <q-icon name="download" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                        <q-item clickable v-ripple class="text-negative" @click="deleteCheckpoint(checkpoint)">
                                            <q-item-section>{{ $t('workspaceCheckpointDelete') }}</q-item-section>
                                            <q-item-section side>
                                                <q-icon color="negative" name="delete" size="xs" />
                                            </q-item-section>
                                        </q-item>
                                    </q-list>
                                </q-menu>
                                <q-card-section class="q-pa-none q-mt-sm text-center">
                                    <q-img
                                        src="images/brain-icon.png"
                                        class="cursor-pointer"
                                        fit="contain"
                                        width="80px"
                                    >
                                    </q-img>
                                    <div class="text-bold q-mt-md">{{ checkpoint.name }} ({{ checkpoint.id }})</div>
                                </q-card-section>
                            </q-card>
                        </div>
                    </div>
                    </q-scroll-area>
                </div>
            </div>
            <div class="col column">
                <monitoring-window
                    class="col"
                    :workspace="selectedWorkspace"
                    :robots="robots"
                    :sensors="selectedSensors"
                    v-model:selected-dataset-id="selectedDatasetId"
                    v-model:selected-checkpoint-id="selectedCheckpointId"
                    v-model:focused="focused"
                    v-model:selected-episode="selectedEpisode"
                    :datasets="datasets"
                    :checkpoints="checkpoints"
                    :status="status"
                    :class="isFailureDetected ? 'border-red' : 'border-white'"
                />
            </div>
        </div>
        <form-dialog
            v-model="showWorkspaceForm"
            :title="$t(workspaceForm.find((e) => e.key === 'id').value ? 'workerkspaceEditFormTitle' : 'workspaceCreateFormTitle')"
            :form="workspaceForm"
            :ok-button-label="$t(workspaceForm.find((e) => e.key === 'id').value ? 'save' : 'create')"
            :cancel-button-label="$t('cancel')"
            @submit="saveWorkspace"
        ></form-dialog>
        <form-dialog
            v-model="showSensorForm"
            :title="$t('workspaceSensorFormTitle')"
            :form="sensorForm"
            @submit="saveSensorSettings"
            :ok-button-label="$t('save')"
        ></form-dialog>
        <form-dialog
            v-model="showDatasetForm"
            :title="$t(datasetForm.find(f => f.key === 'id')? 'datasetEditFormTitle' : 'datasetAddFormTitle')"
            :form="datasetForm"
            @submit="saveDataset"
            :ok-button-label="$t(datasetForm.find(f => f.key === 'id') ? 'save' : 'add')"
        >
            <template v-slot:dataset_metadata>
                <div style="color: red;">{{ $t('workspaceDatasetIncompatible') }}</div>
                <div v-for="robot in datasetForm.find(f => f.key === 'dataset_metadata').value.robots" :key="robot">
                    <div class="row q-my-md q-col-gutter-sm">
                        <div class="text-white">{{ robot }}</div>
                        <q-select
                            dense
                            outlined
                            dark
                            bg-color="dark"
                            v-model="datasetForm.find(f => f.key === 'robot_mappings').value[robot]"
                            :options="robots"
                            :label="$t('workspaceSelectRobot')"
                            style="width: 200px"
                            map-options
                            emit-value
                            option-label="name"
                            option-value="id"
                        ></q-select>
                    </div>
                </div>
                <div v-for="sensor in datasetForm.find(f => f.key === 'dataset_metadata').value.sensors" :key="sensor">
                    <div class="row q-my-md q-col-gutter-sm">
                        <div class="text-white">{{ sensor }}</div>
                        <q-select
                            dense
                            outlined
                            dark
                            bg-color="dark"
                            v-model="datasetForm.find(f => f.key === 'sensor_mappings').value[sensor]"
                            :options="selectedSensors"
                            :label="$t('workspaceSelectSensor')"
                            style="width: 200px"
                            map-options
                            emit-value
                            option-label="name"
                            option-value="id"
                        ></q-select>
                    </div>
                </div>
            </template>
        </form-dialog>
        <form-dialog
            v-model="showCheckpointForm"
            :title="$t('checkpointEditFormTitle')"
            :form="checkpointForm"
            @submit="saveCheckpoint"
            :ok-button-label="$t('save')"
        ></form-dialog>
        <form-dialog
            v-model="showMergeDatasetForm"
            :title="$t('mergeDatasetFormTitle')"
            :form="mergeDatasetForm"
            @submit="mergeDatasets"
            :ok-button-label="$t('save')"
        ></form-dialog>

        <q-dialog
            v-model="showAugmentationForm"
            persistent
            full-width
        >
            <data-augmentation-dialog
                :dataset="augmentingDataset"
                :task-id="selectedWorkspaceId"
                v-if="selectedWorkspaceId"
            />
        </q-dialog>

        <q-dialog v-model="showCheckpointInfo" full-width>
            <q-card class="bg-secondary border-rounded border-white" dark>
                <q-card-section class="bg-dark text-white row">
                    <div class="text-h6">{{ $t('checkpointInfo') }}</div>
                    <q-space></q-space>
                    <q-btn dense color="white" round icon="close" text-color="dark" @click="showCheckpointInfo = false"/>
                </q-card-section>

                <q-separator />
                <q-card-section class="bg-secondary q-px-xl q-py-lg">
                    <checkpoint-info
                        :checkpoint="selectedCheckpoint"
                        :height="400"
                    />
                </q-card-section>
                <q-card-section class="bg-secondary q-px-xl q-pt-none" align="right">
                    <div>{{ $t('checkpointLossEpoch', { loss: selectedCheckpoint.loss.toFixed(4), epoch: selectedCheckpoint.best_epoch }) }}</div>
                </q-card-section>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import FormDialog from 'src/components/v2/FormDialog.vue';
import { useSensor } from '../../composables/useSensor';
import { useRobot } from 'src/composables/useRobot';
import { Notify, Loading } from 'quasar';
import { useI18n } from 'vue-i18n';
import DataAugmentationDialog from 'src/components/v2/DataAugmentationDialog.vue';
import { useSocket } from 'src/composables/useSocket.js';
import { useProcessStore } from 'src/stores/processStore';
import MonitoringWindow from 'src/components/v2/MonitoringWindow.vue';
import CheckpointInfo from 'src/components/v2/CheckpointInfo.vue';
import WebRtcVideo from 'src/components/v2/WebRtcVideo.vue';
import TutorialHint from 'src/components/v2/TutorialHint.vue';

const processStore = useProcessStore();

const { socket } = useSocket();

const { t } = useI18n()

const workspaces = ref([]);
const selectedWorkspaceId = ref(null);

const showWorkspaceForm = ref(false);
const workspaceForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('workspaceName'), type: 'text', value: '', default: '' },
]);

function openCreateWorkspaceForm() {
    selectedWorkspaceId.value = null;
    workspaceForm.value.forEach((field) => {
        field.value = field.default;
    });
    showWorkspaceForm.value = true;
}

function openEditWorkspaceForm(workspace) {
    workspaceForm.value.forEach((field) => {
        field.value = workspace[field.key] || field.default;
    });
    workspaceForm.value.find((e) => e.key === 'id').value = workspace.id; // Set ID for edit
    showWorkspaceForm.value = true;
}

function saveWorkspace (form) {
    if (workspaceForm.value.find((e) => e.key === 'id').value) {
        return api.put(`/task/${form.id}`, form).then(() => {
            listWorkspaces().then(() => {
                selectedWorkspaceId.value = workspaces.value.find(w => w.name === form.name).id;
            });
        });
    }
    return api.post('/task', form).then(() => {
        listWorkspaces().then(() => {
            selectedWorkspaceId.value = workspaces.value.find(w => w.name === form.name).id;
        });
    });
};

function listWorkspaces() {
    return api.get('/tasks').then((response) => {
        workspaces.value = response.data.tasks;
        workspaces.value.push({ id: 'new', name: t('workspaceSelectCreateNew') });
    });
}

const selectedWorkspace = computed(() => {
    return workspaces.value.find(w => w.id === selectedWorkspaceId.value) || {};
});

console.log('Selected Workspace:', selectedWorkspace);

const robots = computed(() => {
    if (!selectedWorkspace.value) {
        return [];
    }
    if (!selectedWorkspace.value.assembly) {
        return [];
    }
    return selectedWorkspace.value.assembly.robots.map((robot) => {
        const handler = useRobot(robot);
        robot.handler = handler;
        // Topic visibility는 topicStore가 push로 추적 — type별 분기 불필요.
        return robot;
    })
});

const sensors = ref([]);

const selectedSensors = computed(() => {
    if (!selectedWorkspace.value) {
        return [];
    }
    return selectedWorkspace.value.sensors.map((sensor) => {
        const handler = useSensor(sensor);
        sensor.handler = handler;
        // Topic visibility는 topicStore가 push로 추적 — type별 분기 불필요.
        return sensor;
    });
});

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

const assemblies = ref([]);
function listAssemblies() {
    return api.get('/assemblies').then((response) => {
        assemblies.value = response.data.assemblies || [];
    }).catch((error) => {
        console.error('Error fetching assemblies:', error);
    });
}

const selectedTab = ref('setting');

function toggleSensor(sensor) {
    sensor.process_id = `sensor_${sensor.id}`;
    if (sensor.status === 'on') {
        sensor.handler.stopSensor()
    } else {
        sensor.handler.startSensor()
    }
}

const showSensorForm = ref(false);
const sensorForm = ref([
    { key: 'id', value: null },
    { key: 'sensor_ids', label: t('datasetSensorsLabel'), type: 'multiselect_list', options: computed(() => sensors.value.map(s => ({ label: s.name, value: s.id }))), value: [] }
])

const sensorSettingsMap = ref({});
watch(() => selectedWorkspace.value?.settings, (newSettings) => {
    sensorSettingsMap.value = JSON.parse(JSON.stringify(newSettings || {}));
}, { deep: true, immediate: true });

const commonSensorResolution = ref({});

function initCommonSensorResolution() {
    console.log('Initializing commonSensorResolution...');
    commonSensorResolution.value = (() => {
        const firstSensorId = selectedWorkspace.value.sensor_ids[0];
        if (firstSensorId) {
            const settings = selectedWorkspace.value.sensor_img_size[firstSensorId];
            if (settings && settings.length === 2) {
                return {
                    width: settings[0],
                    height: settings[1]
                };
            }
        }
        return { width: 640, height: 480 }; // Default resolution
    })();
}

watch(commonSensorResolution, (newRes) => {
    updateAllSensorResolutions(newRes.width, newRes.height);
}, { deep: true });

function updateAllSensorResolutions(width, height) {
    for(const sensorId of selectedWorkspace.value.sensor_ids) {
        selectedWorkspace.value.sensor_img_size[sensorId] = [width, height];
    }

    updateWorkspaceDeviceSetting({
        device_type: 'sensors',
        key: 'img_size',
        setting: selectedWorkspace.value.sensor_img_size
    })
}


function openSensorForm() {
    if (selectedWorkspace.value) {
        sensorForm.value.find(e => e.key === 'sensor_ids').value =
            (selectedWorkspace.value.sensors || []).map(s => s.id).filter(id => id != null);
    }
    showSensorForm.value = true;
}

function saveSensorSettings(form) {
    console.log('Sensor IDs to update:', form.sensor_ids);
    return updateWorkspace({ sensor_ids: form.sensor_ids });
}

function updateWorkspace(form) {
    console.log('Updating workspace with form:', form);
    return api.put(`/task/${selectedWorkspace.value.id}`, form).then(() => {
        listWorkspaces();
    });
}

function updateWorkspaceDeviceSetting(form) {
    console.log('Updating workspace device settings with form:', form);
    return api.put(`/task/${selectedWorkspace.value.id}/device_settings`, form).then(() => {
        listWorkspaces();
    });
}

const focused = ref({});
watch(selectedWorkspaceId, (newVal) => {
    if (!newVal) {
        return;
    }
    focused.value = {};
    listDatasets();
    listCheckpoints();
    initCommonSensorResolution();
});

function toggleRobot(robot) {
    robot.process_id = `robot_${robot.id}`;
    const startFlow = () => robot.handler.startRobot();
    if (robot.status === 'on') {
        robot.handler.stopRobot();
    } else if (robot.status === 'error') {
        robot.handler.stopRobot().finally(() => startFlow());
    } else {
        startFlow();
    }
}

const datasets = ref([]);
function listDatasets() {
    return api.get('/datasets', {
        params: {
            task_id: selectedWorkspaceId.value,
        }
    }).then((res) => {
        datasets.value = res.data.datasets || [];
    }).catch((error) => {
        console.error('Error fetching datasets:', error);
    });
}

function deleteFile(datasetId, file) {
    api.delete(`/dataset/${datasetId}/${file.name}`).then(() => {
        datasets.value = datasets.value.map((d) => {
            if (d.id === datasetId) {
                return {
                    ...d,
                    episodes: d.episodes.filter((e) => e.name !== file.name),
                };
            }
            return d;
        });
    })
}

const selectedEpisode = ref({});
function selectEpisode(episode) {
    selectedEpisode.value = episode;
}
function deselectEpisode() {
    selectedEpisode.value = {};
}   

function deleteDataset(dataset) {
    Notify.create({
        message: t('workspaceConfirmDeleteDataset', { name: dataset.name }),
        color: 'negative',
        actions: [
            { label: t('cancel'), color: 'white', handler: () => { /* do nothing */ } },
            { label: t('delete'), color: 'white', handler: () => {
                api.delete(`/dataset/${dataset.id}`).then(() => {
                    listDatasets();
                }).catch((error) => {
                    console.error('Error deleting dataset:', error);
                    Notify.create({
                        color: 'negative',
                        message: t('workspaceErrorDeletingDataset')
                    });
                });
            }}
        ]
    });
}

const showDatasetForm = ref(false);
const datasetForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('datasetName'), type: 'text', value: '', default: '' },
    { key: 'dataset_metadata', label: t('datasetMetadataLabel'), type: 'custom', value: {}, default: {}, show: (form) => !checkDatasetCompatibility(form)},
    { key: 'robot_mappings', label: t('datasetRobotMappingsLabel'), type: 'custom', value: [], default: {}, show: () => false },
    { key: 'sensor_mappings', label: t('datasetSensorMappingsLabel'), type: 'custom', value: {}, default: {}, show: () => false },
])

// const datasetMetadataForm = ref([
//     { key: 'robot_mappings', label: 'Robot Mappings', type: 'custom', value: [], default: {} },
//     { key: 'sensor_mappings', label: 'Sensor Mappings', type: 'custom', value: {}, default: {} },
// ]);

function openAddDatasetForm() {
    datasetForm.value.forEach((field) => {
        field.value = field.default;
    });
    showDatasetForm.value = true;
}

async function openEditDatasetForm(dataset) {
    // 1. 기본 필드 값 세팅 (id, name 등)
    datasetForm.value.forEach((field) => {
        field.value = dataset[field.key] || field.default;
    });

    // 2. robot_mappings 필드 커스텀 세팅

    console.log('Dataset Metadata:', datasetForm.value.find((e) => e.key === 'robot_mappings'));
    
    const robotMappingField = datasetForm.value.find((e) => e.key === 'robot_mappings');
    if (robotMappingField && dataset.dataset_metadata?.robots) {
        // robots 리스트: ['robot_a', 'robot_b']
        // 변환 결과: { "robot_a": null, "robot_b": null }
        robotMappingField.value = Object.fromEntries(
            dataset.dataset_metadata.robots.map(robot => [robot, null])
        );
    }
    console.log('Robot Mappings set to:', robotMappingField.value);

    const sensorMappingField = datasetForm.value.find((e) => e.key === 'sensor_mappings');
    if (sensorMappingField && dataset.dataset_metadata?.sensors) {
        sensorMappingField.value = Object.fromEntries(
            dataset.dataset_metadata.sensors.map(sensor => [sensor, null])
        );
    }

    showDatasetForm.value = true;
}

async function saveDataset(form) {
    try {
        // 1. 현재 datasetForm(배열)의 모든 value를 하나의 객체로 추출
        const payload = {};
        datasetForm.value.forEach(field => {
            payload[field.key] = field.value;
        });

        // 만약 인자로 들어온 form에 추가 데이터가 있다면 합쳐줍니다.
        const finalData = { ...form, ...payload };

        const id = datasetForm.value.find((e) => e.key === 'id').value;

        if (id) {
            // --- 수정(Edit) 로직 ---
            await api.put(`/dataset/${id}`, finalData);
            
            // 메타데이터 수정 API 호출
            const robotMappings = finalData.robot_mappings || {};
            const sensorMappings = finalData.sensor_mappings || {};
            const hasRobotMappings = Object.values(robotMappings).some(v => v !== null);
            const hasSensorMappings = Object.values(sensorMappings).some(v => v !== null);
            if (hasRobotMappings || hasSensorMappings) {
                await api.post(`/dataset/${id}/:edit_datasets_metadata`, {
                    robot_mappings: robotMappings,
                    sensor_mappings: sensorMappings,
                });
            }

        } else {
            // --- 추가(Add) 로직 ---
            await api.post(`/dataset`, { 
                ...finalData, 
                task_id: selectedWorkspaceId.value 
            });
        }

        // 공통 마무리 로직
        datasetForm.value.forEach(field => field.value = field.default); // 폼 초기화
        await listDatasets(); // 목록 갱신
        showDatasetForm.value = false; // 다이얼로그 닫기

    } catch (error) {
        console.error('데이터 저장 실패:', error);
        // 사용자에게 에러 알림 (예: $q.notify 등)
    }
}

function checkDatasetCompatibility(form) {

    const workspaceRobotTypes = robots.value.map(r => "robot_" + r.id).sort();
    const workspaceSensorTypes = selectedSensors.value.map(s => "sensor_" + s.id).sort();

    const datasetRobotTypes = ref([]);
    const datasetSensorTypes = ref([]);

    if (Array.isArray(form)) {
        // 1. 배열 형태인 form에서 dataset_metadata 값을 추출합니다.
        const datasetMetadataEntry = form.find(item => item.key === 'dataset_metadata');
        const dataset_metadata = datasetMetadataEntry ? datasetMetadataEntry.value : { robots: [], sensors: [] };

        // 3. 데이터셋의 구성을 가져옵니다.
        datasetRobotTypes.value = (dataset_metadata.robots || []).filter(t => t !== null).sort();
        datasetSensorTypes.value = (dataset_metadata.sensors || []).filter(t => t !== null).sort();

    } else {
        datasetRobotTypes.value = (form.dataset_metadata?.robots || []).filter(t => t !== null).sort();
        datasetSensorTypes.value = (form.dataset_metadata?.sensors || []).filter(t => t !== null).sort();
    }
    const robotsMatch = JSON.stringify(workspaceRobotTypes) === JSON.stringify(datasetRobotTypes.value);
    const sensorsMatch = JSON.stringify(workspaceSensorTypes) === JSON.stringify(datasetSensorTypes.value);

    if (datasetRobotTypes.value.length == 0 && datasetSensorTypes.value.length == 0) {
        return true;
    }

    return robotsMatch && sensorsMatch;
}

const showAugmentationForm = ref(false);
const augmentingDataset = ref({});
watch(showAugmentationForm, () => {
    deselectEpisode();
});
function openAugmentationForm(dataset) {
    if (!dataset.episodes.length) {
        Notify.create({
            color: 'negative',
            message: t('workspaceAugmentNeedsEpisodes')
        });
        return;
    }
    augmentingDataset.value = dataset;
    showAugmentationForm.value = true;
}

function deleteWorkspace(workspace) {
    Notify.create({
        message: t('workspaceConfirmDeleteWorkspace', { name: workspace.name }),
        color: 'negative',
        actions: [
            { label: t('cancel'), color: 'white', handler: () => { /* do nothing */ } },
            { label: t('delete'), color: 'white', handler: () => {
                return api.delete(`/task/${workspace.id}`).then(() => {
                    listWorkspaces();
                    if (selectedWorkspaceId.value === workspace.id) {
                        selectedWorkspaceId.value = null;
                    }
                });
            }}
        ]
    });
}


const status = computed(() => {
    if (processStore.isRunning('checkpoint_test')) {
        return 'testing';
    } else if (processStore.isRunning('record_episode')) {
        return 'inferencing';
    } else if (processStore.isRunning('replay_episode')) {
        return 'replaying';
    } else {
        listDatasets();
        return 'pending';
    }
});

const selectedDatasetId = ref(null);
const selectedDataset = computed(() => {
    return datasets.value.find(d => d.id === selectedDatasetId.value) || null;
});
const selectedCheckpointId = ref(null);
const selectedCheckpoint = computed(() => {
    return checkpoints.value.find(c => c.id === selectedCheckpointId.value) || null;
});
const checkpoints = ref([]);
function listCheckpoints() {
    return api.get('/checkpoints', {
        params: {
            where: `task_id,=,${selectedWorkspaceId.value}|status,=,finished`,
            order: 'created_at DESC'
        }
    }).then((res) => {
        checkpoints.value = res.data.checkpoints || [];
    });
}

const showCheckpointForm = ref(false);
const checkpointForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('trainCheckpointName'), type: 'text', value: '', default: '' },
]);
function openCheckpointForm(checkpoint) {
    showCheckpointForm.value = true;
    checkpointForm.value.forEach((field) => {
        field.value = checkpoint[field.key] || field.default;
    });
}
function generateOodFeatures(checkpoint) {
    api.post(`/checkpoint/${checkpoint.id}/:generate_ood_features`).then(() => {
        Notify.create({ color: 'positive', message: t('workspaceOodStarted') });
    }).catch((err) => {
        Notify.create({ color: 'negative', message: t('workspaceOodFailed', { error: err }) });
    });
}

async function exportCheckpoint(checkpoint) {
    Loading.show({ message: t('workspaceCheckpointExporting', { name: checkpoint.name }) });
    try {
        const res = await api.post(
            `/checkpoint/${checkpoint.id}/:export`,
            null,
            { responseType: 'blob' }
        );

        // Backend may return JSON error inside a blob; sniff content-type.
        const contentType = res.headers['content-type'] || '';
        if (!contentType.includes('application/zip')) {
            const text = await res.data.text();
            let message = text;
            try {
                message = JSON.parse(text).message || text;
            } catch { /* not JSON */ }
            throw new Error(message);
        }

        // Pull filename from Content-Disposition; fall back if header is missing.
        const cd = res.headers['content-disposition'] || '';
        const match = cd.match(/filename\*?=(?:UTF-8'')?"?([^";]+)"?/);
        const filename = match
            ? decodeURIComponent(match[1])
            : `checkpoint_${checkpoint.id}.zip`;

        const url = URL.createObjectURL(res.data);
        const a = document.createElement('a');
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        a.remove();
        URL.revokeObjectURL(url);

        Notify.create({
            color: 'positive',
            message: t('workspaceCheckpointExported', { filename }),
            timeout: 4000,
        });
    } catch (err) {
        console.error('exportCheckpoint failed:', err);
        Notify.create({
            color: 'negative',
            message: t('workspaceCheckpointExportFailed', { error: err.message || err }),
            timeout: 6000,
        });
    } finally {
        Loading.hide();
    }
}

function deleteCheckpoint(checkpoint) {
    Notify.create({
        message: t('workspaceConfirmDeleteCheckpoint', { name: checkpoint.name }),
        color: 'negative',
        actions: [
            { label: t('cancel'), color: 'white', handler: () => { /* do nothing */ } },
            { label: t('delete'), color: 'white', handler: () => {
                api.delete(`/checkpoint/${checkpoint.id}`).then(() => {
                    listCheckpoints();
                    if (selectedCheckpointId.value === checkpoint.id) {
                        selectedCheckpointId.value = null;
                    }
                });
            }}
        ]
    });
}
function saveCheckpoint(form) {
    return api.put(`/checkpoint/${form.id}`, form).then(() => {
        checkpointForm.value.forEach(field => field.value = field.default); // Reset form fields
        listCheckpoints()
        
    })
}

// function toggleFailureDetection() {
//     const checkpoint = checkpoints.value.at(-1);
//     const isRunning = processStore.isRunning('failure_detection');
//     const action = isRunning ? 'stop' : 'start';
//     const url = `/checkpoint/${checkpoint.id}/:${action}_failure_detection`;

//     api.post(url, {
//         robots: robots.value.filter(r => selectedWorkspace.value.robot_ids.includes(r.id)),
//         sensors: selectedWorkspaceSensors.value,
//         task: selectedWorkspace.value
//     }).then(() => {
//         Notify.create({
//             color: 'positive',
//             message: `Failure detection ${action}ed.`
//         });
//     }).catch((error) => {
//         console.error(`Error ${action}ing failure detection:`, error);
//         Notify.create({
//             color: 'negative',
//             message: `Error ${action}ing failure detection.`
//         });
//     });
// }

const isFailureDetected = ref(false);
const uncertaintyScore = ref(0);

const showCheckpointInfo = ref(false);
const openCheckpointInfoDialog = (checkpoint) => {
    selectedCheckpointId.value = checkpoint.id;
    showCheckpointInfo.value = true;
}; 


const videoContainer = ref(null);
const isCropping = ref(false);
const cropStartPoint = ref({ x: 0, y: 0 });
const cropEndPoint = ref({ x: 0, y: 0 });

const cropAreaStyle = computed(() => {
    if (!isCropping.value) return {};
    const videoEl = videoContainer.value;
    if (!videoEl) return {};

    const left = Math.min(cropStartPoint.value.x, cropEndPoint.value.x);
    const top = Math.min(cropStartPoint.value.y, cropEndPoint.value.y);
    const width = Math.abs(cropStartPoint.value.x - cropEndPoint.value.x);
    const height = Math.abs(cropStartPoint.value.y - cropEndPoint.value.y);

    return {
        left: `${left}px`,
        top: `${top}px`,
        width: `${width}px`,
        height: `${height}px`,
    };
});


function startCrop(event) {
    isCropping.value = true;
    const rect = videoContainer.value.getBoundingClientRect();
    console.log('Video Container Rect:', rect);
    console.log('Mouse Event:', event);
    cropStartPoint.value = {
        x: event.clientX - rect.left,
        y: event.clientY - rect.top,
    };
    cropEndPoint.value = { ...cropStartPoint.value };
}

function doCrop(event) {
    if (isCropping.value) {
        const rect = videoContainer.value.getBoundingClientRect();
        cropEndPoint.value = {
            x: event.clientX - rect.left,
            y: event.clientY - rect.top,
        };
    }
}

function endCrop() {
    isCropping.value = false;
    if (sam3DrawingBox.value) {
        saveSam3Box();
        sam3DrawingBox.value = false;
    } else {
        saveCroppedArea();
    }
}

function cancelCrop() {
    if (isCropping.value) {
        isCropping.value = false;
        cropStartPoint.value = { x: 0, y: 0 };
        cropEndPoint.value = { x: 0, y: 0 };
    }
}

function saveCroppedArea() {
    if (!focused.value.id) return;
    const videoEl = videoContainer.value;
    if (!videoEl) return;
    
    const videoRect = videoEl.querySelector('video').getBoundingClientRect();
    const containerRect = videoEl.getBoundingClientRect();

    console.log(focused.value);

    const scaleX = focused.value.resolution[0] / videoRect.width;
    const scaleY = focused.value.resolution[1] / videoRect.height;

    const x1_rel = Math.min(cropStartPoint.value.x, cropEndPoint.value.x) - (videoRect.left - containerRect.left);
    const y1_rel = Math.min(cropStartPoint.value.y, cropEndPoint.value.y) - (videoRect.top - containerRect.top);
    const x2_rel = Math.max(cropStartPoint.value.x, cropEndPoint.value.x) - (videoRect.left - containerRect.left);
    const y2_rel = Math.max(cropStartPoint.value.y, cropEndPoint.value.y) - (videoRect.top - containerRect.top);
    
    const x1 = Math.round(x1_rel * scaleX);
    const y1 = Math.round(y1_rel * scaleY);
    const x2 = Math.round(x2_rel * scaleX);
    const y2 = Math.round(y2_rel * scaleY);

    const x1_clamped = Math.max(0, Math.min(focused.value.resolution[0], x1));
    const y1_clamped = Math.max(0, Math.min(focused.value.resolution[1], y1));
    const x2_clamped = Math.max(0, Math.min(focused.value.resolution[0], x2));
    const y2_clamped = Math.max(0, Math.min(focused.value.resolution[1], y2));

    selectedWorkspace.value.sensor_cropped_area[focused.value.id] = [x1_clamped, y1_clamped, x2_clamped, y2_clamped];
        
    updateWorkspaceDeviceSetting({ 
        device_type: 'sensors',
        key: 'cropped_area',
        setting: selectedWorkspace.value.sensor_cropped_area
    });
}

function resetCroppedArea() {
    if (!focused.value.id) return;
    selectedWorkspace.value.sensor_cropped_area[focused.value.id] = [
        0,
        0,
        focused.value.resolution[0],
        focused.value.resolution[1]
    ];
    updateWorkspaceDeviceSetting({
        device_type: 'sensors',
        key: 'cropped_area',
        setting: selectedWorkspace.value.sensor_cropped_area
    });
}

const showMergeDatasetForm = ref(false);
const mergeDatasetForm = ref([
    { key: 'source_dataset_id', label: t('mergeDatasetSourceLabel'), type: 'select', options: computed(() => datasets.value.map(d => ({ label: d.name, value: d.id }))), value: null, default: null },
    { key: 'target_dataset_ids', label: t('mergeDatasetTargetsLabel'), type: 'multiselect_list', options: computed(() => datasets.value.map(d => ({ label: d.name, value: d.id }))), value: [], default: [] },
]);

function openMergeDatasetForm(dataset) {
    mergeDatasetForm.value.forEach((field) => {
        field.value = field.default;
    });
    mergeDatasetForm.value.find(e => e.key === 'source_dataset_id').value = dataset.id;
    showMergeDatasetForm.value = true;
}

function mergeDatasets(form) {
    Loading.show();
    return api.post('/dataset/:merge', form).then(() => {
        listDatasets()
    }).finally(() => {
        Loading.hide();
    });
}

onUnmounted(() => {
    robots.value.forEach(robot => {
        if (robot.handler) {
            robot.handler.unSubscribeRobot();
        }
    });
    if (processStore.isRunning('failure_detection') && selectedCheckpointId.value) {
        api.post(`/checkpoint/${selectedCheckpointId.value}/:stop_failure_detection`)
          .catch(err => console.error('Failed to stop failure detection on unmount:', err));
    }
});

onMounted(() => {
    listWorkspaces();
    listSensors();
    // listRobots();
    listAssemblies();
    socket.on('augmentation_complete', (data) => {
        showAugmentationForm.value = false
        listDatasets().then(() => {
            showAugmentationForm.value = false;
            const new_dataset = datasets.value.find(d => d.id === data.dataset_id);
            if (new_dataset) {
                selectedDatasetId.value = new_dataset.id;
            } else {
                console.error(`Could not find the newly augmented dataset with id: ${data.dataset_id}`);
            }
        });
    })

    socket.on('episode_added', (data) => {
        selectedDataset.value?.episodes.push({
            name: data.name,
        });
    });

    socket.on('failure_detection_result', (data) => {
        console.log(data);
        if (data.failure_detected) {
            isFailureDetected.value = true;
        }
        else {
            isFailureDetected.value = false;
        }
        uncertaintyScore.value = Math.round(data.uncertainty_score * 100) / 100;
    });
});

// ── SAM3 segmentation per-sensor config ───────────────────────────────────
const SAM3_DEFAULT = {
    enabled: false,
    text_prompts: [],
    boxes: [],
    mode: 'background',
    color: [0, 0, 0],
};

const sam3DrawingBox = ref(false);
const sam3PreviewLoading = ref(false);
const sam3PreviewImg = ref(null);

const sam3Cfg = computed(() => {
    if (!focused.value?.id || !selectedWorkspace.value) {
        return { ...SAM3_DEFAULT };
    }
    const map = selectedWorkspace.value.sensor_sam3 || {};
    const cur = map[focused.value.id] || {};
    return {
        ...SAM3_DEFAULT,
        ...cur,
        text_prompts: Array.isArray(cur.text_prompts) ? cur.text_prompts : [],
        boxes: Array.isArray(cur.boxes) ? cur.boxes : [],
        color: Array.isArray(cur.color) ? cur.color : [0, 0, 0],
    };
});

const sam3ColorHex = computed(() => {
    const [r, g, b] = sam3Cfg.value.color;
    const h = (n) => Math.max(0, Math.min(255, Math.round(n))).toString(16).padStart(2, '0');
    return `#${h(r)}${h(g)}${h(b)}`;
});

function hexToRgb(hex) {
    const m = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})/i.exec(hex || '');
    if (!m) return [0, 0, 0];
    return [parseInt(m[1], 16), parseInt(m[2], 16), parseInt(m[3], 16)];
}

function persistSam3() {
    if (!selectedWorkspace.value) return;
    updateWorkspaceDeviceSetting({
        device_type: 'sensors',
        key: 'sam3',
        setting: selectedWorkspace.value.sensor_sam3,
    });
}

function updateSam3(patch) {
    if (!focused.value?.id || !selectedWorkspace.value) return;
    if (!selectedWorkspace.value.sensor_sam3) {
        selectedWorkspace.value.sensor_sam3 = {};
    }
    const sid = focused.value.id;
    const cur = { ...sam3Cfg.value, ...selectedWorkspace.value.sensor_sam3[sid] };
    selectedWorkspace.value.sensor_sam3[sid] = { ...cur, ...patch };
    persistSam3();
}

function addSam3TextPrompt() {
    const cur = sam3Cfg.value;
    updateSam3({ text_prompts: [...cur.text_prompts, ''] });
}

function updateSam3TextPrompt(idx, value) {
    const cur = sam3Cfg.value;
    const next = [...cur.text_prompts];
    next[idx] = value;
    updateSam3({ text_prompts: next });
}

function removeSam3TextPrompt(idx) {
    const cur = sam3Cfg.value;
    updateSam3({ text_prompts: cur.text_prompts.filter((_, i) => i !== idx) });
}

function removeSam3Box(idx) {
    const cur = sam3Cfg.value;
    updateSam3({ boxes: cur.boxes.filter((_, i) => i !== idx) });
}

function toggleSam3DrawBox() {
    sam3DrawingBox.value = !sam3DrawingBox.value;
}

function saveSam3Box() {
    if (!focused.value.id) return;
    const videoEl = videoContainer.value;
    if (!videoEl) return;
    const video = videoEl.querySelector('video');
    if (!video) return;
    const videoRect = video.getBoundingClientRect();
    const containerRect = videoEl.getBoundingClientRect();
    const scaleX = focused.value.resolution[0] / videoRect.width;
    const scaleY = focused.value.resolution[1] / videoRect.height;
    const x1_rel = Math.min(cropStartPoint.value.x, cropEndPoint.value.x) - (videoRect.left - containerRect.left);
    const y1_rel = Math.min(cropStartPoint.value.y, cropEndPoint.value.y) - (videoRect.top - containerRect.top);
    const x2_rel = Math.max(cropStartPoint.value.x, cropEndPoint.value.x) - (videoRect.left - containerRect.left);
    const y2_rel = Math.max(cropStartPoint.value.y, cropEndPoint.value.y) - (videoRect.top - containerRect.top);
    const x1 = Math.max(0, Math.round(x1_rel * scaleX));
    const y1 = Math.max(0, Math.round(y1_rel * scaleY));
    const x2 = Math.min(focused.value.resolution[0], Math.round(x2_rel * scaleX));
    const y2 = Math.min(focused.value.resolution[1], Math.round(y2_rel * scaleY));
    if (x2 - x1 < 5 || y2 - y1 < 5) return;
    updateSam3({ boxes: [...sam3Cfg.value.boxes, [x1, y1, x2, y2]] });
}

function testSam3Preview() {
    if (!focused.value?.id) return;
    const cfg = sam3Cfg.value;
    if (!cfg.text_prompts.length && !cfg.boxes.length) {
        Notify.create({ type: 'warning', message: t('workspaceSam3NeedPrompt') });
        return;
    }
    sam3PreviewLoading.value = true;
    sam3PreviewImg.value = null;
    api.post(`/sensor/${focused.value.id}/sam3_preview`, {
        text_prompts: cfg.text_prompts.filter(p => p && p.trim()),
        boxes: cfg.boxes,
        mode: cfg.mode,
        color: cfg.color,
    })
        .then((res) => {
            sam3PreviewImg.value = res.data.image;
        })
        .catch((err) => {
            Notify.create({
                type: 'negative',
                message: err?.response?.data?.message || String(err),
            });
        })
        .finally(() => {
            sam3PreviewLoading.value = false;
        });
}
</script>
<style>
.crop-area {
    position: absolute;
    border: 2px dashed red;
    pointer-events: none;
}
.crop-area-saved {
    position: absolute;
    border: 2px solid green;
    pointer-events: none;
}
</style>