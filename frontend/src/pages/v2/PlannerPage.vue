<template>
    <q-page class="q-pt-lg q-pr-lg full-height column">
        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerIntro')" />
        <!-- Planner selection header -->
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-lg">{{ $t('plannerIntroTitle') }}</div>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="selectedPlannerId"
                        :options="planners"
                        :label="$t('plannerSelectLabel')"
                        style="width: 400px"
                        class="q-ml-lg"
                        map-options
                        emit-value
                        option-label="name"
                        option-value="id"
                        :disable="pageLoading"
                    >
                        <template v-slot:option="scope">
                            <q-item :props="scope.props" v-if="scope.opt.id === 'new'" v-bind="scope.itemProps" @click="openCreatePlannerForm">
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
                                        <q-btn icon="edit" size="sm" flat round @click.stop="openEditPlannerForm(scope.opt)" />
                                        <q-btn icon="delete" size="sm" color="red" flat round @click.stop="deletePlanner(scope.opt)" />
                                    </div>
                                </q-item-section>
                            </q-item>
                        </template>
                    </q-select>
                </div>
                <div class="text-body text-white">{{ $t('plannerIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('plannerIntroBody2') }}</div>
            </div>
        </div>

        <!-- Page loading -->
        <div class="col q-mb-lg border-rounded border-grey flex-center flex column" v-if="pageLoading">
            <q-spinner-gears size="50px" color="primary" class="q-mb-md" />
            <div class="text-h6 text-grey">{{ $t('plannerInitializing') }}</div>
        </div>

        <!-- Main content area -->
        <div class="col q-mb-lg border-rounded border-grey text-grey flex-center flex text-h6" v-else-if="!selectedPlannerId">
            {{ $t('plannerSelectFirst') }}
        </div>
        <div class="col row q-mb-lg" v-else>
            <!-- First Column: Workspace Selection -->
            <div class="col-2 bg-secondary q-mr-sm border-rounded q-pa-sm column">
                <div class="text-h6 text-white q-mb-md">{{ $t('plannerWorkspacesTitle') }}</div>
                <TutorialHint step="1" class="q-mb-sm" :text="$t('tutorialPlannerWorkspaces')" />
                <q-btn
                    outline
                    class="full-width q-mb-sm"
                    rounded
                    color="primary bg-dark"
                    icon="add"
                    :label="$t('plannerAddWorkspaces')"
                    @click="openAddWorkspacesForm"
                ></q-btn>
                <q-scroll-area class="col">
                    <q-list bordered separator class="border-rounded bg-dark" dark>
                        <q-expansion-item
                            v-for="workspace in selectedWorkspaces"
                            :key="workspace.id"
                            class="text-white"
                            expand-separator
                        >
                            <template v-slot:header>
                                <q-item-section>
                                    <q-item-label>{{ workspace.name }}</q-item-label>
                                </q-item-section>
                                <q-item-section side>
                                    <q-btn icon="close" size="sm" flat round @click.stop="removeWorkspace(workspace.id)" />
                                </q-item-section>
                            </template>
                            <q-card class="bg-dark text-white">
                                <q-card-section>
                                    <div class="text-subtitle2 q-mb-sm">{{ $t('plannerSensorsTitle') }}</div>
                                    <div v-for="sensor in workspace.sensors" :key="sensor.id"
                                        class="q-pa-sm q-px-md q-my-xs border-rounded row items-center"
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
                                    <div class="text-subtitle2 q-mt-md q-mb-sm">{{ $t('plannerRobotsTitle') }}</div>
                                    <div v-if="!workspace.assembly" class="text-caption text-grey q-mb-sm">{{ $t('plannerNoAssembly') }}</div>
                                    <div v-for="robot in (workspace.assembly?.robots || [])" :key="robot.id"
                                        class="q-pa-sm q-px-md q-my-xs border-rounded row items-center"
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
                                            <div class="text-caption text-negative" v-else-if="robot.status === 'error'">{{ $t('plannerErrorBadge') }}</div>
                                        </div>
                                        <q-inner-loading :showing="robot.status === 'loading'"></q-inner-loading>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
                </q-scroll-area>
            </div>

            <!-- Second Column: Blocks Panel -->
            <div class="col-3 bg-secondary q-mr-sm border-rounded q-pa-sm column">
                <div class="row items-center q-mb-md">
                    <div class="text-h6 text-white">{{ $t('plannerBlocksTitle') }}</div>
                    <q-space />
                    <q-btn
                        outline
                        rounded
                        color="primary bg-dark"
                        icon="add"
                        :label="$t('plannerNewBlock')"
                        @click="openBlockForm"
                        size="sm"
                        :disable="isRunning"
                    ></q-btn>
                </div>

                <TutorialHint step="2" class="q-mb-sm" :text="$t('tutorialPlannerBlocks')" />
                <TutorialHint step="3" class="q-mb-sm" :text="$t('tutorialPlannerRun')" />

                <!-- Run / Stop bar -->
                <div class="row items-center q-mb-sm">
                    <q-btn
                        v-if="!isRunning"
                        unelevated
                        color="positive"
                        icon="play_arrow"
                        :label="$t('plannerRunAll')"
                        size="sm"
                        class="col"
                        :disable="!blocks.length"
                        @click="startRun"
                    >
                        <q-badge
                            @click.stop="repeatActive = !repeatActive"
                            :color="repeatActive ? 'blue' : 'grey-5'"
                            floating>
                            <q-icon name="repeat" size="xs" class="cursor-pointer" />
                            <q-tooltip>{{ repeatActive ? $t('plannerRepeatOn') : $t('plannerRepeatOff') }}</q-tooltip>
                        </q-badge>
                    </q-btn>
                    <q-btn
                        v-else
                        unelevated
                        color="negative"
                        icon="stop"
                        :label="$t('plannerStop')"
                        size="sm"
                        class="col"
                        @click="stopRun"
                    ></q-btn>
                </div>
                <div v-if="isRunning && runStatusText" class="text-caption text-grey q-mb-xs">
                    {{ runStatusText }}
                </div>
                <q-scroll-area class="col">
                    <div v-if="!blocks.length" class="text-grey text-center q-pa-xl">
                        {{ $t('plannerNoBlocks') }}
                    </div>
                    <div
                        v-for="(block, index) in blocks"
                        :key="block.id"
                        class="q-my-xs border-rounded row items-center text-white"
                        :class="[
                            blockRowBg(index),
                            { 'border-primary': dragOverIndex === index || runningIndex === index }
                        ]"
                        :draggable="!isRunning"
                        @dragstart="onDragStart(index, $event)"
                        @dragover.prevent="onDragOver(index, $event)"
                        @drop="onDrop(index)"
                        @dragend="onDragEnd"
                        :style="{ cursor: isRunning ? 'default' : 'grab', overflow: 'hidden' }"
                    >
                        <div
                            class="self-stretch"
                            :style="{ width: '4px', backgroundColor: blockTypeColor(block.type) }"
                        ></div>
                        <div class="q-pa-sm q-px-md row items-center col">
                            <q-icon
                                :name="blockConfigs[block.type]?.icon || 'help'"
                                :color="blockConfigs[block.type]?.color || 'grey'"
                                size="sm"
                                class="q-mr-sm"
                            />
                            <div class="col">
                                <div class="text-body2 text-bold">{{ block.name || blockConfigs[block.type]?.label }}</div>
                                <div class="text-caption text-grey">
                                    <span v-if="block.type === 'joint_position'">
                                        {{ getWorkspaceName(block.workspace_id) }}
                                    </span>
                                    <span v-else-if="block.type === 'checkpoint'">
                                        {{ block.checkpoint_name }}
                                    </span>
                                </div>
                            </div>
                            <q-badge
                                v-if="block.duration != null"
                                rounded
                                :color="block.type === 'timesleep' ? 'orange-8' : 'purple-8'"
                                class="q-mr-sm"
                            >
                                <q-icon name="timer" size="xs" class="q-mr-xs" />{{ block.duration }}s
                            </q-badge>
                            <q-spinner
                                v-if="runningIndex === index"
                                color="primary"
                                size="sm"
                                class="q-mr-xs"
                            />
                            <q-icon
                                v-else-if="blockResultIcon(index)"
                                :name="blockResultIcon(index)"
                                :color="blockResultColor(index)"
                                size="sm"
                                class="q-mr-xs"
                            />
                            <q-icon name="drag_indicator" color="grey" class="q-ml-xs" v-if="!isRunning" />
                        </div>
                        <q-menu context-menu v-if="!isRunning">
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click="openBlockDetails(block)">
                                    <q-item-section>{{ $t('plannerShowDetails') }}</q-item-section>
                                    <q-item-section side><q-icon name="info" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup @click="openEditBlockForm(block, index)">
                                    <q-item-section>{{ $t('plannerEditBlock') }}</q-item-section>
                                    <q-item-section side><q-icon name="edit" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup @click="copyBlock(block, index)">
                                    <q-item-section>{{ $t('plannerCopyBlock') }}</q-item-section>
                                    <q-item-section side><q-icon name="content_copy" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup @click="deleteBlock(index)">
                                    <q-item-section class="text-red">{{ $t('plannerDeleteBlock') }}</q-item-section>
                                    <q-item-section side><q-icon name="delete" size="xs" color="red" /></q-item-section>
                                </q-item>
                            </q-list>
                        </q-menu>
                    </div>
                </q-scroll-area>
            </div>

            <!-- Third Column: Monitoring Window -->
            <div class="col column">
                <monitoring-window
                    class="col"
                    :workspace="monitorWorkspace"
                    :robots="allSelectedRobots"
                    :sensors="allSelectedSensors"
                    v-model:focused="focused"
                    v-model:selected-dataset-id="monitorDatasetId"
                    v-model:selected-checkpoint-id="monitorCheckpointId"
                    v-model:selected-episode="monitorEpisode"
                    :datasets="[]"
                    :checkpoints="[]"
                    status="pending"
                    monitor-only
                />
            </div>
        </div>

        <!-- Planner forms and dialogs -->
        <form-dialog
            v-model="showPlannerForm"
            :title="$t(plannerForm.find((e) => e.key === 'id').value ? 'plannerEditFormTitle' : 'plannerCreateFormTitle')"
            :form="plannerForm"
            :ok-button-label="$t(plannerForm.find((e) => e.key === 'id').value ? 'save' : 'create')"
            :cancel-button-label="$t('cancel')"
            @submit="savePlanner"
        ></form-dialog>

        <!-- Form for selecting Workspaces to add to Planner -->
        <form-dialog
            v-model="showAddWorkspacesForm"
            :title="$t('plannerAddWorkspacesTitle')"
            :form="addWorkspacesForm"
            @submit="updatePlanner({ workspace_ids: addWorkspacesForm.find(e => e.key === 'workspace_ids').value })"
            :ok-button-label="$t('save')"
        ></form-dialog>

        <!-- Block Details Dialog -->
        <q-dialog v-model="showBlockDetails">
            <q-card style="min-width: 400px;" class="bg-secondary text-white">
                <q-card-section>
                    <div class="text-h6">{{ $t('plannerBlockDetails') }}</div>
                </q-card-section>
                <q-card-section v-if="detailBlock">
                    <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsType') }}:</span> {{ blockConfigs[detailBlock.type]?.label }}</div>
                    <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsName') }}:</span> {{ detailBlock.name }}</div>
                    <template v-if="detailBlock.type === 'joint_position'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsWorkspace') }}:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                        <div v-for="robot in getWorkspaceRobots(detailBlock.workspace_id)" :key="robot.id" class="q-mb-xs">
                            <div class="text-bold text-caption">{{ robot.name }}</div>
                            <div class="text-caption text-grey">{{ (detailBlock.positions?.[robot.id] || []).map(v => Number(v).toFixed(4)).join(', ') }}</div>
                        </div>
                    </template>
                    <template v-if="detailBlock.type === 'checkpoint'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsWorkspace') }}:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsCheckpoint') }}:</span> {{ detailBlock.checkpoint_name }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                    </template>
                    <template v-if="detailBlock.type === 'timesleep'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                    </template>
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat :label="$t('close')" color="grey" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <!-- Block Create/Edit Dialog -->
        <q-dialog v-model="showBlockForm" persistent>
            <q-card :style="{ minWidth: blockForm.type === 'joint_position' && blockForm.workspace_id ? '900px' : '500px' }" class="bg-secondary text-white">
                <q-card-section>
                    <div class="text-h6">{{ editingBlockIndex !== null ? $t('plannerEditBlock') : $t('plannerCreateBlock') }}</div>
                </q-card-section>

                <q-card-section>
                    <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockForm')" />
                    <!-- Block Type -->
                    <q-select
                        dense outlined dark bg-color="dark"
                        v-model="blockForm.type"
                        :options="blockTypeOptions"
                        :label="$t('plannerBlockTypeLabel')"
                        class="q-mb-md"
                        map-options
                        emit-value
                        :disable="editingBlockIndex !== null"
                    ></q-select>

                    <!-- Block Name -->
                    <q-input
                        dense outlined dark bg-color="dark"
                        v-model="blockForm.name"
                        :label="$t('plannerBlockNameLabel')"
                        class="q-mb-md"
                    ></q-input>

                    <!-- Joint Position Fields -->
                    <template v-if="blockForm.type === 'joint_position'">
                        <q-select
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.workspace_id"
                            :options="workspaceOptions"
                            :label="$t('plannerWorkspaceLabel')"
                            class="q-mb-md"
                            map-options
                            emit-value
                        ></q-select>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            :label="$t('plannerDurationSeconds')"
                            type="number"
                            min="0.1"
                            step="0.1"
                            class="q-mb-md"
                        ></q-input>
                        <div v-if="blockForm.workspace_id">
                            <div
                                v-for="robot in getWorkspaceRobots(blockForm.workspace_id)"
                                :key="robot.id"
                                class="q-mb-lg"
                            >
                                <div class="row items-center q-mb-sm">
                                    <div class="text-subtitle2">{{ robot.name }}</div>
                                    <q-badge
                                        :color="robot.status === 'on' ? 'positive' : 'grey'"
                                        class="q-ml-sm"
                                    >
                                        {{ robot.status === 'on' ? $t('plannerOnline') : $t('plannerOffline') }}
                                    </q-badge>
                                </div>

                                <!-- Saved Position (강조) -->
                                <div
                                    v-if="blockForm.positions[robot.id]"
                                    class="border-rounded border-primary bg-dark q-pa-md q-mb-md"
                                >
                                    <div class="row items-center q-mb-sm">
                                        <q-icon name="bookmark" color="primary" class="q-mr-xs" />
                                        <div class="text-caption text-primary text-uppercase letter-spacing-1">{{ $t('plannerSavedPosition') }}</div>
                                        <q-space />
                                        <q-btn
                                            size="sm" unelevated color="primary"
                                            icon="play_arrow"
                                            :label="$t('plannerMoveBtn')"
                                            :disable="robot.status !== 'on'"
                                            @click="moveToSavedPosition(robot)"
                                            class="q-mr-sm"
                                        />
                                        <q-btn
                                            size="sm" outline color="amber"
                                            icon="my_location"
                                            :label="$t('plannerApplyCurrentPos')"
                                            :disable="robot.status !== 'on'"
                                            @click="applyCurrentPos(robot)"
                                        />
                                    </div>
                                    <div class="row q-gutter-sm">
                                        <div
                                            v-for="(jointName, jIdx) in (robot.joint_names || [])"
                                            :key="jIdx"
                                            class="border-rounded border-primary q-px-md q-py-xs text-center"
                                            style="min-width: 100px;"
                                        >
                                            <div class="text-caption text-grey-5">{{ jointName }}</div>
                                            <div class="text-subtitle1 text-primary">
                                                {{ typeof blockForm.positions[robot.id][jIdx] === 'number' ? blockForm.positions[robot.id][jIdx].toFixed(4) : '0' }}
                                            </div>
                                        </div>
                                    </div>
                                </div>

                                <!-- 경계선 -->
                                <q-separator dark class="q-my-md" />
                                <div class="text-caption text-grey-5 q-mb-sm">
                                    <q-icon name="gamepad" size="xs" class="q-mr-xs" />
                                    {{ $t('plannerPendantHint') }}
                                </div>

                                <!-- Robot Pendant for jogging -->
                                <robot-pendant
                                    v-if="robot.status === 'on'"
                                    :robot="robot"
                                />
                                <div v-else class="text-caption text-grey q-mb-sm">
                                    {{ $t('plannerPendantOffHint') }}
                                </div>
                            </div>
                        </div>
                    </template>

                    <!-- Checkpoint Fields -->
                    <template v-if="blockForm.type === 'checkpoint'">
                        <q-select
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.workspace_id"
                            :options="workspaceOptions"
                            :label="$t('plannerWorkspaceLabel')"
                            class="q-mb-md"
                            map-options
                            emit-value
                        ></q-select>
                        <q-select
                            v-if="blockForm.workspace_id"
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.checkpoint_id"
                            :options="filteredCheckpoints"
                            :label="$t('plannerCheckpointLabel')"
                            class="q-mb-md"
                            map-options
                            emit-value
                            option-label="name"
                            option-value="id"
                        ></q-select>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            :label="$t('plannerDurationSeconds')"
                            type="number"
                            class="q-mb-md"
                        ></q-input>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.hz"
                            :label="$t('plannerHzLabel')"
                            :hint="$t('plannerHzHint')"
                            type="number"
                            min="1"
                            class="q-mb-md"
                        ></q-input>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.re_inference_steps"
                            :label="$t('reInferenceSteps')"
                            :hint="$t('reInferenceStepsHint')"
                            type="number"
                            min="1"
                            class="q-mb-md"
                        ></q-input>
                        <q-input
                            v-if="blockForm.re_inference_steps === 1"
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.temporal_ensemble_coeff"
                            :label="$t('temporalEnsembleCoeff')"
                            :hint="$t('temporalEnsembleCoeffHint')"
                            type="number"
                            step="0.01"
                        ></q-input>
                    </template>

                    <!-- Timesleep Fields -->
                    <template v-if="blockForm.type === 'timesleep'">
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            :label="$t('plannerDurationSeconds')"
                            type="number"
                        ></q-input>
                    </template>
                </q-card-section>

                <q-card-actions align="right">
                    <q-btn flat :label="$t('cancel')" color="grey" @click="showBlockForm = false" />
                    <q-btn flat :label="$t('save')" color="primary" @click="saveBlock" />
                </q-card-actions>
            </q-card>
        </q-dialog>

    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch, onUnmounted } from 'vue';
import { useI18n } from 'vue-i18n';
import { api } from 'src/boot/axios';
import FormDialog from 'src/components/v2/FormDialog.vue';
import MonitoringWindow from 'src/components/v2/MonitoringWindow.vue';
import RobotPendant from 'src/components/v2/RobotPendant.vue';
import TutorialHint from 'src/components/v2/TutorialHint.vue';
import { Notify } from 'quasar';
import { useSensor } from '../../composables/useSensor';
import { useRobot } from 'src/composables/useRobot';
import { useSocket } from 'src/composables/useSocket';
import { useTutorialStore } from 'src/stores/tutorialStore';

const { t } = useI18n();
const { socket } = useSocket();
const tutorialStore = useTutorialStore();


const planners = ref([]);
const selectedPlannerId = ref(null);
const showPlannerForm = ref(false);
const plannerForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('plannerNameLabel'), type: 'text', value: '', default: '' },
]);

function openCreatePlannerForm() {
    selectedPlannerId.value = null;
    plannerForm.value.forEach((field) => {
        field.value = field.default;
    });
    showPlannerForm.value = true;
}

function openEditPlannerForm(planner) {
    plannerForm.value.forEach((field) => {
        field.value = planner[field.key] || field.default;
    });
    plannerForm.value.find((e) => e.key === 'id').value = planner.id;
    showPlannerForm.value = true;
}

function savePlanner (form) {
    if (plannerForm.value.find((e) => e.key === 'id').value) {
        return api.put(`/planner/${form.id}`, form).then(() => {
            listPlanners().then(() => {
                selectedPlannerId.value = planners.value.find(w => w.name === form.name).id;
            });
        });
    }
    return api.post('/planner', form).then(() => {
        listPlanners().then(() => {
            selectedPlannerId.value = planners.value.find(w => w.name === form.name).id;
        });
    });
}

function listPlanners() {
    return api.get('/planners').then((response) => {
        planners.value = response.data.planners;
        planners.value.push({ id: 'new', name: t('plannerCreateNew') });
    });
}

const selectedPlanner = computed(() => {
    return planners.value.find(w => w.id === selectedPlannerId.value) || {};
});

function deletePlanner(planner) {
    Notify.create({
        message: t('plannerDeleteConfirm', { name: planner.name }),
        color: 'negative',
        actions: [
            { label: t('cancel'), color: 'white', handler: () => { /* do nothing */ } },
            { label: t('delete'), color: 'white', handler: () => {
                return api.delete(`/planner/${planner.id}`).then(() => {
                    listPlanners();
                    if (selectedPlannerId.value === planner.id) {
                        selectedPlannerId.value = null;
                    }
                });
            }}
        ]
    });
}

// --- Workspace Selection Logic ---
const pageLoading = ref(true);
const showAddWorkspacesForm = ref(false);
const availableWorkspaces = ref([]);
const addWorkspacesForm = ref([
    { key: 'workspace_ids', label: t('plannerWorkspacesTitle'), type: 'multiselect_list', options: computed(() => availableWorkspaces.value.map(w => ({ label: w.name, value: w.id }))), value: [], optional: true }
]);
const selectedWorkspaceIds = ref([]);

const selectedWorkspaces = computed(() => {
    return availableWorkspaces.value.filter(w => selectedWorkspaceIds.value.includes(w.id));
});

function openAddWorkspacesForm() {
    addWorkspacesForm.value.find(e => e.key === 'workspace_ids').value = selectedWorkspaceIds.value.slice();
    showAddWorkspacesForm.value = true;
}

function listAvailableWorkspaces() {
    return api.get('/tasks').then((response) => {
        const tasks = response.data?.tasks || [];
        availableWorkspaces.value = tasks;
        // ref에 할당된 후 reactive proxy를 통해 handler를 붙여야
        // useSensor/useRobot 클로저가 proxy 참조를 잡아 socket으로 들어오는 jointState 변화에 반응함.
        availableWorkspaces.value.forEach(workspace => {
            (workspace.sensors || []).forEach(sensor => {
                sensor.handler = useSensor(sensor);
                if (sensor.type === 'custom') {
                    sensor.handler.checkSensorTopic();
                }
            });
            (workspace.assembly?.robots || []).forEach(robot => {
                robot.handler = useRobot(robot);
                if (robot.type === 'custom') {
                    robot.handler.checkRobotTopic();
                }
            });
        });
    }).catch((error) => {
        console.error('Error fetching available workspaces:', error);
    });
}

function repollCustomDevices() {
    availableWorkspaces.value.forEach(workspace => {
        (workspace.sensors || []).forEach(sensor => {
            if (sensor.type === 'custom' && sensor.handler && sensor.status !== 'on') {
                sensor.handler.checkSensorTopic();
            }
        });
        (workspace.assembly?.robots || []).forEach(robot => {
            if (robot.type === 'custom' && robot.handler && robot.status !== 'on') {
                robot.handler.checkRobotTopic();
            }
        });
    });
}

function listSelectedWorkspaces() {
    selectedWorkspaceIds.value = selectedPlanner.value.task_ids || [];
}

function removeWorkspace(workspaceId) {
    const newIds = selectedWorkspaceIds.value.filter(id => id !== workspaceId);
    updatePlanner({ workspace_ids: newIds });
}

// --- Monitoring Window ---
const focused = ref({});
const monitorDatasetId = ref(null);
const monitorCheckpointId = ref(null);
const monitorEpisode = ref({});

const monitorWorkspace = computed(() => {
    const sensorImgSize = {};
    const sensorCroppedArea = {};
    const sensorRotate = {};
    selectedWorkspaces.value.forEach(ws => {
        if (ws.sensor_img_size) {
            Object.assign(sensorImgSize, ws.sensor_img_size);
        }
        if (ws.sensor_cropped_area) {
            Object.assign(sensorCroppedArea, ws.sensor_cropped_area);
        }
        if (ws.sensor_rotate) {
            Object.assign(sensorRotate, ws.sensor_rotate);
        }
    });
    return {
        sensor_img_size: sensorImgSize,
        sensor_cropped_area: sensorCroppedArea,
        sensor_rotate: sensorRotate,
        settings: {},
    };
});

const allSelectedSensors = computed(() => {
    const sensors = [];
    selectedWorkspaces.value.forEach(ws => {
        (ws.sensors || []).forEach(sensor => {
            if (!sensors.find(s => s.id === sensor.id)) {
                sensors.push(sensor);
            }
        });
    });
    return sensors;
});

const allSelectedRobots = computed(() => {
    const robots = [];
    selectedWorkspaces.value.forEach(ws => {
        (ws.assembly?.robots || []).forEach(robot => {
            if (!robots.find(r => r.id === robot.id)) {
                robots.push(robot);
            }
        });
    });
    return robots;
});

function toggleSensor(sensor) {
    sensor.process_id = `sensor_${sensor.id}`;
    if (sensor.status === 'on') {
        sensor.handler.stopSensor();
    } else {
        sensor.handler.startSensor();
    }
}

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

function updatePlanner(form) {
    return api.put(`/planner/${selectedPlannerId.value}`, form).then(() => {
        return listPlanners();
    }).then(() => {
        listSelectedWorkspaces();
    });
}

// --- Block Configs ---
const blockConfigs = ref({});

function loadBlockConfigs() {
    return api.get('/planner/block_configs').then((response) => {
        blockConfigs.value = response.data.block_configs;
    });
}

const blockTypeOptions = computed(() => {
    return Object.entries(blockConfigs.value).map(([key, config]) => ({
        label: config.label,
        value: key,
    }));
});

// --- Blocks Logic ---
const blocks = ref([]);

function loadBlocks() {
    const plan = selectedPlanner.value.plan;
    blocks.value = Array.isArray(plan) ? plan : [];
}

function saveBlocks() {
    return updatePlanner({ plan: blocks.value });
}

const BLOCK_TYPE_COLORS = {
    joint_position: '#2196F3',
    checkpoint: '#9C27B0',
    timesleep: '#FF9800',
};

function blockTypeColor(type) {
    return BLOCK_TYPE_COLORS[type] || '#9E9E9E';
}

function getWorkspaceName(workspaceId) {
    const ws = availableWorkspaces.value.find(w => w.id === workspaceId);
    return ws ? ws.name : t('plannerUnknown');
}

function getWorkspaceRobots(workspaceId) {
    const ws = availableWorkspaces.value.find(w => w.id === workspaceId);
    return ws?.assembly?.robots || [];
}

const workspaceOptions = computed(() => {
    return selectedWorkspaces.value.map(w => ({ label: w.name, value: w.id }));
});

// --- Checkpoints ---
const checkpoints = ref([]);

function listCheckpoints() {
    return api.get('/checkpoints', {
        params: { where: 'status,=,finished', order: 'created_at DESC' }
    }).then((response) => {
        checkpoints.value = response.data.checkpoints || [];
    }).catch((error) => {
        console.error('Error fetching checkpoints:', error);
    });
}

const filteredCheckpoints = computed(() => {
    if (!blockForm.value.workspace_id) return [];
    return checkpoints.value.filter(c => c.task_id === blockForm.value.workspace_id);
});

// --- Block Details ---
const showBlockDetails = ref(false);
const detailBlock = ref(null);

function openBlockDetails(block) {
    detailBlock.value = block;
    showBlockDetails.value = true;
}

// --- Block Form ---
const showBlockForm = ref(false);
const editingBlockIndex = ref(null);
const blockForm = ref({
    type: null,
    name: '',
    workspace_id: null,
    checkpoint_id: null,
    duration: 5,
    hz: 10,
    re_inference_steps: 1,
    temporal_ensemble_coeff: 0.01,
    positions: {},
});

function openBlockForm() {
    editingBlockIndex.value = null;
    blockForm.value = {
        type: null,
        name: '',
        workspace_id: null,
        checkpoint_id: null,
        duration: 5,
        hz: 10,
        re_inference_steps: 1,
        temporal_ensemble_coeff: 0.01,
        positions: {},
    };
    showBlockForm.value = true;
}

function openEditBlockForm(block, index) {
    editingBlockIndex.value = index;
    blockForm.value = {
        type: block.type,
        name: block.name || '',
        workspace_id: block.workspace_id || null,
        checkpoint_id: block.checkpoint_id || null,
        duration: block.duration || 5,
        hz: typeof block.hz === 'number' ? block.hz : 10,
        re_inference_steps: typeof block.re_inference_steps === 'number' ? block.re_inference_steps : 1,
        temporal_ensemble_coeff: typeof block.temporal_ensemble_coeff === 'number' ? block.temporal_ensemble_coeff : 0.01,
        positions: block.positions ? JSON.parse(JSON.stringify(block.positions)) : {},
    };
    showBlockForm.value = true;
}

function copyBlock(block, index) {
    const copied = JSON.parse(JSON.stringify(block));
    copied.id = Date.now().toString(36) + Math.random().toString(36).substring(2, 7);
    copied.name = (copied.name || '') + ' (copy)';
    blocks.value.splice(index + 1, 0, copied);
    saveBlocks();
}

// workspace 선택 시 positions 초기화/보강
watch(() => blockForm.value.workspace_id, (newId, oldId) => {
    if (blockForm.value.type !== 'joint_position' || !newId) return;
    const robots = getWorkspaceRobots(newId);
    const isEditingInitialLoad = editingBlockIndex.value !== null && oldId === null;
    const existing = blockForm.value.positions || {};
    const positions = {};
    robots.forEach(robot => {
        const dim = robot.joint_names?.length || robot.joint_dim || 6;
        const prior = existing[robot.id];
        if (isEditingInitialLoad && Array.isArray(prior) && prior.length === dim) {
            positions[robot.id] = [...prior];
        } else {
            positions[robot.id] = Array(dim).fill(0);
        }
    });
    blockForm.value.positions = positions;
});

function applyCurrentPos(robot) {
    if (robot.jointState && robot.jointState.length) {
        blockForm.value.positions[robot.id] = [...robot.jointState];
    }
}

function moveToSavedPosition(robot) {
    const pose = blockForm.value.positions?.[robot.id];
    if (!Array.isArray(pose) || !pose.length) return;
    if (!robot.handler) return;
    // 단발 publish 는 interp_node 가 default cmd_interval 만에 jump → duration 무시.
    // 실행 시 적용될 duration 과 동일하게 미리보기 이동도 부드럽게 보간.
    const duration = Number(blockForm.value.duration) || 5.0;
    robot.handler.moveToPose(pose, { duration });
}

// Edit Block dialog가 열리면 workspace 로봇들의 socket listener를 등록 (no-op callback).
// subscribeRobot 내부 listener가 robot.jointState/eePos를 proxy를 통해 쓰므로
// RobotPendant의 live 표시가 reactive하게 업데이트된다.
function ensureRobotLiveListeners() {
    if (blockForm.value.type !== 'joint_position' || !blockForm.value.workspace_id) return;
    const robots = getWorkspaceRobots(blockForm.value.workspace_id);
    robots.forEach(robot => {
        if (!robot.handler) return;
        if (robot.status !== 'on') return;
        api.post(`/robot/${robot.id}/:subscribe_robot`).catch(() => {});
        robot.handler.subscribeRobot(() => {});
    });
}

watch(showBlockForm, (open) => {
    if (open) ensureRobotLiveListeners();
});

watch(() => [blockForm.value.type, blockForm.value.workspace_id], () => {
    if (showBlockForm.value) ensureRobotLiveListeners();
});

function saveBlock() {
    if (!blockForm.value.type) {
        Notify.create({ color: 'negative', message: t('plannerSelectTypeError') });
        return;
    }

    const config = blockConfigs.value[blockForm.value.type];
    if (!config) return;

    const isEditing = editingBlockIndex.value !== null;
    const block = {
        id: isEditing ? blocks.value[editingBlockIndex.value].id : Date.now().toString(36) + Math.random().toString(36).substring(2, 7),
        type: blockForm.value.type,
    };

    // config.keys에 정의된 키만 블록에 포함
    config.keys.forEach(key => {
        block[key] = blockForm.value[key];
    });

    // checkpoint 선택 시 이름 저장
    if (block.type === 'checkpoint' && block.checkpoint_id) {
        const cp = checkpoints.value.find(c => c.id === block.checkpoint_id);
        block.checkpoint_name = cp?.name || '';
    }

    // 이름 자동 생성
    if (!block.name) {
        if (block.type === 'joint_position') {
            block.name = t('plannerNameAutoMove', { workspace: getWorkspaceName(block.workspace_id) });
        } else if (block.type === 'checkpoint') {
            block.name = t('plannerNameAutoCheckpoint', { checkpoint: block.checkpoint_name });
        } else if (block.type === 'timesleep') {
            block.name = t('plannerNameAutoSleep', { duration: block.duration });
        }
    }

    if (isEditing) {
        blocks.value.splice(editingBlockIndex.value, 1, block);
    } else {
        blocks.value.push(block);
    }
    saveBlocks();
    showBlockForm.value = false;
}

function deleteBlock(index) {
    blocks.value.splice(index, 1);
    saveBlocks();
}

// --- Drag and Drop ---
const dragIndex = ref(null);
const dragOverIndex = ref(null);

function onDragStart(index, event) {
    dragIndex.value = index;
    event.dataTransfer.effectAllowed = 'move';
}

function onDragOver(index) {
    dragOverIndex.value = index;
}

function onDrop(index) {
    if (dragIndex.value === null || dragIndex.value === index) return;
    const item = blocks.value.splice(dragIndex.value, 1)[0];
    blocks.value.splice(index, 0, item);
    saveBlocks();
}

function onDragEnd() {
    dragIndex.value = null;
    dragOverIndex.value = null;
}

// --- Run / Stop ---
const isRunning = ref(false);
// repeat 뱃지 ON이면 정지 버튼을 누를 때까지 plan을 무한 반복.
const repeatActive = ref(false);
const currentIteration = ref(0);
const runningIndex = ref(null);
const runStatusText = ref('');
// blockResults: { [block.id]: 'finished' | 'stopped' | 'error' }
const blockResults = ref({});

function blockRowBg(index) {
    if (runningIndex.value === index) return 'bg-blue-grey-9';
    return 'bg-dark';
}

function blockResultIcon(index) {
    const block = blocks.value[index];
    if (!block) return null;
    const r = blockResults.value[block.id];
    if (r === 'finished') return 'check_circle';
    if (r === 'stopped') return 'cancel';
    if (r === 'error') return 'error';
    return null;
}

function blockResultColor(index) {
    const block = blocks.value[index];
    const r = blockResults.value[block?.id];
    if (r === 'finished') return 'positive';
    if (r === 'stopped') return 'orange';
    if (r === 'error') return 'negative';
    return 'grey';
}

function startRun() {
    if (!selectedPlannerId.value) return;
    blockResults.value = {};
    runningIndex.value = null;
    currentIteration.value = 0;
    runStatusText.value = t('plannerStartingStatus');
    // repeat ON이면 0(=무한 반복) 전달, OFF면 1(기본 1회 실행).
    api.post(`/planner/${selectedPlannerId.value}/:start_run`, {
        repeat_count: repeatActive.value ? 0 : 1,
    }).then(() => {
        isRunning.value = true;
    }).catch((error) => {
        const data = error.response?.data || {};
        const errors = Array.isArray(data.errors) ? data.errors.join('\n') : data.message || t('plannerStartFailed');
        Notify.create({
            color: 'negative',
            message: errors,
            timeout: 8000,
            multiLine: true,
        });
        runStatusText.value = '';
    });
}

function stopRun() {
    if (!selectedPlannerId.value) return;
    api.post(`/planner/${selectedPlannerId.value}/:stop_run`).catch((error) => {
        console.error('Error stopping planner run:', error);
    });
}

function onPlannerRunStart(payload) {
    isRunning.value = true;
    runningIndex.value = null;
    blockResults.value = {};
    currentIteration.value = 0;
    runStatusText.value = t('plannerRunningStatus', { current: 0, total: payload?.total ?? blocks.value.length });
}

function onPlannerIterationStart(payload) {
    currentIteration.value = payload?.iteration || 0;
    // 새 회차 시작 시 블록 결과 표시 초기화 (UI에서 "한 바퀴" 시각적 분리).
    blockResults.value = {};
    runningIndex.value = null;
}

function onPlannerBlockStart(payload) {
    runningIndex.value = payload?.index ?? null;
    let text = t('plannerRunningStatusDetail', {
        current: (payload?.index ?? 0) + 1,
        total: payload?.total ?? blocks.value.length,
        name: payload?.name || payload?.type,
    });
    // repeat ON일 때만 회차 표시.
    if (repeatActive.value && currentIteration.value > 0) {
        text += ` · ${t('plannerIterationInfinite', { current: currentIteration.value })}`;
    }
    runStatusText.value = text;
}

function onPlannerBlockEnd(payload) {
    if (payload?.block_id) {
        blockResults.value = { ...blockResults.value, [payload.block_id]: payload.status };
    }
    if (payload?.status === 'error' && payload?.error) {
        Notify.create({
            color: 'negative',
            message: t('plannerBlockFailed', { name: payload.name, error: payload.error }),
            timeout: 6000,
            multiLine: true,
        });
    }
}

function onPlannerRunEnd(payload) {
    isRunning.value = false;
    runningIndex.value = null;
    const status = payload?.status || 'finished';
    runStatusText.value = '';
    Notify.create({
        color: status === 'finished' ? 'positive' : (status === 'stopped' ? 'warning' : 'negative'),
        message: status === 'finished' ? t('plannerRunFinished') :
                 status === 'stopped' ? t('plannerRunStopped') :
                 t('plannerRunFailed', { error: payload?.error || t('plannerUnknownError') }),
        timeout: 5000,
    });
}

function loadRunStatus() {
    if (!selectedPlannerId.value) return;
    api.get(`/planner/${selectedPlannerId.value}/:run_status`).then((res) => {
        isRunning.value = !!res.data?.is_running;
        if (!isRunning.value) {
            runningIndex.value = null;
            runStatusText.value = '';
        }
    }).catch(() => {});
}

// --- Watchers ---
watch(selectedPlannerId, () => {
    listSelectedWorkspaces();
    loadBlocks();
    loadRunStatus();
    repollCustomDevices();
});

watch(selectedWorkspaceIds, () => {
    repollCustomDevices();
}, { deep: true });

watch(() => tutorialStore.hasTopics, (hasTopics) => {
    if (hasTopics) {
        repollCustomDevices();
    }
});

watch(() => tutorialStore.running, (isRunning) => {
    if (isRunning) {
        repollCustomDevices();
    }
});

onUnmounted(() => {
    availableWorkspaces.value.forEach(workspace => {
        if (workspace.assembly?.robots) {
            workspace.assembly.robots.forEach(robot => {
                if (robot.handler) {
                    robot.handler.unSubscribeRobot();
                }
            });
        }
    });
    socket.off('planner_run_start', onPlannerRunStart);
    socket.off('planner_iteration_start', onPlannerIterationStart);
    socket.off('planner_block_start', onPlannerBlockStart);
    socket.off('planner_block_end', onPlannerBlockEnd);
    socket.off('planner_run_end', onPlannerRunEnd);
});

onMounted(async () => {
    pageLoading.value = true;
    loadBlockConfigs();
    listCheckpoints();
    listPlanners();
    await listAvailableWorkspaces();
    // useSensor의 checkSensorTopic(1) 폴링(1초) + sensor:stop 완료 대기
    await new Promise(resolve => setTimeout(resolve, 2000));
    listSelectedWorkspaces();
    loadBlocks();
    loadRunStatus();
    socket.on('planner_run_start', onPlannerRunStart);
    socket.on('planner_iteration_start', onPlannerIterationStart);
    socket.on('planner_block_start', onPlannerBlockStart);
    socket.on('planner_block_end', onPlannerBlockEnd);
    socket.on('planner_run_end', onPlannerRunEnd);
    pageLoading.value = false;
});
</script>
