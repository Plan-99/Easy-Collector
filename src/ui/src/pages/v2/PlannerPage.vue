<template>
    <q-page class="q-pt-lg q-pr-lg full-height column">
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
                        label="Select Planner"
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
            <div class="text-h6 text-grey">Initializing...</div>
        </div>

        <!-- Main content area -->
        <div class="col q-mb-lg border-rounded border-grey text-grey flex-center flex text-h6" v-else-if="!selectedPlannerId">
            Select Planner First
        </div>
        <div class="col row q-mb-lg" v-else>
            <!-- First Column: Workspace Selection -->
            <div class="col-2 bg-secondary q-mr-sm border-rounded q-pa-sm">
                <div class="text-h6 text-white q-mb-md">Workspaces</div>
                <q-btn
                    outline
                    class="full-width q-mb-sm"
                    rounded
                    color="primary bg-dark"
                    icon="add"
                    label="Add Workspaces"
                    @click="openAddWorkspacesForm"
                ></q-btn>
                <q-scroll-area style="height: calc(100% - 100px);">
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
                                    <div class="text-subtitle2 q-mb-sm">Sensors</div>
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
                                            <div class="text-caption text-grey" v-if="sensor.status === 'off'">TOPIC OFF</div>
                                            <div class="text-caption text-positive" v-else-if="sensor.status === 'on'">TOPIC ON</div>
                                        </div>
                                        <q-inner-loading :showing="sensor.status === 'loading'"></q-inner-loading>
                                    </div>
                                    <div class="text-subtitle2 q-mt-md q-mb-sm">Robots</div>
                                    <div v-if="!workspace.assembly" class="text-caption text-grey q-mb-sm">No assembly assigned</div>
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
                                            <div class="text-caption text-grey" v-if="robot.status === 'off'">TOPIC OFF</div>
                                            <div class="text-caption text-positive" v-else-if="robot.status === 'on'">TOPIC ON</div>
                                            <div class="text-caption text-negative" v-else-if="robot.status === 'error'">ERROR</div>
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
            <div class="col-3 bg-secondary q-mr-sm border-rounded q-pa-sm">
                <div class="row items-center q-mb-md">
                    <div class="text-h6 text-white">Blocks</div>
                    <q-space />
                    <q-btn
                        outline
                        rounded
                        color="primary bg-dark"
                        icon="add"
                        label="Create New Block"
                        @click="openBlockForm"
                        size="sm"
                    ></q-btn>
                </div>
                <q-scroll-area style="height: calc(100% - 50px);">
                    <div v-if="!blocks.length" class="text-grey text-center q-pa-xl">
                        No blocks yet. Create a new block to get started.
                    </div>
                    <div
                        v-for="(block, index) in blocks"
                        :key="block.id"
                        class="q-my-xs border-rounded row items-center text-white"
                        :class="[
                            'bg-dark',
                            { 'border-primary': dragOverIndex === index }
                        ]"
                        draggable="true"
                        @dragstart="onDragStart(index, $event)"
                        @dragover.prevent="onDragOver(index, $event)"
                        @drop="onDrop(index)"
                        @dragend="onDragEnd"
                        style="cursor: grab; overflow: hidden;"
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
                            <q-icon name="drag_indicator" color="grey" class="q-ml-xs" />
                        </div>
                        <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click="openBlockDetails(block)">
                                    <q-item-section>Show Details</q-item-section>
                                    <q-item-section side><q-icon name="info" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup @click="openEditBlockForm(block, index)">
                                    <q-item-section>Edit Block</q-item-section>
                                    <q-item-section side><q-icon name="edit" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup @click="copyBlock(block, index)">
                                    <q-item-section>Copy Block</q-item-section>
                                    <q-item-section side><q-icon name="content_copy" size="xs" /></q-item-section>
                                </q-item>
                                <q-item clickable v-ripple v-close-popup @click="deleteBlock(index)">
                                    <q-item-section class="text-red">Delete Block</q-item-section>
                                    <q-item-section side><q-icon name="delete" size="xs" color="red" /></q-item-section>
                                </q-item>
                            </q-list>
                        </q-menu>
                    </div>
                </q-scroll-area>
            </div>

            <!-- Third Column: Monitoring Window -->
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
            />
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
            title="Add Workspaces to Planner"
            :form="addWorkspacesForm"
            @submit="updatePlanner({ workspace_ids: addWorkspacesForm.find(e => e.key === 'workspace_ids').value })"
            ok-button-label="Save"
        ></form-dialog>

        <!-- Block Details Dialog -->
        <q-dialog v-model="showBlockDetails">
            <q-card style="min-width: 400px;" class="bg-secondary text-white">
                <q-card-section>
                    <div class="text-h6">Block Details</div>
                </q-card-section>
                <q-card-section v-if="detailBlock">
                    <div class="q-mb-sm"><span class="text-bold">Type:</span> {{ blockConfigs[detailBlock.type]?.label }}</div>
                    <div class="q-mb-sm"><span class="text-bold">Name:</span> {{ detailBlock.name }}</div>
                    <template v-if="detailBlock.type === 'joint_position'">
                        <div class="q-mb-sm"><span class="text-bold">Workspace:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div v-for="robot in getWorkspaceRobots(detailBlock.workspace_id)" :key="robot.id" class="q-mb-xs">
                            <div class="text-bold text-caption">{{ robot.name }}</div>
                            <div class="text-caption text-grey">{{ (detailBlock.positions?.[robot.id] || []).map(v => Number(v).toFixed(4)).join(', ') }}</div>
                        </div>
                    </template>
                    <template v-if="detailBlock.type === 'checkpoint'">
                        <div class="q-mb-sm"><span class="text-bold">Workspace:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div class="q-mb-sm"><span class="text-bold">Checkpoint:</span> {{ detailBlock.checkpoint_name }}</div>
                        <div class="q-mb-sm"><span class="text-bold">Duration:</span> {{ detailBlock.duration }}s</div>
                    </template>
                    <template v-if="detailBlock.type === 'timesleep'">
                        <div class="q-mb-sm"><span class="text-bold">Duration:</span> {{ detailBlock.duration }}s</div>
                    </template>
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat label="Close" color="grey" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <!-- Block Create/Edit Dialog -->
        <q-dialog v-model="showBlockForm" persistent>
            <q-card :style="{ minWidth: blockForm.type === 'joint_position' && blockForm.workspace_id ? '900px' : '500px' }" class="bg-secondary text-white">
                <q-card-section>
                    <div class="text-h6">{{ editingBlockIndex !== null ? 'Edit Block' : 'Create New Block' }}</div>
                </q-card-section>

                <q-card-section>
                    <!-- Block Type -->
                    <q-select
                        dense outlined dark bg-color="dark"
                        v-model="blockForm.type"
                        :options="blockTypeOptions"
                        label="Block Type"
                        class="q-mb-md"
                        map-options
                        emit-value
                        :disable="editingBlockIndex !== null"
                    ></q-select>

                    <!-- Block Name -->
                    <q-input
                        dense outlined dark bg-color="dark"
                        v-model="blockForm.name"
                        label="Block Name"
                        class="q-mb-md"
                    ></q-input>

                    <!-- Joint Position Fields -->
                    <template v-if="blockForm.type === 'joint_position'">
                        <q-select
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.workspace_id"
                            :options="workspaceOptions"
                            label="Workspace"
                            class="q-mb-md"
                            map-options
                            emit-value
                        ></q-select>
                        <div v-if="blockForm.workspace_id">
                            <div
                                v-for="robot in getWorkspaceRobots(blockForm.workspace_id)"
                                :key="robot.id"
                                class="q-mb-md"
                            >
                                <div class="row items-center q-mb-xs">
                                    <div class="text-subtitle2">{{ robot.name }}</div>
                                    <q-badge
                                        :color="robot.status === 'on' ? 'positive' : 'grey'"
                                        class="q-ml-sm"
                                    >
                                        {{ robot.status === 'on' ? 'ONLINE' : 'OFFLINE' }}
                                    </q-badge>
                                    <q-space />
                                    <q-btn
                                        size="xs" outline color="primary"
                                        label="Read Current"
                                        icon="sync"
                                        @click="readCurrentJoints(robot)"
                                        :disable="robot.status !== 'on'"
                                    />
                                </div>
                                <!-- Robot Pendant for jogging -->
                                <robot-pendant
                                    v-if="robot.status === 'on'"
                                    :robot="robot"
                                    class="q-mb-sm"
                                />
                                <div v-else class="text-caption text-grey q-mb-sm">
                                    Turn on the robot from Workspaces panel to use the pendant.
                                </div>
                                <!-- Joint position inputs -->
                                <div class="row q-gutter-xs" v-if="blockForm.positions[robot.id]">
                                    <q-input
                                        v-for="(jointName, jIdx) in (robot.joint_names || [])"
                                        :key="jIdx"
                                        dense outlined dark bg-color="dark"
                                        type="number"
                                        :label="jointName"
                                        style="width: 100px;"
                                        v-model.number="blockForm.positions[robot.id][jIdx]"
                                    ></q-input>
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
                            label="Workspace"
                            class="q-mb-md"
                            map-options
                            emit-value
                        ></q-select>
                        <q-select
                            v-if="blockForm.workspace_id"
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.checkpoint_id"
                            :options="filteredCheckpoints"
                            label="Checkpoint"
                            class="q-mb-md"
                            map-options
                            emit-value
                            option-label="name"
                            option-value="id"
                        ></q-select>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            label="Duration (seconds)"
                            type="number"
                        ></q-input>
                    </template>

                    <!-- Timesleep Fields -->
                    <template v-if="blockForm.type === 'timesleep'">
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            label="Duration (seconds)"
                            type="number"
                        ></q-input>
                    </template>
                </q-card-section>

                <q-card-actions align="right">
                    <q-btn flat label="Cancel" color="grey" @click="showBlockForm = false" />
                    <q-btn flat label="Save" color="primary" @click="saveBlock" />
                </q-card-actions>
            </q-card>
        </q-dialog>

    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import FormDialog from 'src/components/v2/FormDialog.vue';
import MonitoringWindow from 'src/components/v2/MonitoringWindow.vue';
import RobotPendant from 'src/components/v2/RobotPendant.vue';
import { Notify } from 'quasar';
import { useSensor } from '../../composables/useSensor';
import { useRobot } from 'src/composables/useRobot';


const planners = ref([]);
const selectedPlannerId = ref(null);
const showPlannerForm = ref(false);
const plannerForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: 'Planner Name', type: 'text', value: '', default: '' },
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
        planners.value.push({ id: 'new', name: 'Create New Planner +' });
    });
}

const selectedPlanner = computed(() => {
    return planners.value.find(w => w.id === selectedPlannerId.value) || {};
});

function deletePlanner(planner) {
    Notify.create({
        message: `Are you sure you want to delete planner "${planner.name}"?`,
        color: 'negative',
        actions: [
            { label: 'Cancel', color: 'white', handler: () => { /* do nothing */ } },
            { label: 'Delete', color: 'white', handler: () => {
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
    { key: 'workspace_ids', label: 'Workspaces', type: 'multiselect_list', options: computed(() => availableWorkspaces.value.map(w => ({ label: w.name, value: w.id }))), value: [], required: false }
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

        availableWorkspaces.value = tasks.map(workspace => {
            const augmentedWorkspace = { ...workspace };

            augmentedWorkspace.sensors = (workspace.sensors || []).map(sensor => {
                sensor.handler = useSensor(sensor);
                return sensor;
            });

            if (augmentedWorkspace.assembly && augmentedWorkspace.assembly.robots) {
                augmentedWorkspace.assembly.robots = augmentedWorkspace.assembly.robots.map(robot => {
                    robot.handler = useRobot(robot);
                    return robot;
                });
            }

            return augmentedWorkspace;
        });
    }).catch((error) => {
        console.error('Error fetching available workspaces:', error);
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
    selectedWorkspaces.value.forEach(ws => {
        if (ws.sensor_img_size) {
            Object.assign(sensorImgSize, ws.sensor_img_size);
        }
    });
    return {
        sensor_img_size: sensorImgSize,
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
    return ws ? ws.name : 'Unknown';
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

// workspace 선택 시 positions 초기화 (신규 생성 또는 workspace 변경 시에만)
watch(() => blockForm.value.workspace_id, (newId, oldId) => {
    if (blockForm.value.type !== 'joint_position' || !newId) return;
    // edit 모드에서 최초 로드 시에는 기존 positions 유지
    if (editingBlockIndex.value !== null && oldId === null) return;
    const robots = getWorkspaceRobots(newId);
    const positions = {};
    robots.forEach(robot => {
        const dim = robot.joint_names?.length || robot.joint_dim || 6;
        positions[robot.id] = Array(dim).fill(0);
    });
    blockForm.value.positions = positions;
});

function readCurrentJoints(robot) {
    if (robot.jointState && robot.jointState.length) {
        blockForm.value.positions[robot.id] = [...robot.jointState];
    }
}

function saveBlock() {
    if (!blockForm.value.type) {
        Notify.create({ color: 'negative', message: 'Please select a block type.' });
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
            block.name = `Move - ${getWorkspaceName(block.workspace_id)}`;
        } else if (block.type === 'checkpoint') {
            block.name = `Run ${block.checkpoint_name}`;
        } else if (block.type === 'timesleep') {
            block.name = `Sleep ${block.duration}s`;
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

// --- Watchers ---
watch(selectedPlannerId, () => {
    listSelectedWorkspaces();
    loadBlocks();
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
    pageLoading.value = false;
});
</script>
