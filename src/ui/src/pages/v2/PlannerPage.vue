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

        <!-- Main content area with three columns -->
        <div class="col q-mb-lg border-rounded border-grey text-grey flex-center flex text-h6" v-if="!selectedPlannerId">
            Select Planner First
        </div>
        <div class="col row q-mb-lg" v-else>
            <!-- First Column: Workspace Selection (1/7 width, using col-2) -->
            <div class="col-2 bg-secondary q-mr-lg border-rounded q-pa-sm">
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
                <q-scroll-area style="height: calc(100% - 100px);"> <!-- Adjust height for scrollable area -->
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
                                    <div v-for="sensor in workspace.settingd.sensors" :key="sensor.id" class="row items-center q-mb-xs">
                                        <div>{{ sensor.name }}</div>
                                        <q-space />
                                        <q-toggle
                                            :model-value="sensor.handler?.status === 'on'"
                                            dense
                                            color="positive"
                                            @click="toggleSensor(workspace.id, sensor)"
                                        ></q-toggle>
                                    </div>
                                    <div class="text-subtitle2 q-mt-md q-mb-sm">Robots</div>
                                    <div v-for="robot in workspace.robots" :key="robot.id" class="row items-center q-mb-xs">
                                        <div>{{ robot.name }}</div>
                                        <q-space />
                                        <q-toggle
                                            :model-value="robot.handler?.status === 'on'"
                                            dense
                                            color="positive"
                                            @click="toggleRobot(workspace.id, robot)"
                                        ></q-toggle>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
                </q-scroll-area>
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
            title="Add Workspaces to Planner"
            :form="addWorkspacesForm"
            @submit="updatePlanner({ workspace_ids: addWorkspacesForm.find(e => e.key === 'workspace_ids').value })"
            ok-button-label="Save"
        ></form-dialog>

    </q-page>
</template>

<script setup>
import { ref, onMounted, computed, watch, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import FormDialog from 'src/components/v2/FormDialog.vue';
import { Notify, Loading } from 'quasar';
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
};

function listPlanners() {
    return api.get('/planners').then((response) => {
        planners.value = response.data.planners;
        planners.value.push({ id: 'new', name: 'Create New Planner +' });
    });
}

const selectedPlanner = computed(() => {
    return planners.value.find(w => w.id === selectedPlannerId.value) || {};
});

console.log('Selected Planner:', selectedPlanner);

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

// --- New Workspace Selection Logic ---
const showAddWorkspacesForm = ref(false);
const availableWorkspaces = ref([]); // All available workspaces
const addWorkspacesForm = ref([
    { key: 'workspace_ids', label: 'Workspaces', type: 'multiselect_list', options: computed(() => availableWorkspaces.value.map(w => ({ label: w.name, value: w.id }))), value: [], required: false }
]);
const selectedWorkspaceIds = ref([]); // IDs of workspaces linked to the current planner

const selectedWorkspaces = computed(() => {
    return availableWorkspaces.value.filter(w => selectedWorkspaceIds.value.includes(w.id));
});

function openAddWorkspacesForm() {
    addWorkspacesForm.value.find(e => e.key === 'workspace_ids').value = selectedWorkspaceIds.value.slice(); // Pre-select currently linked workspaces
    showAddWorkspacesForm.value = true;
}

// Fetches all workspaces, similar to listPlanners but for /tasks endpoint
function listAvailableWorkspaces() {
    return api.get('/tasks').then((response) => {
        // 1. tasks 데이터가 존재하는지 먼저 확인 (기본값 빈 배열)
        const tasks = response.data?.tasks || [];

        availableWorkspaces.value = tasks.map(workspace => {
            const augmentedWorkspace = { ...workspace };

            // 2. sensors가 없을 경우를 대비해 빈 배열로 처리 후 map 실행
            augmentedWorkspace.sensors = (workspace.sensors || []).map(sensor => {
                const handler = useSensor(sensor);
                return { ...sensor, handler: handler };
            });

            // 3. robots가 없을 경우를 대비해 빈 배열로 처리 후 map 실행
            augmentedWorkspace.robots = (workspace.robots || []).map(robot => {
                const handler = useRobot(robot);
                return { ...robot, handler: handler };
            });

            return augmentedWorkspace;
        });
    }).catch((error) => {
        console.error('Error fetching available workspaces:', error);
    });
}

function listSelectedWorkspaces() {
    selectedWorkspaceIds.value = selectedPlanner.value.task_ids || [];
    selectedWorkspaces.value = availableWorkspaces.value.filter(w => selectedWorkspaceIds.value.includes(w.id));
}

function toggleSensor(workspace, sensor) {
    if (!sensor.handler) return;

    sensor.process_id = `sensor_${workspace.id}_${sensor.id}`;
    if (sensor.handler.status === 'on') {
        sensor.handler.stopSensor();
    } else {
        sensor.handler.startSensor();
    }
}

function toggleRobot(workspace, robot) {
    if (!robot.handler) return;

    robot.process_id = `robot_${workspace.id}_${robot.id}`;
    const startFlow = () => robot.handler.startRobot();
    if (robot.handler.status === 'on') {
        robot.handler.stopRobot();
    } else if (robot.handler.status === 'error') {
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
        console.log('Updated Planner with workspaces:', form.workspace_ids);
    })
}

watch(selectedPlannerId, () => {
    listSelectedWorkspaces();
});


onUnmounted(() => {
});

onMounted(() => {
    listPlanners();
    listAvailableWorkspaces();
    listSelectedWorkspaces();
});
</script>