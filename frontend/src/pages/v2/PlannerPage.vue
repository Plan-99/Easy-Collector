<template>
    <q-page :class="['q-pt-lg q-pr-lg q-pb-lg', { 'full-height column': isEmptyState }]">
        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerIntro')" />
        <!-- Planner selection header -->
        <div class="border-rounded bg-secondary q-pa-md q-mb-md row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="row">
                    <div class="text-h5 text-primary text-bold q-mb-md">{{ $t('plannerIntroTitle') }}</div>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="selectedPlannerId"
                        :options="planners"
                        :label="$t('plannerSelectLabel')"
                        style="width: 400px"
                        class="q-ml-md"
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
        <div class="row q-mb-lg items-stretch" v-else>
            <!-- First Column: Workspace Selection -->
            <div class="col-2 bg-secondary q-mr-sm border-rounded q-pa-sm">
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
            </div>

            <!-- Right side: Plans (top, horizontal timelines) + Monitoring (bottom) -->
            <div class="col column">
                <!-- Plans area: 모든 그룹이 통째로 보이도록 자연 높이로 둠 -->
                <div class="bg-secondary border-rounded q-pa-sm q-mb-sm">
                    <div class="row items-center q-mb-sm">
                        <div class="text-h6 text-white q-mr-md">{{ $t('plannerPlansTitle') }}</div>
                        <TutorialHint step="2" :text="$t('tutorialPlannerBlocks')" class="q-mr-sm" />
                        <TutorialHint step="3" :text="$t('tutorialPlannerRun')" class="q-mr-sm" />
                        <q-space />
                        <q-btn
                            v-if="!isRunning"
                            unelevated
                            color="positive"
                            icon="play_arrow"
                            :label="$t('plannerRunAllGroups')"
                            size="sm"
                            :disable="!hasAnyBlock || plans.length === 0"
                            @click="startRun(null)"
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
                            @click="stopRun"
                        ></q-btn>
                    </div>

                    <div>
                        <div v-if="!plans.length" class="text-grey text-center q-pa-xl">
                            {{ $t('plannerNoGroups') }}
                        </div>

                        <div
                            v-for="group in plans"
                            :key="group.id"
                            class="border-rounded q-mb-sm q-pa-sm"
                            :class="groupRunningSet.has(group.id) ? 'bg-blue-grey-10 border-primary' : 'bg-dark'"
                        >
                            <!-- Track header -->
                            <div class="row items-center q-mb-xs">
                                <q-icon name="hub" size="xs" class="q-mr-xs text-white" />
                                <div class="text-body2 text-white text-weight-medium">{{ groupTitle(group) }}</div>
                                <q-badge color="grey-8" class="q-ml-sm">
                                    {{ $t('plannerGroupBlocksCount', { count: (group.blocks || []).length }) }}
                                </q-badge>
                                <q-space />
                                <div v-if="runStatusByGroup[group.id]" class="text-caption text-grey-5 q-mr-sm ellipsis" style="max-width: 360px;">
                                    {{ runStatusByGroup[group.id] }}
                                </div>
                                <q-btn
                                    v-if="!isRunning"
                                    size="sm"
                                    flat
                                    round
                                    color="positive"
                                    icon="play_arrow"
                                    :disable="!(group.blocks || []).length"
                                    @click.stop="startRun(group.id)"
                                >
                                    <q-tooltip>{{ $t('plannerRunGroup') }}</q-tooltip>
                                </q-btn>
                                <q-spinner
                                    v-else-if="groupRunningSet.has(group.id)"
                                    color="primary"
                                    size="xs"
                                />
                                <q-btn
                                    v-if="!isRunning"
                                    size="sm"
                                    flat
                                    round
                                    color="primary"
                                    icon="add"
                                    @click.stop="openBlockForm(group)"
                                >
                                    <q-tooltip>{{ $t('plannerNewBlock') }}</q-tooltip>
                                </q-btn>
                            </div>

                            <!-- Horizontal block track -->
                            <div v-if="!(group.blocks || []).length" class="text-caption text-grey text-center q-pa-md">
                                {{ $t('plannerNoBlocks') }}
                            </div>
                            <div v-else class="row no-wrap items-stretch" style="overflow-x: auto; overflow-y: hidden;">
                                <div
                                    v-for="(block, index) in (group.blocks || [])"
                                    :key="block.id"
                                    class="planner-block-card column q-mr-xs"
                                    :class="[
                                        runningByGroup[group.id] === index ? 'bg-blue-grey-9' : 'bg-grey-10',
                                        { 'block-card-active': isDragOver(group.id, index) || runningByGroup[group.id] === index }
                                    ]"
                                    :draggable="!isRunning"
                                    @dragstart="onDragStart(group.id, index, $event)"
                                    @dragover.prevent="onDragOver(group.id, index, $event)"
                                    @drop="onDrop(group.id, index)"
                                    @dragend="onDragEnd"
                                    :style="{ cursor: isRunning ? 'default' : 'grab' }"
                                >
                                    <div :style="{ height: '3px', backgroundColor: blockTypeColor(block.type) }"></div>
                                    <div class="q-pa-xs col column">
                                        <div class="row items-center no-wrap">
                                            <q-icon
                                                :name="blockConfigs[block.type]?.icon || 'help'"
                                                :color="blockConfigs[block.type]?.color || 'grey'"
                                                size="xs"
                                                class="q-mr-xs"
                                            />
                                            <q-spinner
                                                v-if="runningByGroup[group.id] === index"
                                                color="primary"
                                                size="xs"
                                            />
                                            <q-icon
                                                v-else-if="blockResultIcon(group.id, block.id)"
                                                :name="blockResultIcon(group.id, block.id)"
                                                :color="blockResultColor(group.id, block.id)"
                                                size="xs"
                                            />
                                            <q-space />
                                            <q-icon name="drag_indicator" color="grey-7" size="xs" v-if="!isRunning" />
                                        </div>
                                        <div class="text-caption text-bold text-white ellipsis q-mt-xs">
                                            {{ block.name || blockConfigs[block.type]?.label }}
                                        </div>
                                        <div class="text-caption text-grey-5 ellipsis">
                                            <span v-if="block.type === 'joint_position'">
                                                {{ getWorkspaceName(block.workspace_id) }}
                                            </span>
                                            <span v-else-if="block.type === 'checkpoint'">
                                                {{ block.checkpoint_name }}
                                            </span>
                                            <span v-else-if="block.type === 'timesleep'">—</span>
                                            <span v-else-if="block.type === 'sync'">
                                                ID: {{ block.sync_id || '—' }}
                                            </span>
                                        </div>
                                        <q-space />
                                        <div class="row q-mt-xs" v-if="block.type === 'sync' || block.duration != null || block.until_done">
                                            <q-badge
                                                v-if="block.type === 'sync'"
                                                rounded
                                                color="teal-8"
                                            >
                                                <q-icon name="sync" size="xs" class="q-mr-xs" />{{ $t('plannerSyncBadge') }}
                                            </q-badge>
                                            <q-badge
                                                v-else-if="block.type === 'checkpoint' && block.until_done"
                                                rounded
                                                color="purple-8"
                                            >
                                                <q-icon name="check_circle" size="xs" class="q-mr-xs" />{{ $t('plannerUntilDoneBadge') }}
                                            </q-badge>
                                            <q-badge
                                                v-else
                                                rounded
                                                :color="block.type === 'timesleep' ? 'orange-8' : 'purple-8'"
                                            >
                                                <q-icon name="timer" size="xs" class="q-mr-xs" />{{ block.duration }}s
                                            </q-badge>
                                        </div>
                                    </div>
                                    <q-menu context-menu v-if="!isRunning">
                                        <q-list bordered separator>
                                            <q-item clickable v-ripple v-close-popup @click="openBlockDetails(block)">
                                                <q-item-section>{{ $t('plannerShowDetails') }}</q-item-section>
                                                <q-item-section side><q-icon name="info" size="xs" /></q-item-section>
                                            </q-item>
                                            <q-item clickable v-ripple v-close-popup @click="openEditBlockForm(group, block, index)">
                                                <q-item-section>{{ $t('plannerEditBlock') }}</q-item-section>
                                                <q-item-section side><q-icon name="edit" size="xs" /></q-item-section>
                                            </q-item>
                                            <q-item clickable v-ripple v-close-popup @click="copyBlock(group, block, index)">
                                                <q-item-section>{{ $t('plannerCopyBlock') }}</q-item-section>
                                                <q-item-section side><q-icon name="content_copy" size="xs" /></q-item-section>
                                            </q-item>
                                            <q-item clickable v-ripple v-close-popup @click="deleteBlock(group, index)">
                                                <q-item-section class="text-red">{{ $t('plannerDeleteBlock') }}</q-item-section>
                                                <q-item-section side><q-icon name="delete" size="xs" color="red" /></q-item-section>
                                            </q-item>
                                        </q-list>
                                    </q-menu>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Monitoring (bottom): 페이지가 자라는 만큼만 보이도록 고정 높이 -->
                <div style="height: 600px;">
                    <monitoring-window
                        class="full-height"
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
                        <div class="q-mb-sm" v-if="detailBlock.until_done">
                            <span class="text-bold">{{ $t('plannerUntilDone') }}:</span>
                            {{ $t('plannerDoneThreshold') }} = {{ detailBlock.done_threshold ?? 0.5 }}
                        </div>
                        <div class="q-mb-sm" v-else>
                            <span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s
                        </div>
                    </template>
                    <template v-if="detailBlock.type === 'timesleep'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                    </template>
                    <template v-if="detailBlock.type === 'sync'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerSyncId') }}:</span> {{ detailBlock.sync_id }}</div>
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
                    <div v-if="formGroupId" class="text-caption text-grey-5 q-mt-xs">
                        {{ $t('plannerForGroup') }}: {{ groupTitle(formGroup) }}
                    </div>
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
                            v-if="formWorkspaceOptions.length > 1"
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.workspace_id"
                            :options="formWorkspaceOptions"
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
                                class="q-mb-md"
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

                                <q-separator dark class="q-my-md" />
                                <div class="text-caption text-grey-5 q-mb-sm">
                                    <q-icon name="gamepad" size="xs" class="q-mr-xs" />
                                    {{ $t('plannerPendantHint') }}
                                </div>

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
                            v-if="formWorkspaceOptions.length > 1"
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.workspace_id"
                            :options="formWorkspaceOptions"
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
                            :disable="blockForm.until_done"
                        ></q-input>
                        <q-toggle
                            v-model="blockForm.until_done"
                            :label="$t('plannerUntilDone')"
                            color="purple"
                            dark
                            class="q-mb-xs"
                            :disable="!selectedCheckpointHasSucceed"
                        ></q-toggle>
                        <div class="text-caption text-grey q-mb-md">
                            {{ selectedCheckpointHasSucceed ? $t('plannerUntilDoneHint') : $t('plannerCheckpointNoSucceed') }}
                        </div>
                        <q-input
                            v-if="blockForm.until_done"
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.done_threshold"
                            :label="$t('plannerDoneThreshold')"
                            :hint="$t('plannerDoneThresholdHint')"
                            type="number"
                            step="0.05"
                            min="0"
                            max="1"
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

                    <!-- Sync Fields -->
                    <template v-if="blockForm.type === 'sync'">
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.sync_id"
                            :label="$t('plannerSyncId')"
                            :hint="$t('plannerSyncIdHint')"
                            class="q-mb-md"
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
import { ref, onMounted, computed, watch, onUnmounted, reactive } from 'vue';
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
const isEmptyState = computed(() => pageLoading.value || !selectedPlannerId.value);
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
        loadPlans();
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

// --- Plans (groups) ---
const plans = ref([]);

function loadPlans() {
    const raw = selectedPlanner.value.plans;
    plans.value = Array.isArray(raw) ? raw.map(g => ({
        id: g.id,
        workspace_ids: g.workspace_ids || [],
        blocks: Array.isArray(g.blocks) ? g.blocks : [],
    })) : [];
}

const hasAnyBlock = computed(() => plans.value.some(g => (g.blocks || []).length > 0));

function saveGroupBlocks(group) {
    return updatePlanner({ group_id: group.id, blocks: group.blocks });
}

const BLOCK_TYPE_COLORS = {
    joint_position: '#2196F3',
    checkpoint: '#9C27B0',
    timesleep: '#FF9800',
    sync: '#009688',
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

function groupTitle(group) {
    if (!group) return '';
    const names = (group.workspace_ids || []).map(getWorkspaceName);
    return names.join(' + ');
}

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
const formGroupId = ref(null);
const blockForm = ref({
    type: null,
    name: '',
    workspace_id: null,
    checkpoint_id: null,
    duration: 5,
    until_done: false,
    done_threshold: 0.5,
    sync_id: '',
    hz: 10,
    re_inference_steps: 1,
    temporal_ensemble_coeff: 0.01,
    positions: {},
});

const formGroup = computed(() => plans.value.find(g => g.id === formGroupId.value) || null);

const selectedCheckpointHasSucceed = computed(() => {
    const cp = checkpoints.value.find(c => c.id === blockForm.value.checkpoint_id);
    return !!cp?.train_settings?.has_succeed;
});

watch(() => blockForm.value.checkpoint_id, () => {
    if (!selectedCheckpointHasSucceed.value) {
        blockForm.value.until_done = false;
    }
});

// 폼이 특정 그룹에 묶여있을 때 그 그룹의 워크스페이스들만 보여줌.
const formWorkspaceOptions = computed(() => {
    const grp = formGroup.value;
    const wsIds = grp ? grp.workspace_ids : selectedWorkspaceIds.value;
    return wsIds
        .map(id => availableWorkspaces.value.find(w => w.id === id))
        .filter(Boolean)
        .map(w => ({ label: w.name, value: w.id }));
});

function openBlockForm(group) {
    editingBlockIndex.value = null;
    formGroupId.value = group ? group.id : null;
    const wsIds = group?.workspace_ids || [];
    // 그룹이 워크스페이스 1개만 가지면 자동 선택 — 다이얼로그에서 드롭다운을 숨김.
    blockForm.value = {
        type: null,
        name: '',
        workspace_id: wsIds.length === 1 ? wsIds[0] : null,
        checkpoint_id: null,
        duration: 5,
        hz: 10,
        re_inference_steps: 1,
        temporal_ensemble_coeff: 0.01,
        positions: {},
    };
    showBlockForm.value = true;
}

function openEditBlockForm(group, block, index) {
    editingBlockIndex.value = index;
    formGroupId.value = group.id;
    blockForm.value = {
        type: block.type,
        name: block.name || '',
        workspace_id: block.workspace_id || null,
        checkpoint_id: block.checkpoint_id || null,
        duration: block.duration || 5,
        until_done: !!block.until_done,
        done_threshold: typeof block.done_threshold === 'number' ? block.done_threshold : 0.5,
        sync_id: block.sync_id || '',
        hz: typeof block.hz === 'number' ? block.hz : 10,
        re_inference_steps: typeof block.re_inference_steps === 'number' ? block.re_inference_steps : 1,
        temporal_ensemble_coeff: typeof block.temporal_ensemble_coeff === 'number' ? block.temporal_ensemble_coeff : 0.01,
        positions: block.positions ? JSON.parse(JSON.stringify(block.positions)) : {},
    };
    showBlockForm.value = true;
}

function copyBlock(group, block, index) {
    const copied = JSON.parse(JSON.stringify(block));
    copied.id = Date.now().toString(36) + Math.random().toString(36).substring(2, 7);
    copied.name = (copied.name || '') + ' (copy)';
    group.blocks.splice(index + 1, 0, copied);
    saveGroupBlocks(group);
}

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
    const duration = Number(blockForm.value.duration) || 5.0;
    robot.handler.moveToPose(pose, { duration });
}

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

    const grp = formGroup.value;
    if (!grp) {
        Notify.create({ color: 'negative', message: t('plannerNoGroupSelected') });
        return;
    }

    const config = blockConfigs.value[blockForm.value.type];
    if (!config) return;

    const isEditing = editingBlockIndex.value !== null;
    const block = {
        id: isEditing ? grp.blocks[editingBlockIndex.value].id : Date.now().toString(36) + Math.random().toString(36).substring(2, 7),
        type: blockForm.value.type,
    };

    config.keys.forEach(key => {
        block[key] = blockForm.value[key];
    });

    if (block.type === 'checkpoint' && block.checkpoint_id) {
        const cp = checkpoints.value.find(c => c.id === block.checkpoint_id);
        block.checkpoint_name = cp?.name || '';
    }

    if (!block.name) {
        if (block.type === 'joint_position') {
            block.name = t('plannerNameAutoMove', { workspace: getWorkspaceName(block.workspace_id) });
        } else if (block.type === 'checkpoint') {
            block.name = t('plannerNameAutoCheckpoint', { checkpoint: block.checkpoint_name });
        } else if (block.type === 'timesleep') {
            block.name = t('plannerNameAutoSleep', { duration: block.duration });
        } else if (block.type === 'sync') {
            block.name = t('plannerNameAutoSync', { sync_id: block.sync_id });
        }
    }

    if (isEditing) {
        grp.blocks.splice(editingBlockIndex.value, 1, block);
    } else {
        grp.blocks.push(block);
    }
    saveGroupBlocks(grp);
    showBlockForm.value = false;
}

function deleteBlock(group, index) {
    group.blocks.splice(index, 1);
    saveGroupBlocks(group);
}

// --- Drag and Drop (group-scoped) ---
const dragState = ref({ groupId: null, index: null });
const dragOverState = ref({ groupId: null, index: null });

function isDragOver(groupId, index) {
    return dragOverState.value.groupId === groupId && dragOverState.value.index === index;
}

function onDragStart(groupId, index, event) {
    dragState.value = { groupId, index };
    event.dataTransfer.effectAllowed = 'move';
}

function onDragOver(groupId, index) {
    // 같은 그룹 안에서만 reorder 표시.
    if (dragState.value.groupId !== groupId) return;
    dragOverState.value = { groupId, index };
}

function onDrop(groupId, index) {
    const ds = dragState.value;
    if (ds.groupId !== groupId) return;  // 그룹 간 이동 금지
    if (ds.index === null || ds.index === index) return;
    const grp = plans.value.find(g => g.id === groupId);
    if (!grp) return;
    const item = grp.blocks.splice(ds.index, 1)[0];
    grp.blocks.splice(index, 0, item);
    saveGroupBlocks(grp);
}

function onDragEnd() {
    dragState.value = { groupId: null, index: null };
    dragOverState.value = { groupId: null, index: null };
}

// --- Run / Stop ---
const isRunning = ref(false);
const repeatActive = ref(false);
const currentIterationByGroup = reactive({});
const runningByGroup = reactive({});  // groupId -> currently running block index
const runStatusByGroup = reactive({});  // groupId -> status text
const blockResultsByGroup = reactive({});  // groupId -> { blockId: 'finished'|'stopped'|'error' }
const groupRunningSet = ref(new Set());  // groupIds currently mid-run

function blockResultIcon(groupId, blockId) {
    const r = blockResultsByGroup[groupId]?.[blockId];
    if (r === 'finished') return 'check_circle';
    if (r === 'stopped') return 'cancel';
    if (r === 'error') return 'error';
    return null;
}

function blockResultColor(groupId, blockId) {
    const r = blockResultsByGroup[groupId]?.[blockId];
    if (r === 'finished') return 'positive';
    if (r === 'stopped') return 'orange';
    if (r === 'error') return 'negative';
    return 'grey';
}

function resetRunState() {
    Object.keys(runningByGroup).forEach(k => delete runningByGroup[k]);
    Object.keys(runStatusByGroup).forEach(k => delete runStatusByGroup[k]);
    Object.keys(blockResultsByGroup).forEach(k => delete blockResultsByGroup[k]);
    Object.keys(currentIterationByGroup).forEach(k => delete currentIterationByGroup[k]);
    groupRunningSet.value = new Set();
}

function startRun(groupId) {
    if (!selectedPlannerId.value) return;
    resetRunState();
    api.post(`/planner/${selectedPlannerId.value}/:start_run`, {
        repeat_count: repeatActive.value ? 0 : 1,
        group_id: groupId,
    }).then((res) => {
        isRunning.value = true;
        const ids = res.data?.group_ids || [];
        ids.forEach(gid => {
            runStatusByGroup[gid] = t('plannerStartingStatus');
        });
    }).catch((error) => {
        const data = error.response?.data || {};
        const errors = Array.isArray(data.errors) ? data.errors.join('\n') : data.message || t('plannerStartFailed');
        Notify.create({
            color: 'negative',
            message: errors,
            timeout: 8000,
            multiLine: true,
        });
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
    resetRunState();
    const ids = payload?.group_ids || [];
    ids.forEach(gid => {
        runStatusByGroup[gid] = t('plannerStartingStatus');
    });
}

function onPlannerGroupStart(payload) {
    const gid = payload?.group_id;
    if (!gid) return;
    const newSet = new Set(groupRunningSet.value);
    newSet.add(gid);
    groupRunningSet.value = newSet;
    runStatusByGroup[gid] = t('plannerRunningStatus', { current: 0, total: payload?.total ?? 0 });
}

function onPlannerIterationStart(payload) {
    const gid = payload?.group_id;
    if (!gid) return;
    currentIterationByGroup[gid] = payload?.iteration || 0;
    blockResultsByGroup[gid] = {};
    runningByGroup[gid] = undefined;
}

function onPlannerBlockStart(payload) {
    const gid = payload?.group_id;
    if (!gid) return;
    runningByGroup[gid] = payload?.index ?? null;
    let text = t('plannerRunningStatusDetail', {
        current: (payload?.index ?? 0) + 1,
        total: payload?.total ?? 0,
        name: payload?.name || payload?.type,
    });
    if (repeatActive.value && (currentIterationByGroup[gid] || 0) > 0) {
        text += ` · ${t('plannerIterationInfinite', { current: currentIterationByGroup[gid] })}`;
    }
    runStatusByGroup[gid] = text;
}

function onPlannerBlockEnd(payload) {
    const gid = payload?.group_id;
    if (!gid) return;
    if (payload?.block_id) {
        if (!blockResultsByGroup[gid]) blockResultsByGroup[gid] = {};
        blockResultsByGroup[gid] = { ...blockResultsByGroup[gid], [payload.block_id]: payload.status };
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

function onPlannerGroupEnd(payload) {
    const gid = payload?.group_id;
    if (!gid) return;
    const newSet = new Set(groupRunningSet.value);
    newSet.delete(gid);
    groupRunningSet.value = newSet;
    runningByGroup[gid] = undefined;
    runStatusByGroup[gid] = '';
}

function onPlannerRunEnd(payload) {
    isRunning.value = false;
    Object.keys(runningByGroup).forEach(k => delete runningByGroup[k]);
    Object.keys(runStatusByGroup).forEach(k => delete runStatusByGroup[k]);
    groupRunningSet.value = new Set();
    const status = payload?.status || 'finished';
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
            resetRunState();
        }
    }).catch(() => {});
}

// --- Watchers ---
watch(selectedPlannerId, () => {
    listSelectedWorkspaces();
    loadPlans();
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

watch(() => tutorialStore.running, (isRun) => {
    if (isRun) {
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
    socket.off('planner_group_start', onPlannerGroupStart);
    socket.off('planner_iteration_start', onPlannerIterationStart);
    socket.off('planner_block_start', onPlannerBlockStart);
    socket.off('planner_block_end', onPlannerBlockEnd);
    socket.off('planner_group_end', onPlannerGroupEnd);
    socket.off('planner_run_end', onPlannerRunEnd);
});

onMounted(async () => {
    pageLoading.value = true;
    loadBlockConfigs();
    listCheckpoints();
    listPlanners();
    await listAvailableWorkspaces();
    await new Promise(resolve => setTimeout(resolve, 2000));
    listSelectedWorkspaces();
    loadPlans();
    loadRunStatus();
    socket.on('planner_run_start', onPlannerRunStart);
    socket.on('planner_group_start', onPlannerGroupStart);
    socket.on('planner_iteration_start', onPlannerIterationStart);
    socket.on('planner_block_start', onPlannerBlockStart);
    socket.on('planner_block_end', onPlannerBlockEnd);
    socket.on('planner_group_end', onPlannerGroupEnd);
    socket.on('planner_run_end', onPlannerRunEnd);
    pageLoading.value = false;
});
</script>

<style scoped>
.planner-block-card {
    width: 160px;
    min-width: 160px;
    max-width: 160px;
    flex-shrink: 0;
    border-radius: 6px;
    border: 1px solid rgba(255, 255, 255, 0.06);
    overflow: hidden;
    transition: border-color 0.15s ease;
}
.planner-block-card.block-card-active {
    border-color: var(--q-primary);
}
</style>
