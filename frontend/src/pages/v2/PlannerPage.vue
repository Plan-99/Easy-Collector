<template>
    <q-page class="q-pt-md q-pr-md full-height column">
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
        <div class="col q-mb-md border-rounded border-grey flex-center flex column" v-if="pageLoading">
            <q-spinner-gears size="50px" color="primary" class="q-mb-md" />
            <div class="text-h6 text-grey">{{ $t('plannerInitializing') }}</div>
        </div>

        <!-- Main content area -->
        <div class="col q-mb-md border-rounded border-grey text-grey flex-center flex text-h6" v-else-if="!selectedPlannerId">
            {{ $t('plannerSelectFirst') }}
        </div>
        <div class="col row q-mb-md" v-else>
            <!-- First Column: Workspace Selection -->
            <div class="col-2 bg-secondary q-mr-md border-rounded q-pa-sm">
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
                                    <q-item-label lines="1">{{ workspace.name }}</q-item-label>
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
                                        <div class="ellipsis">{{ sensor.name }}</div>
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
                                        <q-inner-loading
                                            :showing="sensor.status === 'loading'"
                                            style="pointer-events: none; background: rgba(0, 0, 0, 0.4)"
                                        ></q-inner-loading>
                                    </div>
                                    <div class="text-subtitle2 q-mt-md q-mb-sm">{{ $t('plannerRobotsTitle') }}</div>
                                    <div v-if="!workspace.assembly" class="text-caption text-grey q-mb-sm">{{ $t('plannerNoAssembly') }}</div>
                                    <div v-for="robot in (workspace.assembly?.robots || [])" :key="robot.id"
                                        class="q-pa-sm q-px-md q-my-xs border-rounded row items-center"
                                        :class="robot.status === 'on' ? 'bg-green-10' : (robot.status === 'error' ? 'bg-red-10' : 'bg-grey-8')"
                                    >
                                        <div class="ellipsis">{{ robot.name }}</div>
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
                                        <q-inner-loading
                                            :showing="robot.status === 'loading'"
                                            style="pointer-events: none; background: rgba(0, 0, 0, 0.4)"
                                        ></q-inner-loading>
                                    </div>
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
            </div>

            <!-- Right side: Plans (top, horizontal timelines) + Monitoring (bottom) -->
            <div class="col column">
                <!-- Plans area: 모든 그룹이 통째로 보이도록 자연 높이로 둠 -->
                <div class="bg-secondary border-rounded q-pa-sm q-mb-md">
                    <div class="row items-center q-mb-sm">
                        <TutorialHint step="2" :text="$t('tutorialPlannerBlocks')" class="q-mr-sm" />
                        <TutorialHint step="3" :text="$t('tutorialPlannerRun')" class="q-mr-sm" />
                        <q-space />
                        <q-btn
                            outline
                            color="white"
                            icon="download"
                            :label="$t('plannerExport')"
                            size="sm"
                            class="q-mr-sm"
                            :disable="!hasAnyBlock || plans.length === 0 || isRunning"
                            @click="exportPlanner"
                        >
                            <q-tooltip>{{ $t('plannerExportTooltip') }}</q-tooltip>
                        </q-btn>
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

                            <!-- Horizontal block track — q-scroll-area 로 가로
                                 스크롤. 카드들이 너비를 넘으면 자동 스크롤바. -->
                            <div v-if="!(group.blocks || []).length" class="text-caption text-grey text-center q-pa-md">
                                {{ $t('plannerNoBlocks') }}
                            </div>
                            <q-scroll-area
                                v-else
                                style="height: 145px; width: 100%;"
                                :thumb-style="{ height: '6px', borderRadius: '3px', background: 'var(--q-primary)', opacity: 0.5 }"
                                :bar-style="{ height: '8px', background: 'rgba(255,255,255,0.05)' }"
                            >
                                <div class="row no-wrap items-stretch">
                                <PlannerBlockCard
                                    v-for="(block, index) in (group.blocks || [])"
                                    :key="block.id"
                                    :block="block"
                                    :block-config="blockConfigs[block.type]"
                                    :running="runningByGroup[group.id] === index"
                                    :result="blockResultsByGroup[group.id]?.[block.id]"
                                    :progress="blockProgressById[block.id]"
                                    :workspace-name="getWorkspaceName(block.workspace_id)"
                                    :active="isDragOver(group.id, index) || runningByGroup[group.id] === index"
                                    width="160px"
                                    class="q-mr-xs"
                                    :style="{ cursor: isRunning ? 'default' : 'grab' }"
                                    :draggable="!isRunning"
                                    @dragstart="onDragStart(group.id, index, $event)"
                                    @dragover.prevent="onDragOver(group.id, index, $event)"
                                    @drop="onDrop(group.id, index)"
                                    @dragend="onDragEnd"
                                >
                                    <template #header-right>
                                        <q-icon name="drag_indicator" color="grey-7" size="xs" v-if="!isRunning" />
                                    </template>
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
                                </PlannerBlockCard>
                                </div>
                            </q-scroll-area>
                        </div>
                    </div>
                </div>

                <!-- Monitoring: 내용만큼 자연 높이로 두고 하단 마진을 맞춘다. -->
                <div class="q-mb-md">
                    <monitoring-window
                        class="full-width"
                        :workspace="monitorWorkspace"
                        :workspaces="selectedWorkspaces"
                        :robots="allSelectedRobots"
                        :sensors="allSelectedSensors"
                        :loading="detailsLoading"
                        v-model:focused="focused"
                        v-model:selected-dataset-id="monitorDatasetId"
                        v-model:selected-checkpoint-id="monitorCheckpointId"
                        v-model:selected-episode="monitorEpisode"
                        :datasets="[]"
                        :checkpoints="[]"
                        status="pending"
                        :visible-view-keys="monitorVisibleViewKeys"
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
                    <template v-if="detailBlock.type === 'move_relative_ee'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsWorkspace') }}:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                        <div v-for="robot in getWorkspaceRobots(detailBlock.workspace_id)" :key="robot.id" class="q-mb-xs">
                            <div class="text-bold text-caption">
                                {{ robot.name }}
                                <q-chip v-if="isToolRobot(robot)" dense color="amber-9" text-color="white" size="sm">tool</q-chip>
                            </div>
                            <div v-if="isToolRobot(robot)" class="text-caption text-grey">
                                abs = [{{ (detailBlock.tool_positions?.[robot.id] || []).map(v => Number(v).toFixed(4)).join(', ') }}]
                            </div>
                            <div v-else class="text-caption text-grey">
                                Δ = [{{ (detailBlock.deltas?.[robot.id] || [0,0,0,0,0,0]).map(v => Number(v).toFixed(4)).join(', ') }}]
                            </div>
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
                        <div class="q-mb-sm" v-if="detailBlock.max_steps">
                            <span class="text-bold">{{ $t('plannerMaxSteps') }}:</span> {{ detailBlock.max_steps }}
                        </div>
                    </template>
                    <template v-if="detailBlock.type === 'replay_episode'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsWorkspace') }}:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerReplayDatasetLabel') }}:</span> {{ detailBlock.dataset_name || detailBlock.dataset_id }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerReplayEpisodeLabel') }}:</span> {{ detailBlock.episode_index }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerHzLabel') }}:</span> {{ detailBlock.hz }}</div>
                        <div class="q-mb-sm">
                            <span class="text-bold">{{ $t('plannerReplayMoveToFirst') }}:</span>
                            {{ detailBlock.move_to_first ? 'yes' : 'no' }}
                            <span v-if="detailBlock.move_to_first && detailBlock.settle_sec > 0">
                                (+{{ detailBlock.settle_sec }}s)
                            </span>
                        </div>
                    </template>
                    <template v-if="detailBlock.type === 'timesleep'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                    </template>
                    <template v-if="detailBlock.type === 'sync'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerSyncId') }}:</span> {{ detailBlock.sync_id }}</div>
                    </template>
                    <template v-if="detailBlock.type === 'query_pose'">
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsWorkspace') }}:</span> {{ getWorkspaceName(detailBlock.workspace_id) }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerPoseTypeLabel') }}:</span>
                            {{ detailBlock.pose_type === 'end_effector_position' ? $t('plannerPoseTypeEE') : $t('plannerPoseTypeJoint') }}
                        </div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerServiceNameLabel') }}:</span> {{ detailBlock.service_name }}</div>
                        <div class="q-mb-sm"><span class="text-bold">{{ $t('plannerBlockDetailsDuration') }}:</span> {{ detailBlock.duration }}s</div>
                    </template>
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat :label="$t('close')" color="grey" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <!-- Query Pose: External Node Example Dialog -->
        <q-dialog v-model="showExampleNode">
            <q-card style="min-width: 640px; max-width: 880px;" class="bg-secondary text-white">
                <q-card-section class="row items-center q-pb-none">
                    <q-icon name="terminal" size="sm" class="q-mr-sm" />
                    <div class="text-h6">{{ $t('plannerExampleNodeTitle') }}</div>
                    <q-space />
                    <q-btn flat round dense icon="close" v-close-popup />
                </q-card-section>
                <q-card-section class="q-pt-sm">
                    <div class="text-caption text-grey-4 q-mb-sm">
                        {{ $t('plannerExampleNodeIntro') }}
                    </div>
                    <q-tabs
                        v-model="exampleTab"
                        dense
                        align="left"
                        active-color="primary"
                        indicator-color="primary"
                        class="text-grey-4"
                    >
                        <q-tab name="joint" :label="$t('plannerPoseTypeJoint')" />
                        <q-tab name="ee" :label="$t('plannerPoseTypeEE')" />
                    </q-tabs>
                    <q-separator dark />
                    <div
                        class="q-mt-md q-pa-sm"
                        style="background: #0c1116; border: 1px solid rgba(255,255,255,0.08); border-radius: 6px;"
                    >
                        <div class="row items-center q-mb-xs">
                            <q-icon name="circle" size="8px" color="red" class="q-mr-xs" />
                            <q-icon name="circle" size="8px" color="amber" class="q-mr-xs" />
                            <q-icon name="circle" size="8px" color="green" />
                            <q-space />
                            <q-btn
                                dense flat size="sm" icon="content_copy" color="grey-4"
                                @click="copyExampleNode"
                            >
                                <q-tooltip>{{ $t('plannerExampleCopy') }}</q-tooltip>
                            </q-btn>
                        </div>
                        <pre
                            style="margin: 0; color: #d6deeb; font-family: 'JetBrains Mono', 'Fira Code', Menlo, Consolas, monospace; font-size: 12px; line-height: 1.55; white-space: pre; overflow-x: auto; max-height: 480px;"
                        >{{ exampleNodeCode }}</pre>
                    </div>
                    <div class="text-caption text-grey-5 q-mt-md">
                        {{ $t('plannerExampleNodeRunHint') }}
                    </div>
                </q-card-section>
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
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockJointPosition')" />
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
                                        <div class="text-caption text-primary text-uppercase" style="letter-spacing: 1px;">{{ $t('plannerSavedPosition') }}</div>
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

                    <!-- Move Relative EE Fields -->
                    <template v-if="blockForm.type === 'move_relative_ee'">
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockMoveRelativeEE')" />
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
                                    <q-chip
                                        v-if="isToolRobot(robot)"
                                        dense color="amber-9" text-color="white"
                                        class="q-ml-sm"
                                    >tool</q-chip>
                                    <q-badge
                                        :color="robot.status === 'on' ? 'positive' : 'grey'"
                                        class="q-ml-sm"
                                    >
                                        {{ robot.status === 'on' ? $t('plannerOnline') : $t('plannerOffline') }}
                                    </q-badge>
                                </div>
                                <!-- Arm: 6-DOF EE delta -->
                                <div
                                    v-if="!isToolRobot(robot) && Array.isArray(blockForm.deltas[robot.id])"
                                    class="border-rounded border-cyan bg-dark q-pa-md"
                                >
                                    <div class="text-caption text-grey-5 q-mb-sm">
                                        {{ $t('plannerRelativeEEHint') }}
                                    </div>
                                    <div class="row q-col-gutter-sm">
                                        <div
                                            v-for="(label, idx) in ['dx', 'dy', 'dz', 'drx', 'dry', 'drz']"
                                            :key="idx"
                                            class="col-4"
                                        >
                                            <q-input
                                                dense outlined dark bg-color="dark"
                                                hide-bottom-space
                                                v-model.number="blockForm.deltas[robot.id][idx]"
                                                :label="label"
                                                type="number"
                                                step="0.01"
                                            ></q-input>
                                        </div>
                                    </div>
                                </div>
                                <!-- Tool: absolute joint values (no EE / IK) -->
                                <div
                                    v-else-if="isToolRobot(robot) && Array.isArray(blockForm.tool_positions[robot.id])"
                                    class="border-rounded q-pa-md"
                                    style="border: 1px solid #FF8F00; background: #1a1a1a;"
                                >
                                    <div class="text-caption text-grey-5 q-mb-sm">
                                        {{ $t('plannerToolAbsoluteHint') }}
                                    </div>
                                    <div class="row q-col-gutter-sm">
                                        <div
                                            v-for="(jointName, jIdx) in (robot.joint_names || [`j${0}`])"
                                            :key="jIdx"
                                            class="col-4"
                                        >
                                            <q-input
                                                dense outlined dark bg-color="dark"
                                                hide-bottom-space
                                                v-model.number="blockForm.tool_positions[robot.id][jIdx]"
                                                :label="jointName"
                                                type="number"
                                                step="0.01"
                                            ></q-input>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </template>

                    <!-- Replay Episode Fields -->
                    <template v-if="blockForm.type === 'replay_episode'">
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockReplayEpisode')" />
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
                            v-model="blockForm.dataset_id"
                            :options="datasetOptions"
                            :label="$t('plannerReplayDatasetLabel')"
                            class="q-mb-md"
                            map-options
                            emit-value
                            :no-options-label="$t('plannerReplayNoDatasets')"
                        ></q-select>
                        <q-select
                            v-if="blockForm.dataset_id != null"
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.episode_index"
                            :options="episodeOptions"
                            :label="$t('plannerReplayEpisodeLabel')"
                            class="q-mb-md"
                            map-options
                            emit-value
                            :no-options-label="$t('plannerReplayNoEpisodes')"
                        ></q-select>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.hz"
                            :label="$t('plannerHzLabel')"
                            :hint="$t('plannerReplayHzHint')"
                            type="number"
                            min="1"
                            step="1"
                            class="q-mb-md"
                        ></q-input>
                        <q-toggle
                            v-model="blockForm.move_to_first"
                            :label="$t('plannerReplayMoveToFirst')"
                            color="primary"
                            dark
                            class="q-mb-xs"
                        ></q-toggle>
                        <div class="text-caption text-grey q-mb-md">
                            {{ $t('plannerReplayMoveToFirstHint') }}
                        </div>
                        <q-input
                            v-if="blockForm.move_to_first"
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.settle_sec"
                            :label="$t('plannerReplaySettleSec')"
                            :hint="$t('plannerReplaySettleSecHint')"
                            type="number"
                            min="0"
                            step="0.5"
                            class="q-mb-md"
                        ></q-input>
                    </template>

                    <!-- Checkpoint Fields -->
                    <template v-if="blockForm.type === 'checkpoint'">
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockCheckpoint')" />
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
                        <div v-if="blockForm.workspace_id" class="q-mb-md">
                            <div class="text-caption text-grey-5 q-mb-xs">{{ $t('plannerCheckpointLabel') }}</div>
                            <q-btn
                                outline
                                no-caps
                                color="primary"
                                class="full-width"
                                icon="account_tree"
                                :label="blockForm.checkpoint_id ? `#${blockForm.checkpoint_id}` : $t('plannerSelectCheckpoint')"
                                @click="showCheckpointPicker = true"
                            />
                        </div>
                        <q-toggle
                            v-model="blockForm.move_homepose"
                            :label="$t('plannerMoveHomepose')"
                            color="primary"
                            dark
                            class="q-mb-xs"
                        ></q-toggle>
                        <div v-if="blockForm.move_homepose" class="row q-gutter-x-sm q-mb-md">
                            <q-input
                                dense outlined dark bg-color="dark"
                                v-model.number="blockForm.move_homepose_duration"
                                :label="$t('homeposeDurationSec')"
                                type="number"
                                step="0.5"
                                min="0.1"
                                style="width: 130px"
                            />
                            <q-input
                                dense outlined dark bg-color="dark"
                                v-model.number="blockForm.move_homepose_settle_sec"
                                :label="$t('homeposeSettleSec')"
                                type="number"
                                step="0.5"
                                min="0"
                                style="width: 130px"
                            />
                        </div>
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
                            v-if="!blockForm.until_done"
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            :label="$t('plannerDurationSeconds')"
                            type="number"
                            class="q-mb-md"
                        ></q-input>
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
                            class="q-mb-md"
                        ></q-input>

                        <!-- 실패 설정: step 한도 + fallback 블록 -->
                        <q-separator dark class="q-my-md" />
                        <div class="text-subtitle2 text-white q-mb-xs">{{ $t('plannerFailureSettings') }}</div>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.max_steps"
                            :label="$t('plannerMaxSteps')"
                            :hint="$t('plannerMaxStepsHint')"
                            type="number"
                            min="1"
                            step="1"
                            clearable
                            class="q-mb-sm"
                        ></q-input>
                        <div class="text-caption text-grey q-mb-sm">{{ $t('plannerFallbackHint') }}</div>
                        <q-select
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.fallback_block_id"
                            :options="fallbackBlockOptions"
                            :label="$t('plannerFallbackBlock')"
                            emit-value
                            map-options
                            clearable
                        ></q-select>
                    </template>

                    <!-- Timesleep Fields -->
                    <template v-if="blockForm.type === 'timesleep'">
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockTimesleep')" />
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            :label="$t('plannerDurationSeconds')"
                            type="number"
                        ></q-input>
                    </template>

                    <!-- Sync Fields -->
                    <template v-if="blockForm.type === 'sync'">
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockSync')" />
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.sync_id"
                            :label="$t('plannerSyncId')"
                            :hint="$t('plannerSyncIdHint')"
                            class="q-mb-md"
                        ></q-input>
                    </template>

                    <!-- Query Pose Fields -->
                    <template v-if="blockForm.type === 'query_pose'">
                        <TutorialHint class="q-mb-md" :text="$t('tutorialPlannerBlockQueryPose')" />
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
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.pose_type"
                            :options="poseTypeOptions"
                            :label="$t('plannerPoseTypeLabel')"
                            :hint="$t('plannerPoseTypeHint')"
                            class="q-mb-md"
                            map-options
                            emit-value
                        ></q-select>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model="blockForm.service_name"
                            :label="$t('plannerServiceNameLabel')"
                            :hint="$t('plannerServiceNameHint')"
                            class="q-mb-md"
                        ></q-input>
                        <div class="q-mb-md">
                            <q-btn
                                outline dense
                                color="primary"
                                icon="terminal"
                                :label="$t('plannerExampleNodeBtn')"
                                @click="showExampleNode = true"
                                size="sm"
                            />
                        </div>
                        <q-input
                            dense outlined dark bg-color="dark"
                            v-model.number="blockForm.duration"
                            :label="$t('plannerDurationSeconds')"
                            type="number"
                            min="0.1"
                            step="0.1"
                            class="q-mb-md"
                        ></q-input>
                        <div class="q-mb-md">
                            <q-btn
                                outline dense
                                color="indigo-4"
                                icon="play_circle"
                                :label="$t('plannerTestQueryPoseBtn')"
                                :loading="testingQueryPose"
                                :disable="!blockForm.service_name"
                                @click="testQueryPose"
                                size="sm"
                            />
                            <div
                                v-if="queryPoseTestResult"
                                class="q-mt-sm q-pa-sm"
                                style="background: #0c1116; border: 1px solid rgba(255,255,255,0.08); border-radius: 6px;"
                            >
                                <div
                                    :class="queryPoseTestResult.ok ? 'text-positive' : 'text-negative'"
                                    class="text-caption q-mb-xs"
                                >
                                    <q-icon
                                        :name="queryPoseTestResult.ok ? 'check_circle' : 'error'"
                                        size="xs"
                                        class="q-mr-xs"
                                    />
                                    {{ queryPoseTestResult.message }}
                                </div>
                                <pre
                                    v-if="queryPoseTestResult.pose"
                                    style="margin: 0; color: #d6deeb; font-family: 'JetBrains Mono', 'Fira Code', Menlo, Consolas, monospace; font-size: 12px; line-height: 1.55; white-space: pre; overflow-x: auto; max-height: 480px;"
                                >{{ JSON.stringify(queryPoseTestResult.pose, null, 2) }}</pre>
                            </div>
                        </div>
                    </template>
                </q-card-section>

                <q-card-actions align="right">
                    <q-btn flat :label="$t('cancel')" color="grey" @click="showBlockForm = false" />
                    <q-btn flat :label="$t('save')" color="primary" @click="saveBlock" />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <!-- 체크포인트 선택 — git-graph 다이얼로그에서 노드 클릭으로 선택 -->
        <q-dialog v-model="showCheckpointPicker">
            <q-card class="bg-secondary border-rounded border-white" dark style="width: 920px; max-width: 95vw;">
                <q-card-section class="bg-dark text-white row items-center">
                    <div class="text-h6">{{ $t('plannerCheckpointLabel') }}</div>
                    <q-space />
                    <q-btn dense round flat icon="close" v-close-popup />
                </q-card-section>
                <q-separator />
                <q-card-section class="bg-secondary">
                    <CheckpointGraph
                        :checkpoints="filteredCheckpoints"
                        :selected-id="blockForm.checkpoint_id"
                        height="440px"
                        @node-click="onPickerNodeClick"
                    />
                </q-card-section>
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
import PlannerBlockCard from 'src/components/v2/PlannerBlockCard.vue';
import CheckpointGraph from 'src/components/v2/CheckpointGraph.vue';
import RobotPendant from 'src/components/v2/RobotPendant.vue';
import TutorialHint from 'src/components/v2/TutorialHint.vue';
import { Notify, Loading } from 'quasar';
import { useSensor } from '../../composables/useSensor';
import { useRobot } from 'src/composables/useRobot';
import { useSocket } from 'src/composables/useSocket';
import { useTutorialStore } from 'src/stores/tutorialStore';
import { enumerateViews } from 'src/utils/sensorView';

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

// 다이얼로그 / 멀티셀렉트 옵션 용 — light 만 (드롭다운 label/value 만 필요).
// 센서/로봇 핸들러 부착은 ``ensureWorkspaceDetails`` 가 실제로 plan 에 포함된
// (selected) workspace 들에 대해서만 lazy 로 수행.
function listAvailableWorkspaces() {
    return api.get('/tasks').then((response) => {
        const tasks = response.data?.tasks || [];
        // 이전 detail 이 이미 있는 workspace 는 그 detail 유지 (handlers 보존).
        const prev = new Map(availableWorkspaces.value.map(w => [w.id, w]));
        availableWorkspaces.value = tasks.map(t => {
            const old = prev.get(t.id);
            return (old && old.sensors) ? { ...t, ...old } : t;
        });
    }).catch((error) => {
        console.error('Error fetching available workspaces:', error);
    });
}

// 주어진 task id 들의 detail 을 lazy 로 병렬 fetch + sensor/robot 핸들러 부착.
// 이미 detail 이 있는 (sensors 채워진) 항목은 skip — 같은 세션에서 한 번만.
async function ensureWorkspaceDetails(ids) {
    const need = (ids || []).filter(id => {
        const w = availableWorkspaces.value.find(x => x.id === id);
        return w && !w.sensors;
    });
    if (!need.length) return;
    detailsLoading.value = true;
    let results;
    try {
        results = await Promise.all(
            need.map(id => api.get(`/tasks/${id}`).then(r => r.data?.task).catch(() => null))
        );
    } finally {
        detailsLoading.value = false;
    }
    results.forEach((detail) => {
        if (!detail) return;
        const i = availableWorkspaces.value.findIndex(w => w.id === detail.id);
        if (i < 0) return;
        // 먼저 reactive 배열에 넣어 proxy 로 만든 뒤, 그 proxy 객체에 핸들러를
        // 붙인다. useSensor/useRobot 은 전달된 객체의 status 를 직접 갱신하는데,
        // raw 객체(배열에 넣기 전)에 붙이면 그 mutation 을 Vue 반응성이 추적하지
        // 못해(템플릿은 proxy 를 읽음) 재생 버튼을 눌러도 로딩/이미지 등 UI 가
        // 바뀌지 않는다. (Workspace 는 selectedWorkspace.value.sensors=proxy 에
        // 직접 useSensor 를 호출해 문제가 없었다.)
        availableWorkspaces.value[i] = { ...availableWorkspaces.value[i], ...detail };
        const ws = availableWorkspaces.value[i];
        (ws.sensors || []).forEach(sensor => {
            sensor.handler = useSensor(sensor);
            if (sensor.type === 'custom') sensor.handler.checkSensorTopic();
        });
        (ws.assembly?.robots || []).forEach(robot => {
            robot.handler = useRobot(robot);
            if (robot.type === 'custom') robot.handler.checkRobotTopic();
        });
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
    const ids = selectedPlanner.value.task_ids || [];
    selectedWorkspaceIds.value = ids;
    // plan 에 실제로 포함된 workspace 들의 풀 detail 만 lazy fetch — 드롭다운에만
    // 나타나는 workspace 는 light 로 충분.
    ensureWorkspaceDetails(ids);
}

function removeWorkspace(workspaceId) {
    const newIds = selectedWorkspaceIds.value.filter(id => id !== workspaceId);
    updatePlanner({ workspace_ids: newIds });
}

// --- Monitoring Window ---
const focused = ref({});
// 워크스페이스 detail(센서/로봇) 을 fetch 하는 동안 true — MonitoringWindow 가
// 빈 상태 메시지 대신 스피너를 표시하도록.
const detailsLoading = ref(false);
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

// 모니터에 보여줄 view 들. 핵심: viewport 는 항상 mount 상태 유지 (MonitoringWindow
// 가 v-show 로만 가린다) — WebRTC PC 가 destroy/create 를 반복하지 않게.
//   - 실행 중인 ``checkpoint`` 블록이 있으면 그 체크포인트가 학습된 view 들만
//     (``task.sensor_ids`` → ``enumerateViews`` 로 view_key 매핑).
//   - 그 외엔 'primary' — 물리 센서 당 첫 viewport 만 노출 (기본 센서뷰).
// 체크포인트 블록은 항상 특정 workspace 에 묶여 실행되므로 매칭 키를
// ``${workspace_id}-${viewKey}`` (= vp.key) 로 잡아 그 workspace 의 해당 view 만
// 노출. 같은 viewKey 가 다른 workspace 의 viewport 에도 있어도 영향 없음.
const monitorVisibleViewKeys = computed(() => {
    const out = new Set();
    let anyRunningCheckpoint = false;
    for (const groupId of Object.keys(runningByGroup)) {
        const blockIdx = runningByGroup[groupId];
        if (blockIdx == null) continue;
        const group = plans.value.find(g => g.id === groupId);
        if (!group) continue;
        const block = group.blocks?.[blockIdx];
        if (!block || block.type !== 'checkpoint') continue;
        const cp = checkpoints.value.find(c => c.id === block.checkpoint_id);
        if (!cp) continue;
        // /api/checkpoints 응답이 axios 경로에선 ``task`` 객체를 생략하고
        // ``task_id`` 만 보낼 때가 있어, availableWorkspaces 에서 task 를 다시
        // 조회해 sensor_ids 를 얻는다.
        let cpSids = cp.task?.sensor_ids;
        if (!cpSids || cpSids.length === 0) {
            const task = availableWorkspaces.value.find(w => w.id === cp.task_id);
            cpSids = task?.sensor_ids || [];
        }
        if (!cpSids.length) continue;
        anyRunningCheckpoint = true;
        const wsId = block.workspace_id;
        enumerateViews(cpSids).forEach(({ viewKey }) => {
            out.add(`${wsId}-${viewKey}`);
        });
    }
    if (!anyRunningCheckpoint) return 'primary';
    return out;
});

function toggleSensor(sensor) {
    sensor.process_id = `sensor_${sensor.id}`;
    if (sensor.status === 'on') {
        sensor.handler.stopSensor();
    } else if (sensor.status === 'loading') {
        // 로딩 중 토글 = 진행 중인 시작 취소.
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
    } else if (robot.status === 'loading') {
        // 로딩 중 토글 = 진행 중인 시작 취소.
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

const poseTypeOptions = computed(() => ([
    { label: t('plannerPoseTypeJoint'), value: 'joint_position' },
    { label: t('plannerPoseTypeEE'), value: 'end_effector_position' },
]));

// --- Query Pose: external node example modal ---
const showExampleNode = ref(false);
const exampleTab = ref('joint');

// --- Query Pose: in-form service test ---
const testingQueryPose = ref(false);
const queryPoseTestResult = ref(null);

async function testQueryPose() {
    const serviceName = (blockForm.value.service_name || '').trim();
    if (!serviceName) return;
    testingQueryPose.value = true;
    queryPoseTestResult.value = null;
    try {
        const { data } = await api.post('/planner/:test_query_pose', {
            service_name: serviceName,
            pose_type: blockForm.value.pose_type,
            workspace_id: blockForm.value.workspace_id,
            duration: blockForm.value.duration,
        });
        queryPoseTestResult.value = {
            ok: true,
            message: data.message || t('plannerTestQueryPoseOk'),
            pose: data.pose || null,
        };
    } catch (err) {
        queryPoseTestResult.value = {
            ok: false,
            message: err?.response?.data?.message || err.message || t('plannerTestQueryPoseFailed'),
            pose: null,
        };
    } finally {
        testingQueryPose.value = false;
    }
}

const EXAMPLE_NODE_JOINT = `# my_target_server.py
# Run: python3 my_target_server.py
#
# ROS2 service that returns target joint configurations when called by
# the EasyTrainer planner's "Query Pose" block (joint mode).
# Uses only the stock std_srvs/srv/Trigger — no custom message build needed.
#
# A workspace is an ASSEMBLY (a combination of robots, e.g. arm + tool),
# so 'positions' is keyed by assembly slot — not a flat list.
# Slots: left_arm / right_arm / left_tool / right_tool / mobile_base
# Only the slots you include get moved.

import json
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MyJointTargetServer(Node):
    def __init__(self):
        super().__init__('my_joint_target_server')
        self.create_service(Trigger, '/my_target', self.handle)
        self.get_logger().info('service ready at /my_target')

    def handle(self, request, response):
        # Compute the targets at request time — vision, planner, DB, etc.
        # Each slot value must include EVERY joint of that robot (gripper
        # included); a count mismatch makes the robot stay still.
        payload = {
            'positions': {
                'left_arm': [0.0, 0.5, -0.5, 0.0, 0.5, 0.0, 0.3],
                # 'left_tool': [0.4],   # add if your assembly has a tool slot
            }
        }

        response.success = True
        response.message = json.dumps(payload)
        return response


def main():
    rclpy.init()
    rclpy.spin(MyJointTargetServer())


if __name__ == '__main__':
    main()
`;

const EXAMPLE_NODE_EE = `# my_target_server.py
# Run: python3 my_target_server.py
#
# ROS2 service that returns target end-effector poses when called by the
# EasyTrainer planner's "Query Pose" block (end-effector mode).
#
# A workspace is an ASSEMBLY (a combination of robots, e.g. arm + tool),
# so 'poses' is keyed by assembly slot — not a single pose.
# Slots: left_arm / right_arm / left_tool / right_tool / mobile_base
# Only the slots you include get moved.

import json
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MyEETargetServer(Node):
    def __init__(self):
        super().__init__('my_ee_target_server')
        self.create_service(Trigger, '/my_target', self.handle)
        self.get_logger().info('service ready at /my_target')

    def handle(self, request, response):
        # Compute the target EE poses at request time.
        payload = {
            'poses': {
                'left_arm': {
                    'position':    [0.30, 0.00, 0.25],  # x, y, z (m, base frame)
                    'orientation': [0.0,  1.57, 0.0],   # rx, ry, rz (Euler, rad)
                    'gripper':     0.5,                 # driver-specific unit
                },
            }
        }

        response.success = True
        response.message = json.dumps(payload)
        return response


def main():
    rclpy.init()
    rclpy.spin(MyEETargetServer())


if __name__ == '__main__':
    main()
`;

const exampleNodeCode = computed(() =>
    exampleTab.value === 'ee' ? EXAMPLE_NODE_EE : EXAMPLE_NODE_JOINT
);

function copyExampleNode() {
    const text = exampleNodeCode.value;
    const finish = () => Notify.create({ color: 'positive', message: t('plannerExampleCopied'), timeout: 1500 });
    if (navigator.clipboard?.writeText) {
        navigator.clipboard.writeText(text).then(finish).catch(() => {
            Notify.create({ color: 'negative', message: t('plannerExampleCopyFailed') });
        });
    } else {
        // Fallback for non-secure contexts.
        const ta = document.createElement('textarea');
        ta.value = text;
        ta.style.position = 'fixed';
        ta.style.opacity = '0';
        document.body.appendChild(ta);
        ta.select();
        try { document.execCommand('copy'); finish(); }
        catch { Notify.create({ color: 'negative', message: t('plannerExampleCopyFailed') }); }
        document.body.removeChild(ta);
    }
}

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

// 카드 색/아이콘은 PlannerBlockCard 가 자체 보유 — 여기 BLOCK_TYPE_COLORS 와
// blockTypeColor 는 컴포넌트 추출 후 더 이상 필요하지 않아 삭제.

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
        params: { where: 'status,=,finished', order: 'created_at DESC', light: 1 }
    }).then((response) => {
        checkpoints.value = response.data.checkpoints || [];
    }).catch((error) => {
        console.error('Error fetching checkpoints:', error);
    });
}

// 체크포인트 선택 다이얼로그(그래프). 노드 클릭 시 해당 체크포인트를 블록에 지정하고 닫는다.
const showCheckpointPicker = ref(false);
function onPickerNodeClick(checkpoint) {
    blockForm.value.checkpoint_id = checkpoint.id;
    showCheckpointPicker.value = false;
}

const filteredCheckpoints = computed(() => {
    if (!blockForm.value.workspace_id) return [];
    return checkpoints.value.filter(c => c.task_id === blockForm.value.workspace_id);
});

// --- Datasets (for replay_episode block) ---
// Cache keyed by workspace_id so switching workspaces in the form refetches only
// when needed. Each entry: [{ id, name, episodes: [{ name, index, length }] }, ...].
const datasetsByWorkspace = ref({});

async function fetchDatasetsForWorkspace(workspaceId) {
    if (workspaceId == null) return;
    if (datasetsByWorkspace.value[workspaceId]) return;
    try {
        const { data } = await api.get('/datasets', { params: { task_id: workspaceId } });
        datasetsByWorkspace.value = {
            ...datasetsByWorkspace.value,
            [workspaceId]: data.datasets || [],
        };
    } catch (e) {
        console.error('Error fetching datasets:', e);
    }
}

const filteredDatasets = computed(() => {
    const ws = blockForm.value.workspace_id;
    if (ws == null) return [];
    return datasetsByWorkspace.value[ws] || [];
});

const datasetOptions = computed(() => filteredDatasets.value.map(d => ({
    label: d.name || `Dataset ${d.id}`,
    value: d.id,
})));

const episodeOptions = computed(() => {
    const ds = filteredDatasets.value.find(d => d.id === blockForm.value.dataset_id);
    if (!ds || !Array.isArray(ds.episodes)) return [];
    return ds.episodes.map(ep => ({
        label: `${ep.name} (${ep.length} frames)`,
        value: ep.index,
    }));
});

// (watch 들은 blockForm 정의 이후로 이동했음 — TDZ 회피)

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
    until_done: true,
    done_threshold: 0.5,
    sync_id: '',
    service_name: '',
    pose_type: 'joint_position',
    hz: 10,
    re_inference_steps: 1,
    temporal_ensemble_coeff: 0.01,
    move_homepose: true,
    move_homepose_duration: 5,
    move_homepose_settle_sec: 0,
    positions: {},
    // move_relative_ee:
    //   deltas:         { [arm_robot_id]:  [dx,dy,dz,drx,dry,drz] }
    //   tool_positions: { [tool_robot_id]: [abs joint values, length = joint_names] }
    // Tool agent (role='tool' or no IK) 는 EE 가 없어 절대 joint 값 사용.
    deltas: {},
    tool_positions: {},
    // replay_episode: 데이터셋의 특정 에피소드를 qaction 으로 직접 replay.
    dataset_id: null,
    dataset_name: '',
    episode_index: null,
    move_to_first: true,
    settle_sec: 0,
    fallback_block_id: null,
    // checkpoint 실패 조건: done 신호 없이 이 step 수에 도달하면 fail → fallback.
    max_steps: null,
});

const formGroup = computed(() => plans.value.find(g => g.id === formGroupId.value) || null);

// 체크포인트 실패 시 이동할 fallback 블록 후보 — 같은 그룹의 다른 블록들.
const fallbackBlockOptions = computed(() => {
    const grp = formGroup.value;
    if (!grp) return [];
    return (grp.blocks || [])
        .filter((_, i) => i !== editingBlockIndex.value)
        .map(b => ({ label: b.name || b.type, value: b.id }));
});

const selectedCheckpointHasSucceed = computed(() => {
    const cp = checkpoints.value.find(c => c.id === blockForm.value.checkpoint_id);
    return !!cp?.train_settings?.has_succeed;
});

watch(() => blockForm.value.checkpoint_id, () => {
    if (!selectedCheckpointHasSucceed.value) {
        blockForm.value.until_done = false;
    }
});

watch(() => blockForm.value.pose_type, (val) => {
    exampleTab.value = val === 'end_effector_position' ? 'ee' : 'joint';
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
    queryPoseTestResult.value = null;
    const wsIds = group?.workspace_ids || [];
    // 그룹이 워크스페이스 1개만 가지면 자동 선택 — 다이얼로그에서 드롭다운을 숨김.
    blockForm.value = {
        type: null,
        name: '',
        workspace_id: wsIds.length === 1 ? wsIds[0] : null,
        checkpoint_id: null,
        duration: 5,
        until_done: true,
        done_threshold: 0.5,
        sync_id: '',
        service_name: '',
        pose_type: 'joint_position',
        hz: 10,
        re_inference_steps: 1,
        temporal_ensemble_coeff: 0.01,
        move_homepose: true,
        move_homepose_duration: 5,
        positions: {},
        deltas: {},
        tool_positions: {},
        dataset_id: null,
        dataset_name: '',
        episode_index: null,
        move_to_first: true,
        settle_sec: 0,
        fallback_block_id: null,
        max_steps: null,
    };
    showBlockForm.value = true;
}

function openEditBlockForm(group, block, index) {
    editingBlockIndex.value = index;
    formGroupId.value = group.id;
    queryPoseTestResult.value = null;
    blockForm.value = {
        type: block.type,
        name: block.name || '',
        workspace_id: block.workspace_id || null,
        checkpoint_id: block.checkpoint_id || null,
        duration: block.duration || 5,
        // 기존 block 에 필드가 없으면 기본값(true)으로 채워 호환성 유지.
        until_done: block.until_done === undefined ? true : !!block.until_done,
        done_threshold: typeof block.done_threshold === 'number' ? block.done_threshold : 0.5,
        sync_id: block.sync_id || '',
        service_name: block.service_name || '',
        pose_type: block.pose_type || 'joint_position',
        hz: typeof block.hz === 'number' ? block.hz : 10,
        re_inference_steps: typeof block.re_inference_steps === 'number' ? block.re_inference_steps : 1,
        temporal_ensemble_coeff: typeof block.temporal_ensemble_coeff === 'number' ? block.temporal_ensemble_coeff : 0.01,
        move_homepose: block.move_homepose === undefined ? true : !!block.move_homepose,
        move_homepose_duration: typeof block.move_homepose_duration === 'number' ? block.move_homepose_duration : 5,
        move_homepose_settle_sec: typeof block.move_homepose_settle_sec === 'number' ? block.move_homepose_settle_sec : 0,
        positions: block.positions ? JSON.parse(JSON.stringify(block.positions)) : {},
        deltas: block.deltas ? JSON.parse(JSON.stringify(block.deltas)) : {},
        tool_positions: block.tool_positions ? JSON.parse(JSON.stringify(block.tool_positions)) : {},
        dataset_id: block.dataset_id ?? null,
        dataset_name: block.dataset_name || '',
        episode_index: typeof block.episode_index === 'number' ? block.episode_index : null,
        move_to_first: block.move_to_first === undefined ? true : !!block.move_to_first,
        settle_sec: typeof block.settle_sec === 'number' ? block.settle_sec : 0,
        fallback_block_id: block.fallback_block_id ?? null,
        max_steps: typeof block.max_steps === 'number' ? block.max_steps : null,
    };
    showBlockForm.value = true;
}

// 블록 id 는 plan 전체에 걸쳐 유일하면 됨. 과거에는 ``Date.now() + 랜덤`` 의
// base36 문자열이었는데 사용자 식별성이 낮아 sequential 정수(문자열) 로 변경.
// 기존 데이터의 문자열 id 와도 공존 — parseInt 가 NaN 을 돌려주므로 max 계산에
// 영향 없음. 새 id 만 1, 2, 3, ... 으로 늘어남.
function nextBlockId() {
    let maxN = 0;
    for (const grp of (plans.value || [])) {
        for (const b of (grp.blocks || [])) {
            const n = parseInt(b?.id, 10);
            if (Number.isFinite(n) && n > maxN) maxN = n;
        }
    }
    return String(maxN + 1);
}

function copyBlock(group, block, index) {
    const copied = JSON.parse(JSON.stringify(block));
    copied.id = nextBlockId();
    copied.name = (copied.name || '') + ' (copy)';
    group.blocks.splice(index + 1, 0, copied);
    saveGroupBlocks(group);
}

watch(() => blockForm.value.workspace_id, (newId, oldId) => {
    if (blockForm.value.type !== 'joint_position' || !newId) return;
    // 편집 모드의 초기 로드 (openEditBlockForm 직후 workspace_id 가 null → 값
    // 으로 바뀌어 watch 가 발화) 에는 손대지 않는다. openEditBlockForm 이 이미
    // blockForm.value.positions 를 block.positions 의 deep clone 으로 세팅
    // 해뒀고, 여기서 robot.joint_names 의 length 와 prior.length 가 어긋날 때
    // (저장 당시와 현재 robot 설정이 다를 때) 정상 값이 0 으로 덮여 쓰이는
    // 버그가 있었다. 사용자가 폼 안에서 워크스페이스를 직접 바꾸는 경우
    // (oldId 가 null 이 아닌 다른 값) 에만 새 robot 들에 맞게 슬롯 재구성.
    if (editingBlockIndex.value !== null && oldId === null) return;
    const robots = getWorkspaceRobots(newId);
    const existing = blockForm.value.positions || {};
    const positions = {};
    robots.forEach(robot => {
        const dim = robot.joint_names?.length || robot.joint_dim || 6;
        const prior = existing[robot.id];
        if (Array.isArray(prior) && prior.length === dim) {
            positions[robot.id] = [...prior];
        } else {
            positions[robot.id] = Array(dim).fill(0);
        }
    });
    blockForm.value.positions = positions;
});

// move_relative_ee: workspace 변경 시 arm 별 6-DOF delta 슬롯과 tool 별
// 절대 joint 슬롯을 0 으로 초기화 (편집 모드면 기존 값 유지). tool 은 EE 가
// 없어서 absolute joint 위치를 받는다.
watch(() => blockForm.value.workspace_id, (newId, oldId) => {
    if (blockForm.value.type !== 'move_relative_ee' || !newId) return;
    // joint_position watch 와 동일 — 편집 모드 초기 로드 (null → 값) 에는
    // 손대지 않는다. openEditBlockForm 이 이미 block.deltas / block.tool_positions
    // 의 deep clone 으로 세팅해뒀고, robot 의 joint_names 길이가 저장 당시와
    // 다를 때 정상값이 0 으로 덮여쓰이는 동일 버그 방지.
    if (editingBlockIndex.value !== null && oldId === null) return;
    const robots = getWorkspaceRobots(newId);
    const existingDeltas = blockForm.value.deltas || {};
    const existingTools = blockForm.value.tool_positions || {};
    const deltas = {};
    const toolPositions = {};
    robots.forEach(robot => {
        if (isToolRobot(robot)) {
            const dim = robot.joint_names?.length || robot.joint_dim || 1;
            const prior = existingTools[robot.id];
            if (Array.isArray(prior) && prior.length === dim) {
                toolPositions[robot.id] = [...prior];
            } else {
                toolPositions[robot.id] = Array(dim).fill(0);
            }
        } else {
            const prior = existingDeltas[robot.id];
            if (Array.isArray(prior) && prior.length === 6) {
                deltas[robot.id] = [...prior];
            } else {
                deltas[robot.id] = [0, 0, 0, 0, 0, 0];
            }
        }
    });
    blockForm.value.deltas = deltas;
    blockForm.value.tool_positions = toolPositions;
});

// replay_episode: 워크스페이스 바뀌면 dataset 목록 lazy-load.
watch(() => blockForm.value.workspace_id, (newId) => {
    if (blockForm.value.type !== 'replay_episode' || !newId) return;
    fetchDatasetsForWorkspace(newId);
});

// 폼 타입이 replay_episode 로 바뀌면 그 시점에도 dataset 로드.
watch(() => blockForm.value.type, (newType) => {
    if (newType === 'replay_episode' && blockForm.value.workspace_id != null) {
        fetchDatasetsForWorkspace(blockForm.value.workspace_id);
    }
});

// Tool 판정: role==='tool' 또는 ik_solver 가 없으면 tool. arm 은 6-DOF EE
// 상대 이동, tool 은 절대 joint 위치.
function isToolRobot(robot) {
    if (!robot) return false;
    if (robot.role === 'tool') return true;
    // ik_solver 가 명시적으로 falsy/null 이면 IK 없는 agent → tool 로 간주
    if (Object.prototype.hasOwnProperty.call(robot, 'ik_solver') && !robot.ik_solver) return true;
    return false;
}

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
        id: isEditing ? grp.blocks[editingBlockIndex.value].id : nextBlockId(),
        type: blockForm.value.type,
    };

    config.keys.forEach(key => {
        block[key] = blockForm.value[key];
    });

    if (block.type === 'checkpoint' && block.checkpoint_id) {
        const cp = checkpoints.value.find(c => c.id === block.checkpoint_id);
        block.checkpoint_name = cp?.name || '';
    }

    if (block.type === 'replay_episode' && block.dataset_id != null) {
        const ds = (datasetsByWorkspace.value[block.workspace_id] || []).find(d => d.id === block.dataset_id);
        block.dataset_name = ds?.name || '';
    }

    if (!block.name) {
        if (block.type === 'joint_position') {
            block.name = t('plannerNameAutoMove', { workspace: getWorkspaceName(block.workspace_id) });
        } else if (block.type === 'move_relative_ee') {
            block.name = t('plannerNameAutoMoveRelativeEE', { workspace: getWorkspaceName(block.workspace_id) });
        } else if (block.type === 'checkpoint') {
            block.name = t('plannerNameAutoCheckpoint', { checkpoint: block.checkpoint_name });
        } else if (block.type === 'replay_episode') {
            block.name = t('plannerNameAutoReplayEpisode', {
                dataset: block.dataset_name || block.dataset_id,
                episode: block.episode_index,
            });
        } else if (block.type === 'timesleep') {
            block.name = t('plannerNameAutoSleep', { duration: block.duration });
        } else if (block.type === 'sync') {
            block.name = t('plannerNameAutoSync', { sync_id: block.sync_id });
        } else if (block.type === 'query_pose') {
            block.name = t('plannerNameAutoQueryPose', { service: block.service_name });
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
// blockId -> { step, max_steps } — 체크포인트 블록의 실시간 step 진행도.
// planner_block_progress 이벤트로 갱신, planner_block_end 에서 정리.
const blockProgressById = reactive({});
const groupRunningSet = ref(new Set());  // groupIds currently mid-run

// blockResultIcon / blockResultColor 는 PlannerBlockCard 내부에서 처리.

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

async function exportPlanner() {
    if (!selectedPlannerId.value) return;

    // Suggest a filename from the planner's name; sanitize like the backend does.
    const rawName = (selectedPlanner.value?.name || 'planner').trim();
    const safeName = rawName.replace(/[^A-Za-z0-9_-]/g, '_').slice(0, 60);
    const suggestedName = `planner_${selectedPlannerId.value}_${safeName}.zip`;

    // If the browser supports the File System Access API, let the user pick
    // the save path BEFORE we hit the backend (the picker requires user
    // activation, which would be consumed by an awaited request).
    let fileHandle = null;
    if (typeof window.showSaveFilePicker === 'function') {
        try {
            fileHandle = await window.showSaveFilePicker({
                suggestedName,
                types: [{
                    description: 'Zip archive',
                    accept: { 'application/zip': ['.zip'] },
                }],
            });
        } catch (err) {
            // User cancelled the picker — abort silently.
            if (err && err.name === 'AbortError') return;
            // Any other error: fall back to the anchor-click download path.
            console.warn('showSaveFilePicker failed, falling back:', err);
            fileHandle = null;
        }
    }

    Loading.show({ message: t('plannerExporting') });
    try {
        const res = await api.post(
            `/planner/${selectedPlannerId.value}/:export`,
            null,
            { responseType: 'blob' }
        );

        // Backend may return a JSON error inside a blob; sniff content-type.
        const contentType = res.headers['content-type'] || '';
        if (!contentType.includes('application/zip')) {
            const text = await res.data.text();
            let message = text;
            try {
                message = JSON.parse(text).message || text;
            } catch { /* not JSON */ }
            throw new Error(message);
        }

        const cd = res.headers['content-disposition'] || '';
        const match = cd.match(/filename\*?=(?:UTF-8'')?"?([^";]+)"?/);
        const filename = match
            ? decodeURIComponent(match[1])
            : suggestedName;

        if (fileHandle) {
            // Write the blob through the picker handle — saved exactly where
            // the user chose.
            const writable = await fileHandle.createWritable();
            await writable.write(res.data);
            await writable.close();
            Notify.create({
                color: 'positive',
                message: t('plannerExported', { filename: fileHandle.name || filename }),
                timeout: 4000,
            });
        } else {
            // Fallback: classic anchor-click download (goes to the browser's
            // default downloads folder).
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
                message: t('plannerExported', { filename }),
                timeout: 4000,
            });
        }
    } catch (err) {
        console.error('exportPlanner failed:', err);
        Notify.create({
            color: 'negative',
            message: t('plannerExportFailed', { error: err.message || err }),
            timeout: 6000,
        });
    } finally {
        Loading.hide();
    }
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

function onPlannerBlockProgress(payload) {
    const blockId = payload?.block_id;
    if (!blockId) return;
    blockProgressById[blockId] = {
        step: Number(payload.step) || 0,
        maxSteps: payload.max_steps != null ? Number(payload.max_steps) : null,
    };
}

function onPlannerBlockStart(payload) {
    const gid = payload?.group_id;
    if (!gid) return;
    runningByGroup[gid] = payload?.index ?? null;
    // 새 블록 시작 — 이전 진행도 정리.
    if (payload?.block_id) delete blockProgressById[payload.block_id];
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
        // step 진행 뱃지 정리.
        delete blockProgressById[payload.block_id];
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
    socket.off('planner_block_progress', onPlannerBlockProgress);
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
    socket.on('planner_block_progress', onPlannerBlockProgress);
    socket.on('planner_block_end', onPlannerBlockEnd);
    socket.on('planner_group_end', onPlannerGroupEnd);
    socket.on('planner_run_end', onPlannerRunEnd);
    pageLoading.value = false;
});
</script>
