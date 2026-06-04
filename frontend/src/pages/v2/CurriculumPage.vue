<template>
  <q-page class="q-pt-md q-pr-md full-height column">
    <!-- Header card (PlannerPage-style): mascot + title/desc + planner select -->
    <div class="border-rounded bg-secondary q-pa-md q-mb-md row items-center no-wrap">
      <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl" />
      <div class="col">
        <div class="row items-center q-mb-sm">
          <div class="text-h5 text-primary text-bold">{{ t('currIntroTitle') }}</div>
          <q-select
            v-model="selectedPlannerId"
            :options="planners"
            :label="t('currSelectPlanner')"
            outlined
            dark
            bg-color="dark"
            dense
            style="width: 360px"
            class="q-ml-md"
            emit-value
            map-options
            option-label="name"
            option-value="id"
            @update:model-value="onPlannerChange"
          />
          <q-space />
          <q-chip
            v-if="curriculum"
            :color="curriculum.status === 'running' ? 'green' : 'grey'"
            text-color="white"
            :icon="curriculum.status === 'running' ? 'autorenew' : 'pause'"
          >
            {{ curriculum.status === 'running' ? t('currRunning') : t('currIdle') }}
          </q-chip>
        </div>
        <div class="text-body text-white">{{ t('currIntroBody') }}</div>
        <div class="text-body text-white">{{ t('currIntroBody2') }}</div>
      </div>
    </div>

    <!-- No planner chosen yet -->
    <div
      v-if="!selectedPlannerId"
      class="col q-mb-md border-rounded border-grey text-grey flex-center flex text-h6"
    >
      {{ t('currSelectPlannerHint') }}
    </div>

    <!-- Planner chosen, curriculum loading/auto-creating (transient) -->
    <div
      v-else-if="!curriculum"
      class="col q-mb-md border-rounded border-grey flex-center flex column text-grey"
    >
      <q-spinner-gears size="50px" color="primary" class="q-mb-md" />
      <div class="text-h6">{{ t('currPreparing') }}</div>
    </div>

    <!-- Curriculum exists -->
    <div v-else class="col row q-mb-md no-wrap">
      <!-- Left panel: tabs (Planner blocks / Group policy) -->
      <div class="col-3 bg-secondary q-mr-md border-rounded text-white column">
        <q-tabs v-model="leftTab" dense align="left" class="text-grey-4" active-color="primary" indicator-color="primary">
          <q-tab name="planner" :label="t('currTabPlanner')" />
          <q-tab name="device" :label="t('currTabDevice')" />
          <q-tab name="policy" :label="t('currTabPolicy')" />
        </q-tabs>
        <q-separator dark />
        <q-tab-panels v-model="leftTab" class="bg-secondary col scroll" animated>
          <!-- Planner tab: read-only vertical block list using the shared
               PlannerBlockCard (PlannerPage 와 동일 카드 — running spinner +
               result icon + step progress 뱃지 자동 표시). 부제는 그룹 라벨로
               override 하고, 우측 슬롯에 설정 버튼만 둔다. 블록 편집은 불가. -->
          <q-tab-panel name="planner" class="q-pa-sm">
            <PlannerBlockCard
              v-for="blk in plannerBlocks"
              :key="blk.id"
              :block="blk"
              :block-config="blockConfigs[blk.type]"
              :running="runningBlockId === blk.id"
              :result="blockResultById[blk.id]"
              :progress="blockProgressById[blk.id]"
              :active="runningBlockId === blk.id"
              badges-align="right"
              class="q-mb-xs full-width"
            >
              <template #header-right>
                <q-btn
                  v-if="blockConfigurable(blk)"
                  flat
                  dense
                  round
                  size="xs"
                  icon="settings"
                  @click="openBlockDialog(blk)"
                />
              </template>
              <template #subtitle>
                <!-- 체크포인트 그룹에 속한 블록은 그룹 색의 chip 으로 표시 —
                     같은 그룹의 블록들은 같은 색이라 묶음을 한눈에 본다.
                     소속 그룹이 없으면 (motion 블록 미배정 / 모션이 아닌 블록)
                     기존 텍스트 라벨로 폴백. -->
                <q-chip
                  v-if="blockGroupOf(blk)"
                  :color="blockGroupOf(blk).color || 'grey'"
                  text-color="white"
                  dense
                  size="sm"
                  class="q-ma-none"
                >
                  {{ blockGroupOf(blk).name || ('Group #' + blockGroupOf(blk).id) }}
                </q-chip>
                <span v-else class="text-grey-5">
                  {{ blockConfigurable(blk) ? t('currUnassigned') : blk.type }}
                </span>
              </template>
            </PlannerBlockCard>
          </q-tab-panel>

          <!-- Device tab: planner robots & sensors on/off -->
          <q-tab-panel name="device" class="q-pa-md">
            <div v-if="!deviceSensors.length && !deviceRobots.length" class="text-grey">
              {{ t('currNoDevices') }}
            </div>
            <template v-else>
              <div v-if="deviceRobots.length" class="text-subtitle2 q-mb-xs">{{ t('currDeviceRobots') }}</div>
              <div
                v-for="robot in deviceRobots"
                :key="'r' + robot.id"
                class="q-pa-sm q-px-md q-my-xs border-rounded row items-center"
                :class="robot.status === 'on' ? 'bg-green-10' : (robot.status === 'error' ? 'bg-red-10' : 'bg-grey-9')"
              >
                <div>{{ robot.name }}</div>
                <q-space />
                <q-toggle
                  v-if="robot.type !== 'custom'"
                  :model-value="robot.status === 'on'"
                  dense
                  color="positive"
                  @click="toggleRobot(robot)"
                />
                <div v-else class="text-caption" :class="robot.status === 'on' ? 'text-positive' : 'text-grey'">
                  {{ robot.status === 'on' ? t('topicOn') : t('topicOff') }}
                </div>
                <q-inner-loading :showing="robot.status === 'loading'" />
              </div>

              <div v-if="deviceSensors.length" class="text-subtitle2 q-mt-md q-mb-xs">{{ t('currDeviceSensors') }}</div>
              <div
                v-for="sensor in deviceSensors"
                :key="'s' + sensor.id"
                class="q-pa-sm q-px-md q-my-xs border-rounded row items-center"
                :class="sensor.status === 'on' ? 'bg-green-10' : 'bg-grey-9'"
              >
                <div>{{ sensor.name }}</div>
                <q-space />
                <q-toggle
                  v-if="sensor.type !== 'custom'"
                  :model-value="sensor.status === 'on'"
                  dense
                  color="positive"
                  @click="toggleSensor(sensor)"
                />
                <div v-else class="text-caption" :class="sensor.status === 'on' ? 'text-positive' : 'text-grey'">
                  {{ sensor.status === 'on' ? t('topicOn') : t('topicOff') }}
                </div>
                <q-inner-loading :showing="sensor.status === 'loading'" />
              </div>
            </template>
          </q-tab-panel>

          <!-- Group policy tab -->
          <q-tab-panel name="policy" class="q-pa-md">
            <q-select
              v-model="policyGroupId"
              :options="groupOptions"
              :label="t('currSelectGroup')"
              outlined dark bg-color="dark" dense emit-value map-options
              class="q-mb-md"
            />
            <template v-if="policyGroup">
              <div class="text-subtitle2 q-mb-xs">{{ t('currMission') }}</div>
              <div class="row q-col-gutter-sm">
                <div class="col-6">
                  <q-input v-model.number="policyForm.target_success_count" type="number" :label="t('currTargetSuccess')" outlined dense dark bg-color="dark" />
                </div>
                <div class="col-6">
                  <q-input v-model.number="policyForm.failure_save_prob" type="number" step="0.1" :label="t('currFailureSaveProb')" outlined dense dark bg-color="dark" />
                </div>
              </div>
              <q-btn class="q-mt-md" color="primary" icon="save" :label="t('currSave')" dense @click="savePolicy" />
            </template>
          </q-tab-panel>
        </q-tab-panels>
      </div>

      <!-- Right panel: rollout controls + per-group dashboards -->
      <div class="col bg-secondary border-rounded q-pa-md text-white column scroll">
        <!-- Rollout control bar -->
        <div class="row items-center q-col-gutter-sm q-mb-md">
          <div class="col-12 col-md-5">
            <q-select
              v-model="targetGroupIds"
              :options="groupOptions"
              :label="t('currTargetGroups')"
              outlined dark bg-color="dark" dense multiple emit-value map-options use-chips
            />
          </div>
          <div class="col-6 col-md-2">
            <q-input v-model.number="repeatCount" type="number" :label="t('currRepeat')" outlined dense dark bg-color="dark" />
          </div>
          <div class="col-auto">
            <q-btn color="green" icon="play_arrow" :label="t('currStart')" :disable="isRunning || !targetGroupIds.length || !allDevicesOn" @click="startRollout" />
          </div>
          <div class="col-auto">
            <q-btn color="red" icon="stop" :label="t('currStop')" :disable="!isRunning" @click="stopRollout" />
          </div>
          <div class="col-auto">
            <q-btn outline color="orange" icon="restart_alt" :label="t('currReset')" @click="showResetConfirm = true" />
          </div>
          <div class="col-auto">
            <!-- 강제 업그레이드 — mission target 미달이어도 현재 누적된
                 success + dagger 데이터로 학습 시작. rollout 동작 중엔 비활성화. -->
            <q-btn
              outline color="purple-4"
              icon="upgrade"
              :label="t('currUpgradeNow')"
              :disable="isRunning || upgradeBusy"
              :loading="upgradeBusy"
              @click="upgradeNow"
            />
          </div>
          <!-- 로봇/센서가 모두 켜지지 않으면 시작 불가 — 비활성 버튼은 tooltip 이
               안 뜨므로 사유를 별도 줄로 안내. -->
          <div v-if="!isRunning && !allDevicesOn" class="col-12 row items-center q-gutter-x-xs text-warning text-caption">
            <q-icon name="warning" size="xs" />
            <span>{{ t('currStartNeedsDevices') }}</span>
          </div>
        </div>
        <q-separator dark class="q-mb-md" />

        <!-- Per-group dashboards: 세로로 쌓임. 각 그룹 = 좌(수치) / 우(그래프) 반반. -->
        <q-card
          v-for="dg in dashboard"
          :key="dg.checkpoint_group_id"
          flat
          class="bg-dark text-white border-rounded q-mb-md"
        >
          <q-card-section>
            <div class="row items-center q-mb-sm">
              <q-badge :color="dg.color || 'grey'" class="q-mr-sm" rounded />
              <div class="text-subtitle1">{{ dg.name || ('Group #' + dg.checkpoint_group_id) }}</div>
              <q-chip dense outline text-color="white" class="q-ml-sm">{{ (dg.checkpoint_ids || []).length }} {{ t('currCheckpoints') }}</q-chip>
              <q-chip v-if="dg.status === 'training'" color="amber" text-color="black" dense icon="model_training" class="q-ml-xs">
                {{ t('currTraining') }}
              </q-chip>
              <q-space />
              <q-select
                v-model="dashStage[dg.checkpoint_group_id]"
                :options="stageOptions(dg)"
                :label="t('currStage')"
                outlined dense dark bg-color="grey-10" emit-value map-options
                style="min-width: 120px"
              />
              <q-btn
                outline dense color="orange" icon="restart_alt"
                class="q-ml-sm"
                :label="t('currStageReset')"
                :disable="isRunning || !dashSelectedStage(dg)"
                @click="askStageReset(dg)"
              />
            </div>

            <div class="row q-col-gutter-md">
              <!-- 좌: 수치 대시보드 -->
              <div class="col-12 col-md-6">
                <template v-if="dashSelectedStage(dg)">
                  <div class="row items-center q-col-gutter-sm q-mb-sm">
                    <q-chip color="blue" text-color="white" dense icon="insights">
                      {{ t('currSuccessRate') }}:
                      {{ dashSelectedStage(dg).success_count }}/{{ rateDenom(dashSelectedStage(dg)) }}
                      <span class="q-ml-xs">({{ formatRatio(dashSelectedStage(dg).success_count, rateDenom(dashSelectedStage(dg))) }})</span>
                    </q-chip>
                    <q-space />
                    <q-btn flat dense color="orange" icon="delete_sweep" :label="t('currDiscardFailure')" @click="discardFailureByGroup(dg.checkpoint_group_id)" />
                  </div>
                  <!-- 체크포인트별 수치 — 성공/실패/교정 따로. 승급 기준은 cp별
                       (success_eps + dagger_eps) >= target. -->
                  <q-markup-table dark flat dense class="bg-grey-10">
                    <thead>
                      <tr>
                        <th class="text-left">CP</th>
                        <th>{{ t('currSuccessCount') }}</th>
                        <th>{{ t('currFailureCount') }}</th>
                        <th>{{ t('currCorrectionEps') }}</th>
                        <th>{{ t('currAvgLen') }}</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr v-for="b in (dg.cp_blocks || [])" :key="b.block_id">
                        <td class="text-left">{{ (dg.block_labels && dg.block_labels[b.block_id]) || b.name }}</td>
                        <td>{{ dashSelectedStage(dg).checkpoints[b.block_id]?.success_eps ?? 0 }}</td>
                        <td>{{ dashSelectedStage(dg).checkpoints[b.block_id]?.failure_eps ?? 0 }}</td>
                        <td>{{ dashSelectedStage(dg).checkpoints[b.block_id]?.dagger_eps ?? 0 }}</td>
                        <td>{{ dashSelectedStage(dg).checkpoints[b.block_id]?.avg_success_len ?? 0 }}</td>
                      </tr>
                    </tbody>
                  </q-markup-table>
                </template>
              </div>

              <!-- 우: stage별 성공률 그래프 -->
              <div class="col-12 col-md-6">
                <div class="text-caption q-mb-xs">{{ t('currSuccessRate') }}</div>
                <div style="height: 130px">
                  <StageLineChart :labels="stageLabels(dg)" :values="successRateSeries(dg)" :label="t('currSuccessRate')" color="#42A5F5" />
                </div>
              </div>

              <!-- 둘째 row: 좌(판정 조건) / 우(노이즈 ± 범위). stage 선택 시만 -->
              <template v-if="dashSelectedStage(dg)">
                <div class="col-12 col-md-6">
                  <div class="text-caption q-mb-xs">{{ t('currStageCriteriaTitle') }}</div>
                  <q-markup-table dark flat dense class="bg-grey-10">
                    <thead>
                      <tr>
                        <th class="text-left">CP</th>
                        <th>{{ t('currStageMaxSteps') }}</th>
                        <th>{{ t('currStageThreshold') }}</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr v-for="b in (dg.cp_blocks || [])" :key="`crit-${b.block_id}`">
                        <td class="text-left">{{ (dg.block_labels && dg.block_labels[b.block_id]) || b.name }}</td>
                        <td>{{ criteriaFor(dg, b.block_id).max_steps ?? '—' }}</td>
                        <td>{{ criteriaFor(dg, b.block_id).success_threshold ?? '—' }}</td>
                      </tr>
                    </tbody>
                  </q-markup-table>
                </div>

                <div class="col-12 col-md-6">
                  <div class="text-caption q-mb-xs">{{ t('currStageNoiseTitle') }}</div>
                  <div v-if="!noiseBlocks(dg).length" class="text-grey text-caption q-pa-sm">
                    {{ t('currStageNoiseNone') }}
                  </div>
                  <q-markup-table v-else dark flat dense class="bg-grey-10">
                    <thead>
                      <tr>
                        <th class="text-left">Block</th>
                        <th v-for="axis in NOISE_AXES" :key="`hdr-${axis}`">{{ axis }}</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr v-for="entry in noiseBlocks(dg)" :key="`noise-${entry.blockId}`">
                        <td class="text-left">{{ entry.label }}</td>
                        <td v-for="axis in NOISE_AXES" :key="`cell-${entry.blockId}-${axis}`">
                          {{ formatNoiseRange(entry.spec, axis, dashSelectedStage(dg)?.success_rate || 0) }}
                        </td>
                      </tr>
                    </tbody>
                  </q-markup-table>
                </div>
              </template>
            </div>
          </q-card-section>
        </q-card>
      </div>
    </div>

    <!-- Block settings dialog: checkpoint (group change/split + per-cp settings) | motion (group + noise) -->
    <q-dialog v-model="blockDialog">
      <q-card style="width: 600px; max-width: 95vw" class="bg-secondary text-white">
        <q-card-section class="text-subtitle1 row items-center">
          <q-icon :name="blockIcon(blockForm.type)" class="q-mr-sm" />
          {{ blockForm.name }}
        </q-card-section>
        <q-card-section style="max-height: 66vh; overflow: auto">
          <!-- CHECKPOINT block -->
          <template v-if="blockForm.type === 'checkpoint'">
            <div class="text-caption text-grey-4 q-mb-sm">{{ checkpointName(blockForm.checkpoint_id) }}</div>

            <div v-if="otherGroupOptions.length" class="q-mb-md">
              <div class="text-subtitle2 q-mb-xs">{{ t('currChangeGroup') }}</div>
              <div class="row q-col-gutter-sm items-center">
                <div class="col">
                  <q-select v-model="blockForm.targetGroupId" :options="otherGroupOptions" outlined dense dark bg-color="dark" emit-value map-options />
                </div>
                <div class="col-auto">
                  <q-btn color="primary" :label="t('currApply')" :disable="blockForm.targetGroupId == null" @click="applyChangeGroup" />
                </div>
              </div>
            </div>

            <div v-if="!isAloneInGroup" class="q-mb-md">
              <div class="text-subtitle2 q-mb-xs">{{ t('currSplitGroup') }}</div>
              <div class="row q-col-gutter-sm items-center">
                <div class="col">
                  <q-input v-model="blockForm.splitName" :label="t('currNewGroupName')" outlined dense dark bg-color="dark" />
                </div>
                <div class="col-auto">
                  <q-btn color="primary" :label="t('currApply')" :disable="!blockForm.splitName" @click="applySplit" />
                </div>
              </div>
            </div>

            <q-separator dark class="q-my-md" />
            <div class="text-subtitle2 q-mb-xs">{{ t('currCheckpointSettings') }}</div>
            <q-select v-model="blockForm.base_dataset_ids" :options="blockDatasetOptions" :label="t('currBaseDatasets')" outlined dense dark bg-color="dark" multiple emit-value map-options use-chips class="q-mb-sm" />
            <ServerUrlField v-model="blockForm.serverUrl" class="q-mb-sm" />
            <TrainingSettingsForm :model-value="blockForm.trainingForm" :policy-type="blockPolicyType" />

            <div class="text-subtitle2 q-mt-md q-mb-xs">{{ t('currSuccessCriteria') }}</div>
            <div class="text-caption text-grey-5 q-mb-xs">{{ t('currCriteriaHint') }}</div>
            <div class="row q-col-gutter-sm">
              <div class="col-4"><q-input v-model.number="blockForm.initial_max_steps" type="number" :label="t('currInitialMaxSteps')" outlined dense dark bg-color="dark" /></div>
              <div class="col-4"><q-input v-model.number="blockForm.length_limit_rate" type="number" step="0.1" :label="t('currLengthLimitRate')" outlined dense dark bg-color="dark" /></div>
              <div class="col-4"><q-input v-model.number="blockForm.success_threshold" type="number" step="0.05" :label="t('currThreshold')" outlined dense dark bg-color="dark" /></div>
              <div class="col-4"><q-input v-model.number="blockForm.succeed_done_frames" type="number" min="1" :label="t('succeedDoneFrames')" :hint="t('succeedDoneFramesHint')" outlined dense dark bg-color="dark" /></div>
            </div>

            <!-- 성공 데이터 다운샘플 — 켜면 성공 에피소드를 rate(stride) 로 솎아서 기록. -->
            <div class="row items-center q-col-gutter-sm q-mt-sm">
              <div class="col-auto">
                <q-toggle v-model="blockForm.success_downsample" color="primary" :label="t('currSuccessDownsample')" />
              </div>
              <div class="col">
                <q-input
                  v-model.number="blockForm.success_downsample_rate"
                  type="number" min="1"
                  :disable="!blockForm.success_downsample"
                  :label="t('currSuccessDownsampleRate')"
                  :hint="t('currSuccessDownsampleRateHint')"
                  outlined dense dark bg-color="dark"
                />
              </div>
            </div>
            <q-btn class="q-mt-md" color="primary" icon="save" :label="t('currSave')" @click="saveCheckpointBlock" />
          </template>

          <!-- MOTION block -->
          <template v-else>
            <div class="text-subtitle2 q-mb-xs">{{ t('currAffectsGroup') }}</div>
            <q-select v-model="blockForm.groupId" :options="groupOptions" :label="t('currSelectGroup')" outlined dense dark bg-color="dark" emit-value map-options clearable class="q-mb-sm" />

            <!-- 계산식 설명 -->
            <div class="bg-grey-9 q-pa-sm border-rounded q-mb-md">
              <div class="text-caption">{{ t('currNoiseFormula') }}</div>
              <div class="text-caption text-grey-4">{{ t('currNoiseFormulaHint') }}</div>
            </div>

            <div class="text-subtitle2 q-mb-xs">{{ t('currNoiseRate') }}</div>
            <div class="row q-col-gutter-xs q-mb-sm">
              <div class="col-2" v-for="axis in NOISE_AXES" :key="'r' + axis">
                <q-input v-model.number="blockForm.rate[axis]" type="number" step="0.01" :label="axis" outlined dense dark bg-color="dark" />
              </div>
            </div>
            <div class="text-subtitle2 q-mb-xs">{{ t('currNoiseOffset') }}</div>
            <div class="row q-col-gutter-xs q-mb-md">
              <div class="col-2" v-for="axis in NOISE_AXES" :key="'o' + axis">
                <q-input v-model.number="blockForm.offset[axis]" type="number" step="0.01" :label="axis" outlined dense dark bg-color="dark" />
              </div>
            </div>

            <!-- 계산된 실제 노이즈 미리보기 (성공률 10% 가정) -->
            <div class="text-subtitle2 q-mb-xs">{{ t('currNoisePreview') }}</div>
            <div class="row q-col-gutter-xs">
              <div class="col-2" v-for="axis in NOISE_AXES" :key="'p' + axis">
                <q-input :model-value="noisePreview[axis]" :label="axis" readonly outlined dense dark bg-color="grey-10" />
              </div>
            </div>

            <q-btn class="q-mt-md" color="primary" icon="save" :label="t('currSave')" @click="saveMotionBlock" />
          </template>
        </q-card-section>
        <q-card-actions align="right">
          <q-btn flat :label="t('currClose')" v-close-popup />
        </q-card-actions>
      </q-card>
    </q-dialog>

    <!-- ─── Monitor FAB + Dialog ──────────────────────────────────────────
         실행 중 우측 하단에 떠 있는 모니터 버튼. 클릭하면 MonitoringWindow 가
         담긴 풀스크린 다이얼로그가 열린다. 카메라/로봇 상태 + 진행도 + Stop. -->
    <q-page-sticky
      v-if="isRunning"
      position="bottom-right"
      :offset="[24, 24]"
    >
      <q-btn
        fab
        color="primary"
        icon="monitor"
        :label="t('currMonitor')"
        @click="showMonitor = true"
      />
    </q-page-sticky>

    <!-- Monitor 풀스크린 — q-dialog 대신 custom fixed-full 컨테이너 (v-show)
         로 만든 이유: q-dialog 는 닫힐 때 children 을 destroy 해서 안쪽
         MonitoringWindow 와 그 WebRtcVideo / PeerConnection 까지 다 새로
         만들어지고 ros2 gRPC 새 stream 이 또 생긴다. isRunning 동안 카드
         자체는 mount 유지 + ``v-show`` 로 가시성만 토글 → 같은 viewport
         재사용 + backend 의 `_TopicFrameSource` 가 ros2 worker 한 개만 잡고
         계속 흐른다. -->
    <transition name="slide-up">
    <div
      v-if="isRunning"
      v-show="showMonitor"
      class="fixed-full column bg-secondary text-white"
      style="z-index: 6000"
    >
      <div class="row items-center bg-dark q-pa-md">
        <q-icon name="monitor" class="q-mr-sm" />
        <div class="text-h6">{{ t('currMonitor') }}</div>
        <q-space />
        <q-btn flat round dense icon="close" @click="showMonitor = false" />
      </div>
      <q-separator dark />
      <!-- 현재 실행 중 블록 상세 — PlannerBlockCard 그대로 재사용. 동일 group
           의 다른 블록은 카드에서 미니멀하게, 현재 블록만 우측 절반에 큼지막
           하게. 카메라/로봇 보면서 어떤 블록인지 명확. -->
      <div class="row items-stretch bg-dark q-pa-sm q-gutter-x-md">
        <PlannerBlockCard
          v-if="runningBlock"
          :block="runningBlock"
          :block-config="blockConfigs[runningBlock.type]"
          :running="true"
          :result="null"
          :progress="blockProgressById[runningBlock.id]"
          :workspace-name="monitorWorkspace?.name || ''"
          :active="true"
          width="220px"
          badges-align="right"
        />
        <div v-else class="text-grey-5 q-py-md">
          {{ t('currMonitorWaiting') }}
        </div>
      </div>
      <q-separator dark />
      <div class="col scroll q-pa-md">
        <MonitoringWindow
          ref="monitorWindowRef"
          v-if="monitorWorkspaceForProp"
          class="full-width"
          :workspace="monitorWorkspaceForProp"
          :workspaces="monitorAllWorkspaces"
          :robots="monitorRobots"
          :sensors="monitorAllSensors"
          :datasets="[]"
          :checkpoints="[]"
          v-model:focused="monitorFocused"
          v-model:selected-dataset-id="monitorDatasetId"
          v-model:selected-checkpoint-id="monitorCheckpointId"
          v-model:selected-episode="monitorEpisode"
          :status="correctionMode ? 'inferencing' : 'pending'"
          :visible-view-keys="monitorVisibleViewKeys"
          monitor-only
          :correction-mode="correctionMode"
          @correction-done="onCorrectionDone"
          @correction-throw="onCorrectionThrow"
          @correction-stop="onCorrectionStop"
        />
        <div v-else class="text-grey flex flex-center full-height">
          {{ t('currMonitorWaiting') }}
        </div>
      </div>
      <!-- 에피소드 저장 진행 배너 — _judge_and_store 의 lerobot_append_episode
           가 mp4 인코딩으로 메인 thread 를 ~수십초간 잡고 있는 동안 노출. 안내가
           없으면 "다음 시행이 왜 안 시작되지?" 로 보임. -->
      <q-separator v-if="savingEpisode.saving" dark />
      <div v-if="savingEpisode.saving" class="bg-amber-9 q-pa-md">
        <div class="row items-center q-gutter-x-sm">
          <q-spinner-dots color="white" size="22px" />
          <div class="col text-white">
            <div class="text-body2 text-weight-medium">
              {{ savingEpisode.role === 'failure'
                  ? t('currSavingEpisodeFailure')
                  : savingEpisode.role === 'success'
                    ? t('currSavingEpisodeSuccess')
                    : t('currSavingEpisode') }}<span v-if="savingEpisode.frames"> · {{ savingEpisode.frames }} frames</span>
            </div>
            <div class="text-caption text-amber-1">{{ t('currSavingEpisodeHint') }}</div>
          </div>
        </div>
      </div>
      <!-- Stop / progress bar. 모든 블록 타입에서 노출. 동작은:
             - checkpoint: per-block stop 신호 → max_steps 초과와 동일 처리,
                           교정 dialog 가 자동 노출
             - 그 외 (모션 블록): 즉시 전체 롤아웃 중지 (교정 dialog 없음)
           progress bar 는 step 정보가 있는 checkpoint 일 때만. -->
      <q-separator v-if="runningBlock" dark />
      <div v-if="runningBlock" class="bg-dark q-pa-md">
        <div class="row items-center q-gutter-x-md">
          <div class="col" v-if="runningBlock.type === 'checkpoint'">
            <div class="row items-center q-mb-xs">
              <div class="text-caption text-grey-4">
                {{ runningProgress.step }} / {{ runningProgress.maxSteps || '∞' }}
              </div>
              <q-space />
              <div class="text-caption text-grey-4" v-if="runningProgress.maxSteps">
                {{ Math.round((runningProgress.step / runningProgress.maxSteps) * 100) }}%
              </div>
            </div>
            <q-linear-progress
              rounded
              size="10px"
              :value="runningProgress.maxSteps ? runningProgress.step / runningProgress.maxSteps : 0"
              color="primary"
              track-color="grey-9"
            />
          </div>
          <div v-else class="col text-caption text-grey-4">
            {{ t('currStopNonCpHint') }}
          </div>
          <q-btn
            color="red"
            icon="stop"
            :label="runningBlock.type === 'checkpoint' ? t('currStopBlock') : t('currStopRollout')"
            :loading="stopBlockBusy"
            @click="onStopCurrentBlock"
          />
        </div>
      </div>
    </div>
    </transition>

    <!-- 교정 confirm — checkpoint 실패 (max_steps OR Stop) 시 자동 노출.
         "건너뛰기" 를 눌러도 dialog 가 닫히지 않고 계속 열려 있음 — 사용자가
         실패 정보를 확인하면서 fallback 진행을 모니터링할 수 있도록. 닫기는
         우측 상단 X 또는 "예, 교정 시작" 흐름에서만. -->
    <q-dialog v-model="showCorrectionConfirm" persistent>
      <q-card dark class="bg-secondary text-white" style="min-width: 380px">
        <q-card-section class="row items-center bg-dark">
          <q-icon name="error" color="amber" class="q-mr-sm" />
          <div class="text-h6">{{ t('currCorrectionTitle') }}</div>
          <q-space />
          <q-btn flat round dense icon="close" @click="closeCorrectionConfirm" />
        </q-card-section>
        <q-card-section class="text-body">
          <div>{{ t('currCorrectionAsk') }}</div>
          <div class="text-caption text-grey-4 q-mt-sm" v-if="lastFailure">
            {{ t('currCorrectionFailureSummary', {
              step: lastFailure.final_step,
              max: lastFailure.max_steps,
              reason: lastFailure.reason === 'user_stop' ? t('currStopBlock') : t('currMaxStepsExceeded'),
            }) }}
          </div>
          <!-- 백엔드는 사용자 선택과 무관하게 fallback 으로 이미 점프했음을 명시. -->
          <div class="text-caption text-amber-4 q-mt-md">
            <q-icon name="info" size="xs" class="q-mr-xs" />
            {{ t('currFallbackInProgress') }}
          </div>
        </q-card-section>
        <q-card-actions align="right" class="q-pa-md">
          <q-btn flat :label="t('currStopCurriculumBtn')" color="red-4" icon="stop_circle" @click="stopCurriculumFromFailure" />
          <q-space />
          <q-btn flat :label="t('currCorrectionNo')" color="grey-4" @click="dismissCorrection" />
          <q-btn unelevated color="amber" text-color="dark" :label="t('currCorrectionYes')" @click="startCorrection" />
        </q-card-actions>
      </q-card>
    </q-dialog>

    <!-- 전체초기화 확인 — 커리큘럼 전체 데이터셋 삭제. -->
    <q-dialog v-model="showResetConfirm">
      <q-card dark class="bg-secondary text-white" style="min-width: 380px">
        <q-card-section class="row items-center bg-dark">
          <q-icon name="restart_alt" color="orange" class="q-mr-sm" />
          <div class="text-h6">{{ t('currReset') }}</div>
        </q-card-section>
        <q-card-section class="text-body">
          {{ t('currResetConfirmMsg') }}
        </q-card-section>
        <q-card-actions align="right" class="q-pa-md">
          <q-btn flat :label="t('no')" color="grey-4" v-close-popup />
          <q-btn unelevated color="orange" text-color="dark" :label="t('yes')" v-close-popup @click="resetCurriculum" />
        </q-card-actions>
      </q-card>
    </q-dialog>

    <!-- 스테이지 초기화 확인 — 선택된 stage 의 데이터셋만 비움. -->
    <q-dialog v-model="showStageResetConfirm">
      <q-card dark class="bg-secondary text-white" style="min-width: 380px">
        <q-card-section class="row items-center bg-dark">
          <q-icon name="restart_alt" color="orange" class="q-mr-sm" />
          <div class="text-h6">{{ t('currStageReset') }}</div>
        </q-card-section>
        <q-card-section class="text-body">
          {{ t('currStageResetConfirmMsg') }}
        </q-card-section>
        <q-card-actions align="right" class="q-pa-md">
          <q-btn flat :label="t('no')" color="grey-4" v-close-popup />
          <q-btn unelevated color="orange" text-color="dark" :label="t('yes')" v-close-popup @click="resetStage" />
        </q-card-actions>
      </q-card>
    </q-dialog>

    <!-- 텔레옵 설정 dialog — WorkspacePage 의 DAgger 와 유사. 데이터셋 선택은
         없음 (stage 의 dagger 데이터셋에 자동 저장). -->
    <q-dialog v-model="showCorrectionTeleop" persistent>
      <q-card dark class="bg-secondary text-white" style="min-width: 480px">
        <q-card-section class="bg-dark">
          <div class="text-h6 row items-center">
            <q-icon name="school" color="amber" class="q-mr-sm" />
            {{ t('currCorrectionTeleopTitle') }}
          </div>
        </q-card-section>
        <q-card-section class="column q-gutter-y-md">
          <div class="text-caption text-grey-4">{{ t('currCorrectionTeleopDesc') }}</div>
          <q-input
            v-model="correctionLanguageInstruction"
            dense outlined dark bg-color="dark"
            :label="t('languageInstruction')"
            clearable
          />
          <q-select
            v-model="correctionTeleType"
            dense outlined dark bg-color="dark"
            :options="correctionTeleOptions"
            map-options emit-value
            :label="t('teleoperationType')"
          />
          <q-input
            v-if="correctionTeleType !== 'keyboard'"
            v-model.number="correctionHz"
            dense outlined dark bg-color="dark"
            :label="t('replayHzLabel')"
            type="number"
            :min="1"
          />
        </q-card-section>
        <q-card-actions align="right" class="q-pa-md">
          <q-btn flat :label="t('cancel')" color="grey-4" :disable="correctionBusy" v-close-popup />
          <q-btn
            unelevated
            color="amber"
            text-color="dark"
            icon="school"
            :label="t('currCorrectionStart')"
            :loading="correctionBusy"
            :disable="!lastFailure?.dagger_dataset_id"
            @click="submitCorrection"
          />
        </q-card-actions>
      </q-card>
    </q-dialog>

  </q-page>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted, reactive, watch, nextTick } from 'vue'
import { useI18n } from 'vue-i18n'
import { api } from 'src/boot/axios'
import { Notify } from 'quasar'
import TrainingSettingsForm from 'src/components/v2/TrainingSettingsForm.vue'
import ServerUrlField from 'src/components/v2/ServerUrlField.vue'
import StageLineChart from 'src/components/v2/StageLineChart.vue'
import PlannerBlockCard from 'src/components/v2/PlannerBlockCard.vue'
import MonitoringWindow from 'src/components/v2/MonitoringWindow.vue'
import { enumerateViews } from 'src/utils/sensorView'
import { useSocket } from 'src/composables/useSocket'
import { TRAIN_CONFIGS } from 'src/configs/modelConfigs'
import { useSensor } from 'src/composables/useSensor'
import { useRobot } from 'src/composables/useRobot'

const { t } = useI18n()

const planners = ref([])
const curriculum = ref(null)
const selectedPlannerId = ref(null)

const checkpoints = ref([])
const datasets = ref([])
const policies = ref([])
const plannerBlocks = ref([])
const deviceWorkspaces = ref([])
// /api/planner/block_configs 응답. type → { label, icon, color, keys }.
// PlannerBlockCard 가 아이콘/라벨 폴백에 사용.
const blockConfigs = ref({})
// 실행 중 표시용 runtime 상태 — backend curriculum_rollout 이 planner_run 과
// 동일한 ``planner_block_start/end/progress`` 이벤트를 emit 한다.
const runningBlockId = ref(null)
const blockResultById = reactive({})    // { blockId: 'finished' | 'stopped' | 'error' }
const blockProgressById = reactive({})  // { blockId: { step, maxSteps } }
const { socket } = useSocket()

// _judge_and_store 의 lerobot_append_episode 가 sync 인코딩으로 메인 rollout
// thread 를 잡고 있는 동안 사용자에게 "저장 중" 임을 보여주기 위한 상태.
// 4 cameras × 200 frames 기준 ~30초 가량 소요될 수 있어 안내가 없으면
// "뭐가 멈춘 거지?" 로 보임.
const savingEpisode = ref({ saving: false, role: null, frames: 0 })
function onCurriculumSavingEpisode (payload) {
    savingEpisode.value = {
        saving: !!payload?.saving,
        role: payload?.role || null,
        frames: Number(payload?.frames) || 0,
    }
}
function onPlannerBlockStart (payload) {
    const bid = payload?.block_id
    if (!bid) return
    runningBlockId.value = bid
    // 새 블록 시작 — 이전 진행도/결과 정리.
    delete blockProgressById[bid]
    delete blockResultById[bid]
}
function onPlannerBlockProgress (payload) {
    const bid = payload?.block_id
    if (!bid) return
    blockProgressById[bid] = {
        step: Number(payload.step) || 0,
        maxSteps: payload.max_steps != null ? Number(payload.max_steps) : null,
    }
}
function onPlannerBlockEnd (payload) {
    const bid = payload?.block_id
    if (!bid) return
    if (runningBlockId.value === bid) runningBlockId.value = null
    blockResultById[bid] = payload?.status || 'finished'
    delete blockProgressById[bid]
}

// ── Monitor dialog (실행 중 우측 하단 FAB → 풀스크린 다이얼로그) ───────────
const showMonitor = ref(false)

// 모니터/교정이 열려 있는 동안엔 Space/Enter 가 "포커스된 버튼" 을 우발적으로
// 클릭(activate)하지 못하게 막는다. 키보드 텔레옵으로 Space 를 연타하다가, 직전
// 클릭으로 포커스가 남아 있던 '현재 블록 중단' 버튼이 Space 로 눌려
// ``:stop_current_block`` 이 잘못 발사되던 문제 방지 (그 stale 신호가 다음
// 체크포인트를 0스텝에 죽였음 — 백엔드에서도 블록 시작 시 플래그를 비워 이중 방어).
// capture 단계에서 가로채 버튼의 기본 동작 자체를 취소하고 포커스를 떨군다.
function _guardMonitorKeys (e) {
  if (e.key !== ' ' && e.key !== 'Spacebar' && e.key !== 'Enter') return
  const ae = document.activeElement
  if (ae && ae.tagName === 'BUTTON') {
    e.preventDefault()
    ae.blur()
  }
}
watch(showMonitor, (open) => {
  if (open) window.addEventListener('keydown', _guardMonitorKeys, true)
  else window.removeEventListener('keydown', _guardMonitorKeys, true)
})

const monitorFocused = ref({})
const monitorDatasetId = ref(null)
const monitorCheckpointId = ref(null)
const monitorEpisode = ref({})
const stopBlockBusy = ref(false)

// 현재 실행 중인 block 객체 — id 로 plannerBlocks 에서 찾는다.
const runningBlock = computed(() => {
    if (!runningBlockId.value) return null
    return plannerBlocks.value.find((b) => b.id === runningBlockId.value) || null
})

// 현재 실행 중인 block 의 workspace 객체 — deviceWorkspaces 에서 찾는다.
const monitorWorkspace = computed(() => {
    const blk = runningBlock.value
    if (!blk) return null
    const wsId = blk.workspace_id
    return deviceWorkspaces.value.find((w) => w.id === wsId) || null
})
// **모든 view 를 미리 등록** — 커리큘럼 전체 플래너에 등장하는 모든 workspace
// 를 union 해서 MonitoringWindow 에 한꺼번에 넘긴다. sensorViewports 가 각
// (workspace, view) 쌍마다 WebRtcVideo 인스턴스를 mount 한 상태로 유지 →
// visibleViewKeys 로 가시성만 토글하면 블록 전환 시 PC 재협상/재로딩 없이
// 즉시 영상이 보인다. 첫 mount 시 모든 view 가 한 번 connect 하지만 그 뒤로는
// 추가 connect 없음.
const monitorAllWorkspaces = computed(() => {
    const seen = new Map()  // workspace_id → workspace object
    plannerBlocks.value.forEach((blk) => {
        const wsId = blk?.workspace_id
        if (wsId == null || seen.has(wsId)) return
        const ws = deviceWorkspaces.value.find((w) => w.id === wsId)
        if (ws) seen.set(wsId, ws)
    })
    // 현재 실행 중 워크스페이스가 plannerBlocks 에 없는 edge case (e.g. 상태가
    // 늦게 동기화) 에도 안전하게 fallback.
    const cur = monitorWorkspace.value
    if (cur && !seen.has(cur.id)) seen.set(cur.id, cur)
    return [...seen.values()]
})
// 모든 workspace 에 등장하는 sensor 의 union — sensorViewports 내부에서
// sensorById lookup 이 매칭되도록 필수.
const monitorAllSensors = computed(() => {
    const seen = new Map()
    monitorAllWorkspaces.value.forEach((ws) => {
        (ws.sensors || []).forEach((s) => {
            if (s?.id != null && !seen.has(s.id)) seen.set(s.id, s)
        })
    })
    return [...seen.values()]
})
// MonitoringWindow 의 required ``workspace`` prop 용 fallback — 현재 실행 중
// block 의 workspace 가 우선이지만 블록 전환 사이 잠깐 null 인 순간에도
// Monitor 가 unmount/remount 되지 않도록 첫 항목을 placeholder 로 제공.
const monitorWorkspaceForProp = computed(() =>
    monitorWorkspace.value || monitorAllWorkspaces.value[0] || null
)
const monitorRobots = computed(() => monitorWorkspace.value?.assembly?.robots || [])

// PlannerPage 와 동일 — checkpoint 가 실행 중이면 그 checkpoint 가 학습된 view
// 만 노출, 아니면 'primary' (물리 센서 당 첫 view 만).
const monitorVisibleViewKeys = computed(() => {
    const blk = runningBlock.value
    if (!blk || blk.type !== 'checkpoint') return 'primary'
    // checkpoint 의 task.sensor_ids → enumerateViews → vp.key 매핑
    const cp = checkpoints.value.find((c) => c.id === blk.checkpoint_id)
    if (!cp) return 'primary'
    const sids = cp.task?.sensor_ids
        || (deviceWorkspaces.value.find((w) => w.id === cp.task_id)?.sensor_ids)
        || []
    if (!sids.length) return 'primary'
    const wsId = blk.workspace_id
    const out = new Set()
    enumerateViews(sids).forEach(({ viewKey }) => out.add(`${wsId}-${viewKey}`))
    return out
})

// 현재 실행 중 체크포인트 블록의 progress (Monitor 다이얼로그 footer 에 표시).
const runningProgress = computed(() => {
    const blk = runningBlock.value
    if (!blk) return { step: 0, maxSteps: null }
    const p = blockProgressById[blk.id]
    return {
        step: p?.step || 0,
        maxSteps: p?.maxSteps || blk.max_steps || null,
    }
})

async function onStopCurrentBlock () {
    if (!curriculum.value) return
    const blk = runningBlock.value
    stopBlockBusy.value = true
    try {
        if (blk?.type === 'checkpoint') {
            // checkpoint: per-block stop → 백엔드가 curriculum_checkpoint_failed
            // 이벤트를 emit → 교정 dialog 자동 노출.
            await api.post(`/curriculum/${curriculum.value.id}/:stop_current_block`)
            Notify.create({ type: 'warning', message: t('currStopBlock') })
        } else {
            // 그 외 모션 블록: 교정 dialog 가 의미 없으므로 즉시 전체 롤아웃 중지.
            await api.post(`/curriculum/${curriculum.value.id}/:stop_rollout`)
            isRunning.value = false
            // Monitor 도 같이 닫는다 (FAB 가 v-if=isRunning 으로 사라지지만,
            // 다이얼로그도 명시적으로 정리).
            showMonitor.value = false
            Notify.create({ type: 'warning', message: t('currStopRollout') })
        }
    } catch (e) {
        console.error('stop request failed:', e)
        Notify.create({ type: 'negative', message: e?.response?.data?.message || 'Stop failed' })
    } finally {
        stopBlockBusy.value = false
    }
}

// ── Correction flow ──────────────────────────────────────────────────────
const showCorrectionConfirm = ref(false)
const showCorrectionTeleop = ref(false)
const lastFailure = ref(null)
const correctionLanguageInstruction = ref('')
const correctionTeleType = ref('leader')
const correctionHz = ref(20)
const correctionBusy = ref(false)

const correctionTeleOptions = computed(() => ([
    { label: 'Leader', value: 'leader' },
    { label: 'Keyboard', value: 'keyboard' },
    { label: 'Motion planning', value: 'motion_planning' },
    { label: 'External', value: 'externel' },
    { label: 'Vive only', value: 'vive_only' },
    { label: 'Vive external', value: 'vive_external' },
]))

function onCurriculumCheckpointFailed (payload) {
    if (!payload) return
    lastFailure.value = payload
    // 자동으로 Monitor 다이얼로그도 열어서 사용자가 카메라/로봇 상태 보면서
    // 결정할 수 있게.
    showMonitor.value = true
    showCorrectionConfirm.value = true
}
// 백엔드 롤아웃 재개 신호 전송. action='fallback' 이면 fallback 블록으로 점프,
// 'abort' 면 점프 없이 다음 블록으로 진행.
async function sendResumeSignal (action) {
    const f = lastFailure.value
    if (!curriculum.value || !f?.block_id) return
    try {
        await api.post(`/curriculum/${curriculum.value.id}/:resume_after_failure`, {
            block_id: f.block_id,
            action,
        })
    } catch (e) {
        console.error('resume_after_failure failed:', e)
        Notify.create({ type: 'negative', message: e?.response?.data?.message || 'Resume failed' })
    }
}

// "건너뛰기" — 백엔드에 fallback 점프 신호 → confirm dialog 닫음.
async function dismissCorrection () {
    await sendResumeSignal('fallback')
    showCorrectionConfirm.value = false
    lastFailure.value = null
}
// 헤더 X — 닫기 + abort (fallback 점프 없음, 다음 블록으로 자연 진행).
async function closeCorrectionConfirm () {
    await sendResumeSignal('abort')
    showCorrectionConfirm.value = false
    lastFailure.value = null
}
// "커리큘럼 중지" — 전체 rollout 종료. :stop_rollout 이 task_control['stop']
// 을 set 하면 폴링 루프 (decisions 대기) 가 즉시 빠져나가므로 별도 resume
// 신호는 불필요. Monitor / dialog 도 같이 정리.
async function stopCurriculumFromFailure () {
    showCorrectionConfirm.value = false
    lastFailure.value = null
    try {
        if (curriculum.value) {
            await api.post(`/curriculum/${curriculum.value.id}/:stop_rollout`)
        }
    } catch (e) {
        console.error('stop_rollout from failure dialog failed:', e)
        Notify.create({ type: 'negative', message: e?.response?.data?.message || 'Stop failed' })
    }
    isRunning.value = false
    showMonitor.value = false
}
function startCorrection () {
    showCorrectionConfirm.value = false
    // teleop 설정 다이얼로그 열기. 롤아웃은 백엔드에서 pause 유지 중 (resume 신호
    // 안 보냈으므로). 사용자가 teleop 마치고 record_episode 종료할 때 resume.
    correctionLanguageInstruction.value = monitorWorkspace.value?.name || ''
    showCorrectionTeleop.value = true
}

// record_episode (correction) 가 진행 중일 때 MonitoringWindow 의 바텀
// record 패널을 강제 노출 (monitor-only 와도 호환). progress / Success /
// Complete / Throw / Stop UI 는 워크스페이스 record 와 동일.
const correctionMode = ref(false)
// MonitoringWindow 인스턴스 참조 — 교정 시작 시 ``startCorrectionRecording``
// 메서드를 직접 호출해서 keyboard / succeed listener 등록 + start_collection
// API 호출을 한꺼번에 위임 (직접 API 만 호출하면 listener 가 안 붙어서 키보드
// 텔레옵이 안 먹는다).
const monitorWindowRef = ref(null)
// 사용자가 누른 버튼 (Done / Throw / Stop) 을 기록 — record_episode 종료
// (stop_process emit) 시점에 어떤 resume 신호를 보낼지 결정.
//   'next'      — Done: 데이터 저장 후 다음 (sequential) 블록으로
//   'fallback'  — Throw: 데이터 폐기 후 fallback 블록으로 점프
//   'curriculum_stop' — Stop: 전체 롤아웃 중지 (이미 stop_rollout 호출됨)
let _pendingCorrectionAction = 'fallback'
// record_episode 의 stop_process 이벤트와 매칭하기 위한 dataset id (id 형식:
// `record_episode_{dataset_id}`).
let _pendingCorrectionDatasetId = null
async function submitCorrection () {
    const f = lastFailure.value
    if (!f?.dagger_dataset_id) {
        Notify.create({ type: 'negative', message: 'No DAgger dataset found for this stage' })
        return
    }
    const ws = deviceWorkspaces.value.find((w) => w.id === f.workspace_id)
    if (!ws) {
        Notify.create({ type: 'negative', message: 'Workspace not loaded' })
        return
    }
    correctionBusy.value = true
    try {
        // 실패한 체크포인트 블록의 max_steps 를 record_episode 의 episode_len
        // 으로 override — 백엔드 progress 가 step/max_steps 로 emit 되므로
        // MonitoringWindow 교정 바의 분모/스텝 표시가 그대로 들어맞는다.
        const maxSteps = Number(f.max_steps) || Number(ws.episode_len) || 0
        const taskOverride = { ...ws, episode_len: maxSteps }
        // correctionMode / Monitor 를 먼저 켜야 MonitoringWindow 가 mount 되어
        // ref 가 부착된다. showCorrectionTeleop 는 닫고 자연스럽게 전환.
        _pendingCorrectionAction = 'fallback'   // 사용자가 버튼 안 누르고 외부로
                                                // 종료될 경우의 안전 기본값.
        correctionMode.value = true
        // MonitoringWindow 의 selectedDatasetId 가 교정 dataset 을 가리키도록
        // 동기화 (Done/Throw/Stop 시 dataset API 가 올바른 id 로 호출되게).
        monitorDatasetId.value = f.dagger_dataset_id
        showCorrectionTeleop.value = false
        showMonitor.value = true
        // mount / ref 부착이 끝나도록 한 tick 대기.
        await nextTick()
        if (!monitorWindowRef.value) {
            throw new Error('MonitoringWindow not mounted')
        }
        // MonitoringWindow 에 위임 — start_collection API 호출 + keyboard /
        // succeed listener 등록까지 한 번에 처리. 직접 api.post 하면 listener
        // 가 안 붙어서 키보드 텔레옵이 안 먹는다.
        await monitorWindowRef.value.startCorrectionRecording({
            datasetId: f.dagger_dataset_id,
            teleType: correctionTeleType.value,
            hz: Number(correctionHz.value) || 20,
            languageInstruction: correctionLanguageInstruction.value || '',
            taskOverride,
        })
        _pendingCorrectionDatasetId = f.dagger_dataset_id
        Notify.create({ type: 'positive', message: t('currCorrectionStart') })
    } catch (e) {
        console.error('start_collection failed:', e)
        Notify.create({ type: 'negative', message: e?.response?.data?.message || 'Failed' })
        // rollback — record_episode 가 시작되지 않았으므로 correctionMode 도 꺼야
        // MonitoringWindow 바닥의 record 바가 어색하게 남지 않는다.
        correctionMode.value = false
    } finally {
        correctionBusy.value = false
    }
}

// MonitoringWindow 의 교정 바에서 사용자가 누른 버튼 — 의도만 기록하고 실제
// 재개 신호는 record_episode 의 stop_process 이벤트 도착 시 일괄 처리.
function onCorrectionDone () { _pendingCorrectionAction = 'next' }
function onCorrectionThrow () { _pendingCorrectionAction = 'fallback' }
async function onCorrectionStop () {
    _pendingCorrectionAction = 'curriculum_stop'
    // 즉시 전체 롤아웃 중지 — 폴링 루프 (correction_decisions 대기) 가
    // task_control['stop'] 을 보고 빠져나간다.
    try {
        if (curriculum.value) {
            await api.post(`/curriculum/${curriculum.value.id}/:stop_rollout`)
        }
    } catch (e) {
        console.error('stop_rollout failed:', e)
    }
    isRunning.value = false
}

// stop_process 이벤트 핸들러 — 진행 중인 correction record_episode 가 끝나면
// 사용자가 누른 버튼에 따라 resume 신호 송신 + UI 정리. 백엔드 pm 은 process
// 이름 그대로 emit (id='record_episode', dataset_id suffix 없음).
function onStopProcess (data) {
    if (!_pendingCorrectionDatasetId) return
    if (data?.id !== 'record_episode') return
    const action = _pendingCorrectionAction
    _pendingCorrectionDatasetId = null
    _pendingCorrectionAction = 'fallback'
    correctionMode.value = false
    const cleanup = () => {
        showCorrectionConfirm.value = false
        lastFailure.value = null
    }
    if (action === 'curriculum_stop') {
        // 전체 stop 은 이미 onCorrectionStop 이 :stop_rollout 으로 처리.
        // 폴링 루프가 task_control['stop'] 으로 빠져나가므로 resume 신호 불필요.
        cleanup()
        showMonitor.value = false
    } else {
        // 'next' (Done) 또는 'fallback' (Throw) — 폴링 루프에 결정 전달.
        sendResumeSignal(action).finally(cleanup)
    }
}

// episode_saved / episode_thrown — record_episode 가 outer 루프를 자동으로
// 빠져나가도록 (iter=1) 의도했지만, 안전망으로 명시적으로 stop_collection 도
// 호출. 저장이 끝난 뒤(또는 throw 처리 직후) 호출하므로 데이터 손실 없이
// outer 루프의 다음 iteration 시작을 막아 stop_process 가 곧 emit 된다.
function onEpisodeFinished () {
    if (!correctionMode.value || !_pendingCorrectionDatasetId) return
    api.post(`/dataset/${_pendingCorrectionDatasetId}/:stop_collection`).catch((e) => {
        console.error('[curriculum] stop_collection after episode finished failed:', e)
    })
}

// blockTypeColor / BLOCK_TYPE_COLORS 는 Monitor 헤더 chip 을 PlannerBlockCard
// 로 교체하면서 사용처가 사라져 제거. 카드 자체가 type 색상 stripe 를 그린다.

const targetGroupIds = ref([])
const repeatCount = ref(10)
const isRunning = ref(false)

// 대시보드(우측): /:dashboard 응답 groups[]. dashStage = 그룹별 선택된 stage index.
const dashboard = ref([])
const dashStage = reactive({})

const leftTab = ref('planner')

// 그룹 정책 탭
const policyGroupId = ref(null)
const policyForm = reactive({ target_success_count: 20, failure_save_prob: 0.3 })

// 블록 설정 다이얼로그
const blockDialog = ref(false)
const NOISE_AXES = ['x', 'y', 'z', 'ax', 'ay', 'az']
const MOTION_TYPES = ['joint_position', 'move_relative_ee', 'query_pose']
function emptyAxes () { return { x: 0, y: 0, z: 0, ax: 0, ay: 0, az: 0 } }
const blockForm = reactive({
  type: '', name: '', block_id: null, checkpoint_id: null,
  targetGroupId: null, splitName: '',
  base_dataset_ids: [], serverUrl: '', trainingForm: {},
  initial_max_steps: 600, length_limit_rate: 1.5, success_threshold: 0.5,
  succeed_done_frames: 3,
  success_downsample: false, success_downsample_rate: 2,
  groupId: null, rate: emptyAxes(), offset: emptyAxes(),
})
// 현재 다이얼로그 체크포인트가 속한 워크스페이스의 데이터셋만(에피소드 개수 라벨 포함)
const blockDatasets = ref([])
const blockDatasetOptions = computed(() =>
  blockDatasets.value.map((d) => ({
    label: `${d.name} (${(d.episodes || []).length})`,
    value: d.id,
  })),
)
const blockPolicyType = computed(() => {
  const cp = checkpoints.value.find((c) => c.id === blockForm.checkpoint_id)
  const pol = cp && policies.value.find((p) => p.id === cp.policy_id)
  return pol ? pol.type : ''
})

let statusTimer = null

// ── options / computeds ───────────────────────────────────────────────────────
const groups = computed(() => curriculum.value?.checkpoint_groups || [])
// 디바이스 탭: 플래너 워크스페이스의 로봇/센서(중복 제거)
const deviceRobots = computed(() => {
  const out = []
  deviceWorkspaces.value.forEach((ws) => {
    ;(ws.assembly?.robots || []).forEach((r) => { if (!out.find((x) => x.id === r.id)) out.push(r) })
  })
  return out
})
const deviceSensors = computed(() => {
  const out = []
  deviceWorkspaces.value.forEach((ws) => {
    ;(ws.sensors || []).forEach((s) => { if (!out.find((x) => x.id === s.id)) out.push(s) })
  })
  return out
})
// 롤아웃 시작 가드 — 플래너에 속한 모든 로봇/센서가 'on' 이어야 한다. 하나라도
// off/error/loading 이면 시작 불가(custom 디바이스도 토픽이 살아 있으면 'on').
// 디바이스가 아예 없으면 켤 게 없으므로 통과.
const allDevicesOn = computed(() =>
  deviceRobots.value.every((r) => r.status === 'on') &&
  deviceSensors.value.every((s) => s.status === 'on'),
)
const groupOptions = computed(() =>
  groups.value.map((g) => ({ label: g.name || 'Group #' + g.id, value: g.id })),
)
const policyGroup = computed(() => groups.value.find((g) => g.id === policyGroupId.value) || null)
const otherGroupOptions = computed(() => {
  const cur = groupOfCheckpoint(blockForm.checkpoint_id)
  return groups.value
    .filter((g) => !cur || g.id !== cur.id)
    .map((g) => ({ label: g.name || 'Group #' + g.id, value: g.id }))
})
const isAloneInGroup = computed(() => {
  const g = groupOfCheckpoint(blockForm.checkpoint_id)
  return !g || (g.checkpoint_ids || []).length <= 1
})

// 노이즈 미리보기: 성공률 10%(0.1) 가정 시 ± 최댓값 = 0.1 × |rate| + |offset|.
// 실제 적용 값은 [-max, +max] 사이의 균일 랜덤이며 매 rollout 마다 새로 샘플.
const SAMPLE_SUCCESS_RATE = 0.1
const noisePreview = computed(() => {
  const out = {}
  NOISE_AXES.forEach((a) => {
    const r = Math.abs(Number(blockForm.rate[a]) || 0)
    const o = Math.abs(Number(blockForm.offset[a]) || 0)
    const max = Math.round((SAMPLE_SUCCESS_RATE * r + o) * 1000) / 1000
    out[a] = max ? `±${max}` : '0'
  })
  return out
})

// ── helpers ──────────────────────────────────────────────────────────────────
function currentStage (group) {
  const stages = group.stages || []
  return stages.length ? stages[stages.length - 1] : null
}

function checkpointName (cpId) {
  const cp = checkpoints.value.find((c) => c.id === cpId)
  return cp ? cp.name : 'CP #' + cpId
}

function blockPolicyTypeFor (cpId) {
  const cp = checkpoints.value.find((c) => c.id === cpId)
  const pol = cp && policies.value.find((p) => p.id === cp.policy_id)
  return pol ? pol.type : ''
}

// TrainPage step3 와 동일: TRAIN_CONFIGS.Common + 정책별 설정으로 trainingForm 생성 후 저장값 시드.
function buildTrainingForm (policyType, saved) {
  const common = JSON.parse(JSON.stringify(TRAIN_CONFIGS.Common || {}))
  const specific = policyType && TRAIN_CONFIGS[policyType] ? JSON.parse(JSON.stringify(TRAIN_CONFIGS[policyType])) : {}
  const form = { ...common, ...specific }
  Object.keys(form).forEach((k) => {
    if (saved && saved[k] !== undefined) form[k].value = saved[k]
  })
  return form
}

// TrainPage 와 동일: 서버 URL 미설정 시 백엔드 default-url(로컬 살아있으면 localhost,
// 아니면 공용 원격)을 기본값으로 채운다.
function ensureDefaultServerUrl () {
  api.get('/remote-train/default-url')
    .then((res) => {
      if (!blockForm.serverUrl && res?.data?.default_url) blockForm.serverUrl = res.data.default_url
    })
    .catch(() => {
      if (!blockForm.serverUrl) blockForm.serverUrl = 'http://easytrainer.training_server.com'
    })
}

async function loadBlockDatasets (cpId) {
  const cp = checkpoints.value.find((c) => c.id === cpId)
  const taskId = cp && cp.task_id
  blockDatasets.value = []
  if (taskId == null) return
  const { data } = await api.get('/datasets', { params: { task_id: taskId } })
  // 커리큘럼이 수집한 데이터셋(origin=curriculum)은 base 후보에서 제외 — 수동 데이터셋만.
  blockDatasets.value = (data.datasets || data || []).filter((d) => d.origin !== 'curriculum')
}

// ── block ↔ group coloring/labeling ───────────────────────────────────────────
function groupOfCheckpoint (cpId) {
  return groups.value.find((g) => (g.checkpoint_ids || []).includes(cpId)) || null
}
function groupOfMotionBlock (blockId) {
  return groups.value.find((g) => (g.motion_block_ids || []).includes(blockId)) || null
}
function blockGroupOf (blk) {
  if (blk.type === 'checkpoint') return groupOfCheckpoint(blk.checkpoint_id)
  if (MOTION_TYPES.includes(blk.type)) return groupOfMotionBlock(blk.id)
  return null
}
function blockConfigurable (blk) {
  return blk.type === 'checkpoint' || MOTION_TYPES.includes(blk.type)
}
// blockClass / blockGroupLabel 은 PlannerBlockCard 의 subtitle slot 으로 옮기면서
// 사용처가 사라져 제거. 카드 자체 색상은 컴포넌트가 type 별로 처리.
const BLOCK_ICONS = {
  joint_position: 'precision_manufacturing', move_relative_ee: 'open_with',
  checkpoint: 'psychology', replay_episode: 'replay', timesleep: 'hourglass_empty',
  sync: 'sync', query_pose: 'travel_explore',
}
// blockForm 다이얼로그가 아이콘만 별도로 쓰는 경로 — 컴포넌트 외부에서 직접 사용.
function blockIcon (type) { return BLOCK_ICONS[type] || 'widgets' }

// ── loaders ──────────────────────────────────────────────────────────────────
async function loadPlanners () {
  const { data } = await api.get('/planners')
  planners.value = data.planners || []
}

async function onPlannerChange () {
  curriculum.value = null
  loadPlannerBlocks() // sync, reads from already-loaded planners
  // 커리큘럼을 먼저 로드/생성해 설정 화면을 즉시 띄운다.
  await loadCurriculumForPlanner()
  // 보조 데이터(체크포인트/데이터셋/정책/디바이스 — 일부 느릴 수 있음)는 백그라운드 로드.
  loadCheckpoints()
  loadDatasets()
  loadPolicies()
  loadDeviceWorkspaces()
}

async function loadPolicies () {
  const { data } = await api.get('/policies')
  policies.value = data.policies || data || []
}

async function loadDeviceWorkspaces () {
  const planner = planners.value.find((p) => p.id === selectedPlannerId.value)
  const taskIds = planner?.task_ids || []
  if (!taskIds.length) { deviceWorkspaces.value = []; return }
  // 전체 task 목록을 풀로 받지 않고, 플래너에 포함된 task 만 detail 로 병렬
  // fetch. light /tasks 호출 자체는 생략 — 어차피 taskIds 가 이미 있음.
  const results = await Promise.all(
    taskIds.map((id) => api.get(`/tasks/${id}`).then((r) => r.data?.task).catch(() => null))
  )
  const tasks = results.filter(Boolean)
  // 먼저 reactive 배열에 넣어 proxy 로 만든 뒤, 그 proxy 객체에 핸들러를 붙인다.
  // useSensor/useRobot 은 전달된 객체의 status 를 직접 갱신하는데, raw 객체(배열에
  // 넣기 전)에 붙이면 그 mutation 을 Vue 반응성이 추적하지 못해(템플릿은 proxy 를
  // 읽음) 토글을 눌러 디바이스가 실제로 켜져도 활성화 UI 가 안 바뀐다.
  // (PlannerPage 의 loadWorkspaceDetails 와 동일 패턴.)
  deviceWorkspaces.value = tasks
  deviceWorkspaces.value.forEach((ws) => {
    ;(ws.sensors || []).forEach((s) => {
      s.handler = useSensor(s)
      if (s.type === 'custom') s.handler.checkSensorTopic && s.handler.checkSensorTopic()
    })
    ;(ws.assembly?.robots || []).forEach((r) => {
      r.handler = useRobot(r)
      if (r.type === 'custom') r.handler.checkRobotTopic && r.handler.checkRobotTopic()
    })
  })
}

function toggleSensor (sensor) {
  sensor.process_id = `sensor_${sensor.id}`
  if (sensor.status === 'on') sensor.handler.stopSensor()
  else sensor.handler.startSensor()
}

function toggleRobot (robot) {
  robot.process_id = `robot_${robot.id}`
  if (robot.status === 'on') robot.handler.stopRobot()
  else if (robot.status === 'error') robot.handler.stopRobot().finally(() => robot.handler.startRobot())
  else robot.handler.startRobot()
}

async function loadCurriculumForPlanner () {
  if (!selectedPlannerId.value) {
    curriculum.value = null
    return
  }
  // 커리큘럼은 플래너와 1:1 — 있으면 로드, 없으면 자동 생성 후 바로 설정 화면 진입.
  const { data } = await api.get('/curriculums', { params: { planner_id: selectedPlannerId.value } })
  const list = data.curriculums || []
  if (list.length) {
    await loadCurriculum(list[0].id)
  } else {
    const planner = planners.value.find((p) => p.id === selectedPlannerId.value)
    const res = await api.post('/curriculum', {
      planner_id: selectedPlannerId.value,
      name: planner ? planner.name : null,
    })
    await loadCurriculum(res.data.curriculum_id)
  }
}

async function loadPlannerBlocks () {
  const planner = planners.value.find((p) => p.id === selectedPlannerId.value)
  const plans = planner?.plans || []
  plannerBlocks.value = plans.flatMap((g) => g.blocks || [])
}

async function loadCheckpoints () {
  // light=1: 셀렉트 채우기엔 컬럼만 필요 — 무거운 task/policy 확장 생략(빠름).
  const { data } = await api.get('/checkpoints', {
    params: { where: 'status,=,finished', light: 1 },
  })
  checkpoints.value = data.checkpoints || []
}

async function loadDatasets () {
  const { data } = await api.get('/datasets')
  datasets.value = data.datasets || data || []
}

async function loadCurriculum (id) {
  if (!id) return
  const { data } = await api.get(`/curriculum/${id}`)
  curriculum.value = data.curriculum
  refreshStatus()
  loadDashboard()
}

// ── curriculum CRUD ────────────────────────────────────────────────────────────
const upgradeBusy = ref(false)
// 강제 업그레이드 — 현재 collecting 상태의 모든 그룹에 대해 누적된 success +
// dagger 데이터로 학습 시작 (mission target 미달이어도). 학습 끝나면 평소처럼
// graduate → 다음 stage 로 진입.
async function upgradeNow () {
    if (!curriculum.value) return
    upgradeBusy.value = true
    try {
        const { data } = await api.post(`/curriculum/${curriculum.value.id}/:upgrade_now`)
        const n = (data?.promoted_group_ids || []).length
        if (n > 0) {
            Notify.create({ type: 'positive', message: t('currUpgradeStarted', { n }) })
            await loadCurriculum(curriculum.value.id)
        } else {
            Notify.create({ type: 'warning', message: t('currUpgradeNoGroup') })
        }
    } catch (e) {
        console.error('upgrade_now failed:', e)
        Notify.create({ type: 'negative', message: e?.response?.data?.message || 'Upgrade failed' })
    } finally {
        upgradeBusy.value = false
    }
}

const showResetConfirm = ref(false)
const showStageResetConfirm = ref(false)
const stageResetTarget = ref(null)  // 초기화 대상 stage id

async function resetCurriculum () {
  await api.post(`/curriculum/${curriculum.value.id}/:reset`)
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currResetDone') })
}

// 스테이지 초기화 — 대시보드에서 선택된 stage 만 초기화.
function askStageReset (dg) {
  const stage = dashSelectedStage(dg)
  if (!stage) return
  stageResetTarget.value = stage.id
  showStageResetConfirm.value = true
}

async function resetStage () {
  const stageId = stageResetTarget.value
  if (!stageId) return
  await api.post(`/stage/${stageId}/:reset`)
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currStageResetDone') })
}

// ── group policy 탭 ────────────────────────────────────────────────────────────
watch(policyGroupId, () => {
  const m = (policyGroup.value && policyGroup.value.mission) || {}
  Object.assign(
    policyForm,
    { target_success_count: 20, failure_save_prob: 0.3 },
    m,
  )
})

async function savePolicy () {
  await api.put(`/checkpoint_group/${policyGroupId.value}`, { mission: { ...policyForm } })
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currSaved') })
}

// ── 블록 설정 다이얼로그 ───────────────────────────────────────────────────────
function openBlockDialog (blk) {
  blockForm.type = blk.type
  blockForm.name = blk.name || blk.type
  blockForm.block_id = blk.id
  blockForm.checkpoint_id = blk.checkpoint_id != null ? blk.checkpoint_id : null
  blockForm.targetGroupId = null
  blockForm.splitName = ''
  if (blk.type === 'checkpoint') {
    const g = groupOfCheckpoint(blk.checkpoint_id)
    const cs = (g && g.checkpoint_settings) || {}
    const conf = cs[blk.checkpoint_id] || cs[String(blk.checkpoint_id)] || {}
    blockForm.base_dataset_ids = conf.base_dataset_ids || []
    // TrainPage step3 와 동일하게 정책 타입 기반 trainingForm 구성 + 저장값 시드.
    blockForm.trainingForm = buildTrainingForm(blockPolicyTypeFor(blk.checkpoint_id), conf.train_settings || {})
    blockForm.serverUrl = (conf.train_settings || {}).server_url || ''
    if (!blockForm.serverUrl) ensureDefaultServerUrl()
    // 해당 체크포인트 워크스페이스의 데이터셋만 로드(에피소드 개수 라벨용).
    loadBlockDatasets(blk.checkpoint_id)
    // 판정 조건 정책(체크포인트별): 최초 길이 제한 + 길이 제한 rate + 성공 임계값.
    blockForm.initial_max_steps = conf.initial_max_steps != null ? conf.initial_max_steps : 600
    blockForm.length_limit_rate = conf.length_limit_rate != null ? conf.length_limit_rate : 1.5
    blockForm.success_threshold = conf.success_threshold != null ? conf.success_threshold : 0.5
    blockForm.succeed_done_frames = conf.succeed_done_frames != null ? conf.succeed_done_frames : 3
    blockForm.success_downsample = !!conf.success_downsample
    blockForm.success_downsample_rate = conf.success_downsample_rate != null ? conf.success_downsample_rate : 2
  } else {
    const g = groupOfMotionBlock(blk.id)
    blockForm.groupId = g ? g.id : null
    const bn = (g && (g.block_noise || {})[blk.id]) || {}
    blockForm.rate = { ...emptyAxes(), ...(bn.rate || {}) }
    blockForm.offset = { ...emptyAxes(), ...(bn.offset || {}) }
  }
  blockDialog.value = true
}

async function applyChangeGroup () {
  await api.post(`/curriculum/${curriculum.value.id}/:assign_checkpoint`, {
    checkpoint_id: blockForm.checkpoint_id, group_id: blockForm.targetGroupId,
  })
  blockDialog.value = false
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currSaved') })
}

async function applySplit () {
  await api.post(`/curriculum/${curriculum.value.id}/:split_checkpoint`, {
    checkpoint_id: blockForm.checkpoint_id, name: blockForm.splitName,
  })
  blockDialog.value = false
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currSaved') })
}

async function saveCheckpointBlock () {
  // trainingForm(정책 하이퍼파라미터) + server_url 을 flat train_settings 로 직렬화.
  const train_settings = { server_url: blockForm.serverUrl }
  Object.keys(blockForm.trainingForm).forEach((k) => {
    train_settings[k] = blockForm.trainingForm[k].value
  })
  // 판정 조건 정책(최초 길이 제한 + 길이 제한 rate + 임계값)도 함께 전송.
  // 백엔드가 checkpoint_settings 에 저장하고 현재 stage 의 success_criteria 를 시드한다.
  await api.post(`/curriculum/${curriculum.value.id}/:set_checkpoint_settings`, {
    checkpoint_id: blockForm.checkpoint_id,
    // 판정조건(success_criteria)은 블록 단위로 시드되므로 block_id 동봉.
    block_id: blockForm.block_id,
    base_dataset_ids: blockForm.base_dataset_ids,
    train_settings,
    initial_max_steps: blockForm.initial_max_steps,
    length_limit_rate: blockForm.length_limit_rate,
    success_threshold: blockForm.success_threshold,
    succeed_done_frames: blockForm.succeed_done_frames,
    success_downsample: blockForm.success_downsample,
    success_downsample_rate: blockForm.success_downsample_rate,
  })
  blockDialog.value = false
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currSaved') })
}

async function saveMotionBlock () {
  await api.post(`/curriculum/${curriculum.value.id}/:assign_motion_block`, {
    block_id: blockForm.block_id,
    group_id: blockForm.groupId != null ? blockForm.groupId : null,
    block_noise: { rate: { ...blockForm.rate }, offset: { ...blockForm.offset } },
  })
  blockDialog.value = false
  await loadCurriculum(curriculum.value.id)
  Notify.create({ type: 'positive', message: t('currSaved') })
}

// ── 대시보드 ───────────────────────────────────────────────────────────────────
async function loadDashboard () {
  if (!curriculum.value) return
  try {
    const { data } = await api.get(`/curriculum/${curriculum.value.id}/:dashboard`)
    dashboard.value = data.groups || []
    // 선택된 stage 기본값 = 각 그룹의 마지막(최신) stage.
    dashboard.value.forEach((dg) => {
      if (dashStage[dg.checkpoint_group_id] == null && dg.stages.length) {
        dashStage[dg.checkpoint_group_id] = dg.stages[dg.stages.length - 1].index
      }
    })
  } catch {
    /* ignore */
  }
}

function stageOptions (dg) {
  return (dg.stages || []).map((s) => ({ label: `Stage ${s.index}`, value: s.index }))
}
function dashSelectedStage (dg) {
  const idx = dashStage[dg.checkpoint_group_id]
  return (dg.stages || []).find((s) => s.index === idx) || (dg.stages || [])[dg.stages.length - 1] || null
}
function stageLabels (dg) {
  return (dg.stages || []).map((s) => `S${s.index}`)
}
function successRateSeries (dg) {
  return (dg.stages || []).map((s) => Math.round((s.success_rate || 0) * 100))
}
// 성공률 백분율 — 분모 0 이면 '—' 로 (NaN% 노출 방지).
function formatRatio (success, total) {
  const s = Number(success) || 0
  const t = Number(total) || 0
  if (t === 0) return '—'
  return `${Math.round((s / t) * 100)}%`
}
// 성공률 분모 — 그룹 단위 rollout 횟수. success_count / failure_count 는 한
// rollout 마다 정확히 한 쪽이 +1 (체크포인트 N 개여도 그룹 단위로 1회). 교정은
// 같은 실패 rollout 에 부속된 사용자 시연이므로 분모에 더하지 않는다.
function rateDenom (stage) {
  if (!stage) return 0
  return (Number(stage.success_count) || 0)
       + (Number(stage.failure_count) || 0)
}
// 선택된 stage 의 **블록별** 판정 조건 (success_criteria 의 entry, 키=block_id).
function criteriaFor (dg, blockId) {
  const st = dashSelectedStage(dg)
  const crit = (st && st.success_criteria) || {}
  return crit[blockId] || {}
}

// 이 그룹의 모션 블록 + 그 블록의 노이즈 spec 묶음. 사용자가 등록한 노이즈만
// (block_noise 에 entry 가 있는 블록만) 표시.
function noiseBlocks (dg) {
  const bn = dg.block_noise || {}
  const blockById = new Map(plannerBlocks.value.map((b) => [b.id, b]))
  return Object.entries(bn).map(([blockId, spec]) => {
    const blk = blockById.get(blockId)
    return {
      blockId,
      spec,
      label: blk?.name || `${blk?.type || 'block'} #${blockId}`,
    }
  })
}

// 한 축의 ± 최댓값을 사람-친화적으로: ``±(SR × |rate| + |offset|)``.
function formatNoiseRange (spec, axis, successRate) {
  const r = Math.abs(Number((spec?.rate || {})[axis]) || 0)
  const o = Math.abs(Number((spec?.offset || {})[axis]) || 0)
  const max = Math.round(((Number(successRate) || 0) * r + o) * 1000) / 1000
  return max ? `±${max}` : '0'
}

async function discardFailureByGroup (groupId) {
  const g = (curriculum.value?.checkpoint_groups || []).find((x) => x.id === groupId)
  const st = g && currentStage(g)
  if (!st) return
  await api.post(`/stage/${st.id}/:discard_failure`)
  await loadCurriculum(curriculum.value.id)
  await loadDashboard()
  Notify.create({ type: 'positive', message: t('currDiscardDone') })
}

// ── rollout controls ───────────────────────────────────────────────────────────
async function startRollout () {
  // 가드 — 로봇/센서가 모두 켜져 있지 않으면 시작하지 않는다(버튼 disable 과
  // 동일 조건이지만, 토글로 상태가 바뀌는 사이의 경합 방지용 이중 방어).
  if (!allDevicesOn.value) {
    Notify.create({ type: 'warning', message: t('currStartNeedsDevices') })
    return
  }
  await api.post(`/curriculum/${curriculum.value.id}/:start_rollout`, {
    target_group_ids: targetGroupIds.value,
    repeat_count: repeatCount.value,
  })
  isRunning.value = true
  Notify.create({ type: 'positive', message: t('currStart') })
}

async function stopRollout () {
  await api.post(`/curriculum/${curriculum.value.id}/:stop_rollout`)
  isRunning.value = false
  // Monitor / 교정 dialog 들도 같이 정리 — 롤아웃이 멈추면 FAB / 다이얼로그 모두
  // 사라져야 한다 (v-if=isRunning 으로 컨테이너는 destroy 되지만, 교정 dialog
  // 는 별도라 명시적으로 닫는다).
  showMonitor.value = false
  showCorrectionConfirm.value = false
  showCorrectionTeleop.value = false
}

// 백엔드가 정상 종료 (curriculum_rollout_end) 를 알리면 즉시 UI 정리. 3s 폴링
// (refreshStatus) 보다 반응이 빠르고, 사용자가 stop 안 눌렀더라도 자동 정리.
function onCurriculumRolloutEnd () {
  isRunning.value = false
  showMonitor.value = false
  showCorrectionConfirm.value = false
  showCorrectionTeleop.value = false
}

async function refreshStatus () {
  if (!curriculum.value) return
  try {
    const { data } = await api.get(`/curriculum/${curriculum.value.id}/:rollout_status`)
    isRunning.value = data.is_running
    if (curriculum.value) {
      // patch live counts onto current stages
      ;(data.groups || []).forEach((gs) => {
        const g = (curriculum.value.checkpoint_groups || []).find((x) => x.id === gs.checkpoint_group_id)
        const st = g && currentStage(g)
        if (st) {
          st.success_count = gs.success_count
          st.failure_count = gs.failure_count
          st.correction_count = gs.correction_count
        }
      })
    }
    // 롤아웃 진행 중에는 대시보드 수치도 주기적으로 갱신.
    if (data.is_running) loadDashboard()
  } catch {
    // ignore transient errors
  }
}

async function loadBlockConfigs () {
  try {
    const { data } = await api.get('/planner/block_configs')
    blockConfigs.value = data.block_configs || {}
  } catch (e) {
    console.error('load block_configs failed:', e)
  }
}

onMounted(async () => {
  await Promise.all([loadPlanners(), loadBlockConfigs()])
  statusTimer = setInterval(refreshStatus, 3000)
  // curriculum_rollout 이 emit 하는 block_start/end/progress 를 구독해 카드 상태
  // 업데이트.
  socket.on('planner_block_start', onPlannerBlockStart)
  socket.on('planner_block_progress', onPlannerBlockProgress)
  socket.on('planner_block_end', onPlannerBlockEnd)
  // 체크포인트 실패 → Monitor + Correction confirm 자동 노출.
  socket.on('curriculum_checkpoint_failed', onCurriculumCheckpointFailed)
  // 롤아웃 정상/이상 종료 → FAB + dialog 즉시 정리.
  socket.on('curriculum_rollout_end', onCurriculumRolloutEnd)
  // _judge_and_store 의 sync 인코딩 동안 "저장 중" 배너 노출.
  socket.on('curriculum_saving_episode', onCurriculumSavingEpisode)
  // record_episode (교정) 종료 시 사용자 선택에 따라 resume 신호.
  socket.on('stop_process', onStopProcess)
  // 저장/폐기 직후 명시적으로 stop_collection 호출 — iter=1 만으로 부족한
  // 케이스(payload 누락 등)의 안전망. outer 루프의 다음 iteration 시작을 막아
  // record_episode 가 곧 종료되고 stop_process 가 emit 된다.
  socket.on('episode_saved', onEpisodeFinished)
  socket.on('episode_thrown', onEpisodeFinished)
})

onUnmounted(() => {
  if (statusTimer) clearInterval(statusTimer)
  window.removeEventListener('keydown', _guardMonitorKeys, true)
  socket.off('planner_block_start', onPlannerBlockStart)
  socket.off('planner_block_progress', onPlannerBlockProgress)
  socket.off('planner_block_end', onPlannerBlockEnd)
  socket.off('curriculum_checkpoint_failed', onCurriculumCheckpointFailed)
  socket.off('curriculum_rollout_end', onCurriculumRolloutEnd)
  socket.off('curriculum_saving_episode', onCurriculumSavingEpisode)
  socket.off('stop_process', onStopProcess)
  socket.off('episode_saved', onEpisodeFinished)
  socket.off('episode_thrown', onEpisodeFinished)
})
</script>
