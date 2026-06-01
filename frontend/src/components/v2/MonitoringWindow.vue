<template>
    <div class="column full-height border-rounded" :style="{ maxHeight: monitorOnly ? null : '700px' }">
        <TutorialHint v-if="!monitorOnly" :text="$t(monitoringHintKey)" class="q-mb-sm" />
        <div class="bg-secondary border-rounded column q-px-sm col">
        <div :class="[monitorOnly ? 'col' : 'col-6', 'row flex felx-center q-col-gutter-x-sm']" v-if="sensorViewports.length > 0">
            <!-- v-show 로 숨김 (v-if 아님) — 숨겨도 WebRtcVideo 가 unmount 되지
                 않아 PC/track 이 살아있다. v-if 면 매 토글마다 WebRTC 재협상이
                 일어나 streaming server 부하 + 짧은 black screen 발생. -->
            <div v-for="vp in sensorViewports" :key="vp.key" v-show="isViewportVisible(vp)" class="col q-py-sm relative-position">
                <web-rtc-video
                    :process-id="`sensor_${vp.sensor.id}`"
                    :topic="vp.sensor.read_topic"
                    :msg-type="vp.sensor.read_topic_msg"
                    :sensor-id="vp.sensor.id"
                    class="full-height border-rounded cursor-pointer"
                    :loading="vp.sensor.status !== 'on'"
                    v-if="vp.sensor.status !== 'off'"
                    :class="{
                        'border-primary': focused.viewKey === vp.viewKey && focused.device_type === 'sensor',
                    }"
                    @click="focusSensorRobot(vp.sensor, 'sensor', vp.viewKey)"
                    :resize="viewportImgSize(vp.workspace, vp.viewKey, vp.sensor.id)"
                    :cropped_area="viewportCropArea(vp.workspace, vp.viewKey, vp.sensor.id)"
                    :rotate="viewportRotate(vp.workspace, vp.viewKey, vp.sensor.id)"
                    :overlay-src="status === 'testing' && inferenceVisionMapOn ? inferenceHeatmaps[`sensor_${vp.viewKey}`] : null"
                ></web-rtc-video>
                <div class="full-height border-white bg-dark border-rounded flex flex-center" v-else>
                    <q-btn round flat icon="play_arrow" text-color="white" size="xl" @click="vp.sensor.handler.startSensor(vp.sensor)"></q-btn>
                </div>
                <q-chip color="blue-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px; z-index: 2">
                    {{ vp.sensor.name }}<span v-if="vp.occurrence > 0"> · view {{ vp.occurrence + 1 }}</span> {{ $t('sensorSuffix') }}<span v-if="vp.workspaceName"> · {{ vp.workspaceName }}</span>
                </q-chip>
            </div>
        </div>
        <div v-else :class="[monitorOnly ? 'col' : 'col-6', 'q-py-sm']">
            <div class="text-white border-rounded border-white bg-dark full-height flex flex-center">{{ $t('noSensorsMsg') }}</div>
        </div>
        <div :class="[monitorOnly ? 'col' : 'col-5', 'row q-gutter-x-sm']" v-if="robots.length > 0">
            <div v-for="robot in robots" :key="robot.id" class="col column q-pa-md relative-position border-rounded border-white text-white cursor-pointer"
                    :class="{
                        'border-primary': focused.id === robot.id && focused.device_type === 'robot',
                        'bg-dark': robot.status === 'off',
                    }"
                    @click="focusSensorRobot(robot, 'robot')"
                >
                <div v-for="(j, i) in robot.joint_names" :key="i" class="col flex flex-center q-gutter-x-md">
                    <div class="border-rounded border-white q-px-md q-py-xs text-center">{{ j }} {{ robot.jointState ? robot.jointState[i]?.toFixed(4) : $t('unreadable') }}</div>
                    <q-icon name="arrow_forward"></q-icon>
                    <div class="border-rounded border-primary q-px-md q-py-xs text-center text-primary">{{ j }} {{ robot.jointAction ? robot.jointAction[i]?.toFixed(4) : $t('unreadable') }}</div>
                </div>
                <q-btn
                    class="absolute-center q-mb-md q-mr-md"
                    round
                    flat
                    icon="play_arrow"
                    text-color="white"
                    size="xl"
                    @click="robot.handler.startRobot(robot)"
                    v-if="robot.status === 'off'"
                ></q-btn>
                <q-chip color="green-10" text-color="white" class="absolute-top-left" style="top: 20px; left: 15px">{{ robot.name }} {{ $t('robotSuffix') }}</q-chip>
            </div>
        </div>
        <div v-else :class="[monitorOnly ? 'col' : 'col-5', 'border-rounded border-white bg-dark flex flex-center']">
            <div class="text-white">{{ $t('noRobotsMsg') }}</div>
        </div>
        <template v-if="!monitorOnly || correctionMode">
        <!-- correctionMode 에서는 'startAllDevices' 체크 우회 — 커리큘럼이 이미
             필요한 device 를 켜놓은 상태이고, props.sensors 가 모든 워크스페이스
             의 union 이라 다른 워크스페이스의 sensor 가 off 일 수 있다 (그건
             교정 record 와 무관). -->
        <div class="flex flex-center col" v-if="!isRobotSensorAllOn && !correctionMode">
            <div class="text-yellow">{{ $t('startAllDevices') }}</div>
        </div>
        <div class="col q-py-sm" v-else>
            <div v-if="selectedEpisode.name && selectedDatasetId">
                <div class="row q-mb-sm">
                    <div
                        class="col bg-dark border-rounded text-white row items-center q-col-gutter-x-sm"
                    >
                        <div>
                            <div class="text-h6">{{ selectedEpisode?.name }}</div>
                        </div>
                        <div>
                            <q-btn
                                icon="close"
                                round
                                flat
                                @click="selectedEpisode = {}"
                                v-if="status === 'pending'"
                            ></q-btn>
                        </div>
                        <q-space class="col" v-if="status === 'pending'"></q-space>
                        <div class="col q-px-md" v-else>
                            <q-linear-progress
                                :value="replayProgress"
                                color="primary"
                                track-color="black"
                                size="30px"
                                instant-feedback
                            >
                                <div class="absolute-full flex flex-center">
                                    <q-badge color="white" text-color="dark" :label="$t('replayCapturingProgress', { percent: Number(replayProgress * 100).toFixed(0) })" />
                                </div>
                            </q-linear-progress>
                        </div>
                        <div class="row items-center q-gutter-x-sm" v-if="status === 'pending'">
                            <span class="text-caption text-grey">{{ $t('replayActionType') }}:</span>
                            <q-radio
                                v-model="replayActionType"
                                val="joint"
                                :label="$t('replayActionQaction')"
                                dense dark color="primary"
                            />
                            <q-radio
                                v-model="replayActionType"
                                val="ee_delta"
                                :label="$t('replayActionEeDelta')"
                                dense dark color="primary"
                            />
                            <q-checkbox
                                v-model="replayCapture"
                                :label="$t('replayCaptureLabel')"
                                dark dense
                                :disable="!allSensorsOn"
                            >
                                <q-tooltip v-if="!allSensorsOn">{{ $t('replayCaptureTooltip') }}</q-tooltip>
                            </q-checkbox>
                            <q-select
                                v-if="replayCapture"
                                v-model="replayCaptureDatasetId"
                                dense outlined dark bg-color="dark"
                                :label="$t('replayTargetDataset')"
                                style="min-width: 180px"
                                :options="datasets"
                                option-label="name"
                                option-value="id"
                                map-options emit-value
                            />
                            <q-input
                                v-model.number="replayHz"
                                dense outlined dark bg-color="dark"
                                :label="$t('replayHzLabel')"
                                style="width: 80px"
                                type="number"
                                :min="1"
                            />
                        </div>
                        <div>
                            <q-btn
                                color="red"
                                text-color="white"
                                :label="$t('replayPlay')"
                                icon="play_arrow"
                                @click="startReplay"
                                v-if="status === 'pending'"
                            ></q-btn>
                            <q-btn
                                color="white"
                                text-color="red"
                                :label="$t('replayStop')"
                                icon="stop"
                                @click="stopReplay"
                                v-else
                            ></q-btn>
                        </div>
                    </div>
                </div>
            </div>
            <!--
              inference 상태 바 — checkpoint 가 선택되어 있고 *추론 자체* 가
              pending / 실행 중인 동안만 띄운다. DAgger 흐름은 checkpoint 를
              계속 선택해둔 채로 record_episode 를 띄우는데, 이때 status 는
              'inferencing' 으로 바뀐다 → 이 분기 대신 아래의 데이터 수집
              분기 (v-else) 로 흘러가야 한다.
            -->
            <div v-else-if="checkpoint && (status === 'pending' || status === 'testing')" class="row q-mb-sm">
                <div
                    class="col bg-dark border-rounded text-white row flex flex-center q-col-gutter-x-sm"
                    v-if="status === 'pending'"
                >
                    <div>
                        <div class="text-h6">{{ checkpoint?.name }}</div>
                    </div>
                    <div>
                        <q-btn
                            icon="close"
                            round
                            flat
                            @click="selectedCheckpointId = null"
                        ></q-btn>
                    </div>
                    <q-space class="col"></q-space>
                    <div>
                        <q-btn
                            color="red"
                            text-color="white"
                            :label="$t('startInference')"
                            icon="play_arrow"
                            @click="startInference"
                            v-if="status === 'pending'"
                        />
                    </div>
                </div>
                <div
                    class="col bg-dark border-rounded text-white row flex flex-center"
                    v-else
                >
                    <div
                        class="col q-px-md"
                        v-if="moveHomposeInDataCollection && inferenceProgress.episodeLen > 0"
                    >
                        <q-linear-progress
                            instant-feedback
                            :value="inferenceProgress.progress"
                            size="30px"
                            color="primary"
                            track-color="black"
                        >
                            <div class="absolute-full flex flex-center">
                                <q-badge color="white" text-color="dark"
                                    :label="`${inferenceProgress.step}/${inferenceProgress.episodeLen}`" />
                            </div>
                        </q-linear-progress>
                    </div>
                    <q-space v-else></q-space>
                    <q-badge
                        v-if="succeedScore !== null"
                        :color="succeedScore > 0.7 ? 'green' : 'grey-5'"
                        :label="$t('succeedLabel', { score: succeedScore.toFixed(2) })"
                        class="q-mr-sm q-pa-sm text-bold"
                        outline
                    />
                    <q-badge
                        v-if="oodScore !== null"
                        :color="oodTotal > 1.0 ? 'red' : 'orange'"
                        class="q-mr-sm q-pa-sm text-bold"
                        outline
                    >
                        {{ $t('oodLabel') }}: {{ oodScoreDisplay }}
                    </q-badge>
                    <q-btn-dropdown
                        v-if="supportsInferenceVisionMap"
                        :label="$t('visionMap')"
                        :icon="inferenceVisionMapOn ? 'visibility' : 'visibility_off'"
                        :color="inferenceVisionMapOn ? 'primary' : 'grey-7'"
                        text-color="white"
                        class="q-mr-sm"
                        dense
                        no-caps
                    >
                        <q-list class="bg-dark text-white" style="min-width: 160px;">
                            <q-item clickable v-close-popup @click="setInferenceVisionMap(null)">
                                <q-item-section>
                                    <q-item-label>{{ $t('visionMapOff') }}</q-item-label>
                                </q-item-section>
                                <q-item-section side v-if="!inferenceVisionMapOn">
                                    <q-icon name="check" color="primary" />
                                </q-item-section>
                            </q-item>
                            <q-item
                                v-for="opt in inferenceVisionMapMethodOptions"
                                :key="opt.value"
                                clickable
                                v-close-popup
                                @click="setInferenceVisionMap(opt.value)"
                            >
                                <q-item-section>
                                    <q-item-label>{{ opt.label }}</q-item-label>
                                </q-item-section>
                                <q-item-section side v-if="inferenceVisionMapOn && inferenceVisionMapMethod === opt.value">
                                    <q-icon name="check" color="primary" />
                                </q-item-section>
                            </q-item>
                        </q-list>
                    </q-btn-dropdown>
                    <q-btn
                        color="amber"
                        text-color="dark"
                        :label="$t('daggerBtn')"
                        icon="school"
                        class="q-mr-sm"
                        @click="openDaggerDialog"
                    >
                        <q-tooltip>{{ $t('daggerDialogDesc') }}</q-tooltip>
                    </q-btn>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('stopBtn')"
                        icon="stop"
                        @click="stopInference"
                    ></q-btn>
                </div>
            </div>
            <div v-else>
                <div class="text-grey bg-dark border-rounded row full-height flex flex-center" v-if="status === 'pending'">
                    <q-select
                        v-model="selectedDatasetId"
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        :label="$t('selectDataset')"
                        style="width: 200px"
                        :options="datasets"
                        option-label="name"
                        option-value="id"
                        map-options
                        emit-value
                    ></q-select>
                    <q-input
                        v-model="languageInstruction"
                        dense outlined dark bg-color="dark"
                        :label="$t('languageInstruction')"
                        style="min-width: 200px; max-width: 400px;"
                        class="q-ml-sm"
                        clearable
                    />
                    <q-space></q-space>
                    <div class="row items-center q-mr-sm q-gutter-x-sm">
                        <q-select
                            dense outlined dark bg-color="dark"
                            v-model="teleType"
                            :options="teleTypeOptions"
                            map-options emit-value
                            style="min-width: 220px;"
                            :label="$t('teleoperationType')"
                        />
                        <q-input
                            v-if="teleType !== 'keyboard'"
                            v-model.number="collectionHz"
                            dense outlined dark bg-color="dark"
                            :label="$t('replayHzLabel')"
                            style="width: 80px"
                            type="number"
                            :min="1"
                        />
                        <q-btn
                            v-if="teleType === 'motion_planning'"
                            icon="settings"
                            round flat size="sm"
                            color="primary"
                            @click="showRos2ServiceDialog = true"
                        >
                            <q-tooltip>{{ $t('configureRos2Service') }}</q-tooltip>
                        </q-btn>
                        <q-input
                            v-if="moveHomposeInDataCollection"
                            v-model.number="moveHomposeDuration"
                            dense outlined dark bg-color="dark"
                            :label="$t('homeposeDurationSec')"
                            style="width: 90px"
                            type="number"
                            :min="0.1"
                            step="0.5"
                        />
                    </div>
                    <q-btn
                        color="red"
                        icon="fiber_manual_record"
                        text-color="white"
                        :label="$t('rec')"
                        @click="startDataCollection"
                    >
                        <q-badge
                            @click.stop="moveHomposeInDataCollection = !moveHomposeInDataCollection"
                            :color="moveHomposeInDataCollection ? 'blue' : 'grey-5'"
                            floating>
                            <q-icon name="home" size="xs" class="cursor-pointer" />
                        </q-badge>
                    </q-btn>
                </div>
                <div class="row flex flex-center full-height" v-else-if="viveInitializing">
                    <q-spinner-dots color="primary" size="2em" class="q-mr-md" />
                    <div class="text-white q-mr-lg">{{ $t('viveWaiting') }}</div>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('cancel')"
                        icon="close"
                        @click="cancelViveInit"
                    ></q-btn>
                </div>
                <div class="row flex flex-center full-height" v-else-if="movingHomepose">
                    <q-spinner-dots color="primary" size="2em" class="q-mr-md" />
                    <div class="text-white q-mr-lg">{{ $t('movingToHomepose') }}</div>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('stopCollection')"
                        icon="stop"
                        @click="stopDataCollection"
                    ></q-btn>
                </div>
                <div class="row flex flex-center full-height" v-else>
                    <q-input
                        v-if="teleType === 'keyboard'"
                        v-model.number="eeStepSize"
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        :label="$t('collectionStepSize')"
                        type="number"
                        step="0.0001"
                        style="width: 140px"
                        class="q-mr-md"
                        @keyup.enter="$event.target.blur()"
                    />
                    <div class="col q-pr-md">
                        <q-linear-progress
                            v-if="!savingEpisode"
                            :value="collectingProgress"
                            color="primary"
                            track-color="black"
                            size="30px"
                            instant-feedback
                        >
                            <div class="absolute-full flex flex-center">
                                <q-badge color="white" text-color="dark" :label="`${Number(collectingProgress * 100).toFixed(0)}%`" />
                            </div>
                        </q-linear-progress>
                        <div
                            v-else
                            class="row items-center justify-center bg-dark text-white rounded-borders"
                            style="height: 30px;"
                        >
                            <q-spinner-dots color="amber" size="22px" class="q-mr-sm" />
                            <span class="text-caption">{{ $t('savingEpisode') }}<span v-if="savingEpisodeName"> — {{ savingEpisodeName }}</span></span>
                        </div>
                    </div>

                    <q-btn
                        :color="succeedFlag ? 'yellow' : 'blue-grey'"
                        text-color="white"
                        :label="$t('successBtn')"
                        icon="emoji_events"
                        @click="setSucceed"
                        :disable="succeedFlag"
                        class="q-mr-sm"
                    ></q-btn>
                    <q-btn
                        color="green"
                        text-color="white"
                        :label="$t('completeEpisode')"
                        icon="check"
                        @click="completeEpisode"
                        class="q-mr-sm"
                    ></q-btn>
                    <q-btn
                        color="orange"
                        text-color="white"
                        :label="$t('throwEpisode')"
                        icon="delete_outline"
                        @click="throwEpisode"
                        class="q-mr-sm"
                    ></q-btn>
                    <q-btn
                        color="white"
                        text-color="red"
                        :label="$t('stopCollection')"
                        icon="stop"
                        @click="stopDataCollection"
                    ></q-btn>
                </div>
            </div>
        </div>

        <!-- DAgger 다이얼로그 — 추론 중에 stuck 발생 시 그 자세에서 즉시 텔레옵으로 -->
        <q-dialog v-model="showDaggerDialog" persistent>
            <q-card dark class="bg-secondary text-white" style="min-width: 480px; max-width: 600px;">
                <q-card-section class="bg-dark">
                    <div class="text-h6 row items-center">
                        <q-icon name="school" color="amber" class="q-mr-sm" />
                        {{ $t('daggerDialogTitle') }}
                    </div>
                </q-card-section>
                <q-card-section>
                    <div class="text-caption text-grey-4 q-mb-md">
                        {{ $t('daggerDialogDesc') }}
                    </div>
                    <div class="column q-gutter-y-md">
                        <q-select
                            v-model="selectedDatasetId"
                            dense outlined dark bg-color="dark"
                            :label="$t('selectDataset')"
                            :options="datasets"
                            option-label="name"
                            option-value="id"
                            map-options
                            emit-value
                        />
                        <q-input
                            v-model="languageInstruction"
                            dense outlined dark bg-color="dark"
                            :label="$t('languageInstruction')"
                            clearable
                        />
                        <q-select
                            v-model="teleType"
                            dense outlined dark bg-color="dark"
                            :options="teleTypeOptions"
                            map-options emit-value
                            :label="$t('teleoperationType')"
                        />
                        <div class="row q-col-gutter-sm">
                            <q-input
                                v-if="teleType !== 'keyboard'"
                                v-model.number="collectionHz"
                                dense outlined dark bg-color="dark"
                                :label="$t('replayHzLabel')"
                                class="col"
                                type="number"
                                :min="1"
                            />
                            <q-input
                                v-if="teleType === 'keyboard'"
                                v-model.number="eeStepSize"
                                dense outlined dark bg-color="dark"
                                :label="$t('collectionStepSize')"
                                class="col"
                                type="number"
                                step="0.0001"
                            />
                        </div>
                        <q-input
                            v-if="teleType === 'motion_planning'"
                            v-model="ros2Service"
                            dense outlined dark bg-color="dark"
                            :label="$t('ros2ServiceName')"
                            :placeholder="$t('ros2ServiceNamePlaceholder')"
                            :hint="$t('ros2ServiceNameHint')"
                        />
                    </div>
                </q-card-section>
                <q-card-actions align="right" class="q-pa-md">
                    <q-btn flat :label="$t('cancel')" color="grey-4" :disable="daggerBusy" v-close-popup />
                    <q-btn
                        unelevated
                        color="amber"
                        text-color="dark"
                        icon="school"
                        :label="$t('daggerStartBtn')"
                        :loading="daggerBusy"
                        :disable="!selectedDatasetId"
                        @click="submitDagger"
                    />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <!-- Vive 모드 선택 다이얼로그 (v-if 체인 외부) -->
        <q-dialog v-model="viveRobotDialog" persistent>
            <q-card dark style="min-width: 360px">
                <q-card-section>
                    <div class="text-h6">{{ $t('viveRobotDialogTitle') }}</div>
                    <div class="text-body2 q-mt-sm text-grey-4">{{ $t('viveRobotDialogMsg') }}</div>
                </q-card-section>
                <q-card-actions align="center" class="q-pb-md q-gutter-sm">
                    <q-btn
                        color="primary"
                        outline
                        :label="$t('viveWithRobot')"
                        icon="smart_toy"
                        @click="confirmViveMode('vive_external')"
                    />
                    <q-btn
                        color="white"
                        outline
                        :label="$t('viveWithoutRobot')"
                        icon="videocam"
                        @click="confirmViveMode('vive_only')"
                    />
                </q-card-actions>
            </q-card>
        </q-dialog>

        <div style="position: fixed; bottom: 20px; left: 20px; z-index: 1000;">
            <q-btn
                push
                color="white"
                :label="$t('terminal')"
                text-color="dark"
                @click="showProcessConsole = !showProcessConsole"
                style=""
            />
            <process-console
                :process="status === 'testing' ? 'checkpoint_test' : 'record_episode'"
                class="q-mt-md"
                style="border: 1px solid #ffffff; background-color: #1e1e1e; z-index: 1000; width: 800px; height: 400px;"
                v-show="showProcessConsole"
            >
            </process-console>
        </div>
        <div style="position: fixed; top: 10px; right: 20px; z-index: 10000; width: 600px; overflow: hidden;"
                v-if="selectedEpisode.name && selectedDatasetId"
                class="border-white border-rounded bg-dark q-pa-md relative-position"
        >
            <q-btn
                class="absolute-top-right"
                icon="close"
                round
                flat
                @click="selectedEpisode = {}"
                color="white"
                v-if="status === 'pending'"
            ></q-btn>
            <episode-viewer
                :path="`${selectedDatasetId}/${selectedEpisode.name}`"
                :total-frames="selectedEpisodeFrames"
                :disable-seek="isReplaying"
                :disable-playback="isReplaying"
                :task-id="workspace?.id"
            ></episode-viewer>
        </div>
        <!-- Inference Settings Dialog -->
        <form-dialog
            v-model="showInferenceDialog"
            :title="$t('inferenceSettings')"
            :form="inferenceForm"
            :ok-button-label="$t('startInference')"
            min-width="380px"
            @submit="onInferenceSubmit"
        />

        <!-- ROS2 Service Dialog -->
        <q-dialog v-model="showRos2ServiceDialog">
            <q-card style="min-width: 550px;" class="bg-secondary text-white">
                <q-card-section>
                    <div class="text-h6">{{ $t('ros2DialogTitle') }}</div>
                </q-card-section>
                <q-card-section>
                    <q-input
                        dense outlined dark bg-color="dark"
                        v-model="ros2Service"
                        :label="$t('ros2ServiceName')"
                        :placeholder="$t('ros2ServiceNamePlaceholder')"
                        :hint="$t('ros2ServiceNameHint')"
                    />
                </q-card-section>
                <q-card-actions align="right">
                    <q-btn flat :label="$t('cancel')" color="grey" v-close-popup />
                    <q-btn flat :label="$t('save')" color="primary" v-close-popup />
                </q-card-actions>
            </q-card>
            </q-dialog>
        </template>
        </div>
        </div>
</template>

<script setup>
import { defineProps, ref, computed, defineModel, onMounted, onUnmounted, watch } from 'vue';
import { Notify } from 'quasar';
import { useI18n } from 'vue-i18n';
import { api } from 'src/boot/axios';
import ProcessConsole from './ProcessConsole.vue';
import { useSocket } from 'src/composables/useSocket.js';
import WebRtcVideo from './WebRtcVideo.vue';
import EpisodeViewer from 'src/components/v2/EpisodeViewer.vue';
import FormDialog from './FormDialog.vue';
import { DEFAULT_KEYBOARD_SETTINGS, AXIS_TO_EE_INDEX, normalizeEventKey } from 'src/configs/teleopDefaults';
import TutorialHint from './TutorialHint.vue';
import { enumerateViews } from 'src/utils/sensorView';


const { socket } = useSocket();
const { t } = useI18n();

const props = defineProps({
    workspace: {
        type: Object,
        required: true
    },
    robots: {
        type: Array,
        required: true
    },
    sensors: {
        type: Array,
        required: true
    },
    checkpoints: {
        type: Array,
        required: true
    },
    status: {
        type: String,
        required: true
    },
    datasets: {
        type: Array,
        required: true
    },
    monitorOnly: {
        type: Boolean,
        default: false,
    },
    // (선택) 여러 workspace 가 같은 sensor 를 공유하지만 crop/resize/rotate 설정이
    // 서로 다른 경우 각 (workspace, sensor) 쌍마다 별도 viewport 를 렌더링하기 위한
    // 입력. 미지정 시 기존처럼 단일 `workspace` 의 sensor 설정으로 sensor 당 1 viewport.
    workspaces: {
        type: Array,
        default: null,
    },
    // (선택) Planner 같은 multi-view 모드에서 일부 viewport 만 화면에 노출하고
    // 싶을 때 사용. **viewport 자체는 항상 마운트** (WebRTC PC 가 destroy/create
    // 를 반복하지 않도록) 하고, ``v-show`` 로 숨김. 값 종류:
    //   - ``null`` (기본): 모두 노출 (기존 동작)
    //   - ``'primary'``: 물리 센서 당 첫 번째 view (occurrence=0) 만 노출
    //   - ``Set<string>``: 해당 viewKey 만 노출 (예: 체크포인트가 학습된 view 들)
    visibleViewKeys: {
        type: [Object, String, Set],
        default: null,
    },
    // CurriculumPage 의 교정 record_episode 흐름 — monitorOnly=true 와도
    // 호환되도록 바텀 record 패널을 강제 노출. record_episode 의 outer 루프
    // 가 iter=1 로 호출되어 1 에피소드 후 종료되고, MonitoringWindow 는
    // 일반 record 와 동일한 progress / Success / Complete / Throw / Stop UI 를
    // 그대로 사용한다 (간격/Success 버튼 등 일관성 유지). 버튼 클릭 시 부모
    // (CurriculumPage) 가 후속 resume 신호를 보내도록 추가 emit 만.
    correctionMode: {
        type: Boolean,
        default: false,
    },
});

// 교정 record 종료 시 CurriculumPage 가 적절한 resume 신호 (next / fallback /
// 전체 stop) 를 보내도록 emit. completeEpisode/throwEpisode/stopDataCollection
// 의 API 호출은 그대로 두고 (record_episode 워커 정상 종료), 추가로 emit 만.
const emit = defineEmits(['correction-done', 'correction-throw', 'correction-stop']);

// "primary" 모드용: sensorViewports 를 순회하며 처음 만난 sensor 의 viewport 만
// 살린다. workspace 가 여러 개라도 같은 물리 센서는 한 번만 노출 → "기본 센서 뷰".
const primaryViewportKeys = computed(() => {
    const seenSids = new Set();
    const keys = new Set();
    for (const vp of sensorViewports.value) {
        const sid = vp.sensor?.id;
        if (sid == null) continue;
        if (seenSids.has(sid)) continue;
        seenSids.add(sid);
        keys.add(vp.key);
    }
    return keys;
});

// 한 viewport 를 화면에 보일지 여부. props.visibleViewKeys 의 mode 에 따라 분기.
// 핵심: 숨기더라도 컴포넌트는 마운트 상태 유지 → WebRTC 연결은 끊기지 않는다.
//   Set 매칭 키는 ``vp.key`` (``${workspaceId}-${viewKey}``) — 같은 viewKey 가
//   여러 workspace 에 있어도 정확히 한 workspace 의 viewport 만 노출 가능.
function isViewportVisible(vp) {
    const v = props.visibleViewKeys;
    if (v == null) return true;
    if (v === 'primary') return primaryViewportKeys.value.has(vp.key);
    if (v instanceof Set) return v.has(String(vp.key));
    return true;
}

const focused = defineModel('focused', {
    type: Object,
    default: () => ({ id: null, device_type: null })
});

// Multi-view: 각 viewport = (workspace, view) 쌍. 한 workspace 가 같은 sensor 의
// 여러 view 를 등록하면 그만큼 viewport 도 늘어남. Planner 모드 (props.workspaces)
// 와 단일 workspace 모드 모두 같은 enumeration 으로 처리.
const sensorViewports = computed(() => {
    const sensorById = new Map(props.sensors.map(s => [Number(s.id), s]));
    const buildItems = (ws, wsName) => {
        const sids = (ws && ws.sensor_ids) || [];
        return enumerateViews(sids).map(({ sensorId, viewKey, occurrence }) => {
            const live = sensorById.get(sensorId);
            if (!live) return null;
            return {
                key: `${ws.id || 'ws'}-${viewKey}`,
                workspace: ws,
                workspaceName: wsName,
                sensor: live,
                viewKey,
                occurrence,
            };
        }).filter(Boolean);
    };
    if (props.workspaces && props.workspaces.length > 0) {
        const items = [];
        props.workspaces.forEach((ws) => {
            items.push(...buildItems(ws, ws.name || ''));
        });
        return items;
    }
    return buildItems(props.workspace || {}, '');
});

// 모든 lookup: view_key 우선, 없으면 sensor_id 로 fallback (기존 single-view
// dataset/task 호환). `??` 가 아니라 `in` 체크로 "키 존재" 를 판별 — rotation=0
// 같은 falsy 정상값이 fallback 으로 잘못 넘어가는 버그를 막는다 (`||` 사용 시
// d["5_2"]=0 → falsy → d["5"]=270 로 빠짐).
function viewportCropArea(ws, viewKey, sensorId) {
    const d = ws && ws.sensor_cropped_area;
    if (!d) return {};
    if (viewKey in d) return d[viewKey];
    if (sensorId in d) return d[sensorId];
    return {};
}
function viewportImgSize(ws, viewKey, sensorId) {
    const d = ws && ws.sensor_img_size;
    if (!d) return [640, 480];
    if (viewKey in d) return d[viewKey];
    if (sensorId in d) return d[sensorId];
    return [640, 480];
}
function viewportRotate(ws, viewKey, sensorId) {
    const d = ws && ws.sensor_rotate;
    if (!d) return 0;
    if (viewKey in d) return d[viewKey];
    if (sensorId in d) return d[sensorId];
    return 0;
}
const selectedDatasetId = defineModel('selectedDatasetId', {
    type: [String, Number],
    default: null
});
const selectedCheckpointId = defineModel('selectedCheckpointId', {
    type: [String, Number],
    default: null
});
const selectedEpisode = defineModel('selectedEpisode', {
    type: Object,
    default: {}
});
const replayActionType = ref('joint');
const isReplaying = ref(false);
const selectedEpisodeFrames = ref(0);

watch(
    () => [selectedDatasetId.value, selectedEpisode.value?.name],
    ([dsId, epName]) => {
        if (!dsId || !epName) {
            selectedEpisodeFrames.value = 0;
            return;
        }
        api.get(`/dataset/${dsId}/${epName}/:get_data`).then((res) => {
            selectedEpisodeFrames.value = res.data?.episode?.num_frames || 0;
        }).catch((error) => {
            console.error('Error fetching episode frame count:', error);
            selectedEpisodeFrames.value = 0;
        });
    },
    { immediate: true },
);
const checkpoint = computed(() => {
    return props.checkpoints?.find(c => c.id === selectedCheckpointId.value);
});

const teleType = ref('leader')
const collectionHz = ref(20)
const languageInstruction = ref('')
const viveInitializing = ref(false)
const viveRobotDialog = ref(false)


const teleTypeOptions = [
    { label: 'Easy Controller', value: 'leader' },
    { label: 'Keyboard', value: 'keyboard' },
    { label: 'External', value: 'external' },
    { label: 'Vive', value: 'vive_external' },
    { label: 'External + Motion Planning', value: 'motion_planning' },
]

const replayCapture = ref(false)
const replayCaptureDatasetId = ref(null)
const replayHz = ref(5)
const replayProgress = ref(0)

const showRos2ServiceDialog = ref(false)
const ros2Service = ref('')
const allSensorsOn = computed(() => {
    return props.sensors.every(s => s.status === 'on');
});

const isRobotSensorAllOn = computed(() => {
    const allRobotsOn = props.robots.every(r => r.status === 'on');
    const allSensorsOn = props.sensors.every(s => s.status === 'on');
    return allRobotsOn && allSensorsOn;
});

const monitoringHintKey = computed(() => {
    if (!isRobotSensorAllOn.value) return 'tutorialMonOverview';
    if (selectedEpisode.value?.name && selectedDatasetId.value) return 'tutorialMonReplay';
    if (checkpoint.value) {
        return props.status === 'pending' ? 'tutorialMonInferenceSelected' : 'tutorialMonInferenceRunning';
    }
    if (props.status === 'pending') return 'tutorialMonIdlePending';
    if (viveInitializing.value) return 'tutorialMonViveInit';
    if (movingHomepose.value) return 'tutorialMonMovingHome';
    switch (teleType.value) {
        case 'keyboard': return 'tutorialMonCollectingKeyboard';
        case 'leader': return 'tutorialMonCollectingLeader';
        case 'external': return 'tutorialMonCollectingExternal';
        case 'vive_external': return 'tutorialMonCollectingViveExternal';
        case 'vive_only': return 'tutorialMonCollectingViveOnly';
        case 'motion_planning': return 'tutorialMonCollectingMotionPlanning';
        default: return 'tutorialMonOverview';
    }
});

// Multi-view: backend 의 record_episode / checkpoint_test / replay 는 ``sensors``
// 배열을 view 와 1:1 매칭으로 받는다 (같은 sensor_id 가 N 번 들어와도 그대로
// N 개로). WorkspacePage 가 카드 표시용으로 deduped `selectedSensors` 를 넘기지만,
// 백엔드 payload 에는 sensor_ids 순서대로 펼친 list 가 필요하다.
function expandSensorsForBackend(taskSource = null) {
    const ws = taskSource || props.workspace;
    const sensorById = new Map((props.sensors || []).map((s) => [Number(s.id), s]));
    const sids = ws?.sensor_ids || [];
    return sids.map((sid) => sensorById.get(Number(sid))).filter(Boolean);
}

function focusSensorRobot(device, type, viewKey = undefined) {
    if (device.status !== 'on') {
        return;
    }
    // Multi-view: 같은 sensor 의 다른 view 도 별개 focused 로 취급. viewKey 가
    // 같이 일치할 때만 toggle off (그래야 다른 view 클릭 시 그쪽으로 전환).
    const sameTarget = focused.value
        && focused.value.id === device.id
        && focused.value.device_type === type
        && (type !== 'sensor' || focused.value.viewKey === viewKey);
    if (sameTarget) {
        focused.value = {};
        return;
    }

    focused.value = {
        ...device, // device 객체의 모든 속성을 복사
        device_type: type, // device_type 속성을 추가 (또는 덮어쓰기)
        ...(type === 'sensor' && viewKey ? { viewKey } : {}),
    };
}


const collectingProgress = ref(0);
const savingEpisode = ref(false);
const savingEpisodeName = ref('');
const showProcessConsole = ref(false);

const moveHomposeInDataCollection = ref(false);
// home pose 로 이동할 때 걸리는 시간(초). agent.move_to 의 duration 으로 그대로 전달.
const moveHomposeDuration = ref(5);

// home 버튼이 켜진 추론에서만 의미 있는 1 에피소드 길이. 기본값은 task의
// episode_len * 2이지만 watch로 동기화 — props.workspace가 늦게 도착해도 잡힘.
const inferenceEpisodeLen = ref(null);
watch(
    () => props.workspace?.episode_len,
    (newLen) => {
        if (newLen && (inferenceEpisodeLen.value === null || inferenceEpisodeLen.value === undefined)) {
            inferenceEpisodeLen.value = Number(newLen) * 2;
        }
    },
    { immediate: true },
);

const inferenceProgress = ref({ progress: 0, step: 0, episodeLen: 0 });
const movingHomepose = ref(false);
const succeedFlag = ref(false);
const inferenceSucceed = ref(false);
const succeedScore = ref(null);
const oodScore = ref(null);
const oodTotal = computed(() => {
    if (!oodScore.value) return 0;
    const img = oodScore.value.image;
    const state = oodScore.value.state;
    if (typeof img === 'number' && typeof state === 'number') return (img + state) / 2;
    if (typeof img === 'number') return img;
    if (typeof state === 'number') return state;
    return 0;
});
const oodScoreDisplay = computed(() => {
    if (!oodScore.value) return '';
    const img = oodScore.value.image;
    const state = oodScore.value.state;
    const imgStr = typeof img === 'number' ? img.toFixed(2) : '-';
    const stateStr = typeof state === 'number' ? state.toFixed(2) : '-';
    return `${oodTotal.value.toFixed(2)} (${imgStr} + ${stateStr})`;
});

// =========================================================================
// Inference-time Vision Map — overlays per-sensor heatmaps on the live
// WebRTC feed during inference. Backend hooks the running policy's forward
// pass (attention is essentially free; gradcam adds a backward pass per
// step). User toggles via checkbox + method select in the status bar.
// =========================================================================
const inferenceVisionMapOn = ref(false);
const inferenceVisionMapMethod = ref('attention');
const inferenceHeatmaps = ref({}); // { 'sensor_<id>': data-url }
const inferenceVisionMapMethodOptions = computed(() => [
    { label: t('visionMapAttention'), value: 'attention' },
    { label: t('visionMapGradcam'), value: 'gradcam' },
]);
// Only ACT supports the inference-time hooks (no equivalent on Diffusion/PI0).
const supportsInferenceVisionMap = computed(() =>
    String(checkpoint.value?.policy?.type || '').toUpperCase() === 'ACT',
);

function applyInferenceVisionMap(method) {
    if (!selectedCheckpointId.value) return;
    api.post(`/checkpoint/${selectedCheckpointId.value}/:set_inference_vision_map`, { method })
        .catch((err) => console.warn('[InferenceVisionMap] set failed:', err));
}

function setInferenceVisionMap(method) {
    if (method === null) {
        inferenceVisionMapOn.value = false;
    } else {
        inferenceVisionMapMethod.value = method;
        inferenceVisionMapOn.value = true;
    }
}

watch([inferenceVisionMapOn, inferenceVisionMapMethod], () => {
    // Only push to backend when inference is actually running — the status
    // watcher below re-applies on transition into 'testing'.
    if (props.status !== 'testing') return;
    if (inferenceVisionMapOn.value) {
        applyInferenceVisionMap(inferenceVisionMapMethod.value);
    } else {
        applyInferenceVisionMap(null);
        inferenceHeatmaps.value = {};
    }
});

watch(() => props.status, (newStatus, oldStatus) => {
    if (newStatus === 'testing' && oldStatus !== 'testing') {
        // Inference just started — re-push the current toggle so the backend
        // emits heatmaps from the very first step.
        if (inferenceVisionMapOn.value && supportsInferenceVisionMap.value) {
            applyInferenceVisionMap(inferenceVisionMapMethod.value);
        }
    } else if (oldStatus === 'testing' && newStatus !== 'testing') {
        inferenceHeatmaps.value = {};
    }
});

function startDataCollection() {
    if (!selectedDatasetId.value) {
        Notify.create({
            color: 'negative',
            message: t('selectDatasetRequired')
        });
        return;
    }
    // vive_external 선택 시: 실물 로봇 여부를 다이얼로그로 확인
    if (teleType.value === 'vive_external') {
        viveRobotDialog.value = true;
        return;
    }
    _doStartDataCollection(teleType.value);
}

function confirmViveMode(effectiveTeleType) {
    viveRobotDialog.value = false;
    // DAgger 경유 진입이면 home pose 이동을 강제로 끈다. 일반 vive 시작이면
    // 옵션 없이 기존 동작.
    const options = _pendingDaggerVive ? { forceNoHomepose: true } : {};
    _pendingDaggerVive = false;
    _doStartDataCollection(effectiveTeleType, options);
}

function _doStartDataCollection(effectiveTeleType, options = {}) {
    // ``forceNoHomepose`` 는 DAgger 흐름 전용 — 추론 → 수집 전환 시 사용자가
    // 누른 시점에서 즉시 텔레옵으로 넘기고 싶기 때문에 home pose 이동을
    // 강제로 비활성화한다. 일반 collection 흐름은 옵션 없이 호출 → 기존 값
    // (moveHomposeInDataCollection.value) 그대로 사용.
    // ``taskOverride`` 는 curriculum 교정 흐름 전용 — 실패한 체크포인트의
    // max_steps 를 task.episode_len 으로 덮어쓰기 위해 사용.
    const moveHomepose = options.forceNoHomepose ? false : moveHomposeInDataCollection.value;
    const taskForBackend = options.taskOverride || props.workspace;

    if (effectiveTeleType === 'keyboard') {
        addKeyboardListener();
    }
    if (effectiveTeleType === 'vive_external' || effectiveTeleType === 'vive_only') {
        viveInitializing.value = true;
    }
    succeedFlag.value = false;
    addSucceedKeyListener();
    showProcessConsole.value = true;
    collectingProgress.value = 0;
    // 백엔드가 sensor init/env setup/home pose move 까지 끝내고 첫
    // moving_homepose:false emit 을 보내기 전까지는 사용자 입력이 들어오면 안된다.
    // 백엔드 setup 만 3~5초 걸려서 그 사이 keyboardHandler 가 그대로 작동했었음.
    if (moveHomepose && effectiveTeleType !== 'externel' && effectiveTeleType !== 'vive_only') {
        movingHomepose.value = true;
    }
    const payload = {
        task: taskForBackend,
        robots: props.robots,
        sensors: expandSensorsForBackend(taskForBackend),
        tele_type: effectiveTeleType,
        assembly_id: taskForBackend.assembly_id,
        move_homepose: moveHomepose,
        move_homepose_duration: Number(moveHomposeDuration.value) || 5.0,
        hz: collectionHz.value,
        language_instruction: languageInstruction.value || '',
    };
    // ``options.iter`` — record_episode 의 outer 루프 횟수. 교정 흐름은 1 episode
    // 후 자동 종료가 필요해서 iter=1 로 호출 → 사용자가 완료/버리기/중지 중
    // 어떤 걸 누르더라도 outer 루프가 한 번 돌고 끝나서 stop_process emit 이
    // 발생, CurriculumPage 가 적절한 resume 신호를 보낸다.
    if (options.iter != null) {
        payload.iter = options.iter;
    }
    if (teleType.value === 'motion_planning' && ros2Service.value) {
        payload.ros2_service = ros2Service.value;
    }
    return api.post(`/dataset/${selectedDatasetId.value}/:start_collection`, payload).catch((error) => {
        viveInitializing.value = false;
        // 실패 시 등록한 listener 도 정리해야 다음 시도에서 중복 등록 안 됨.
        if (effectiveTeleType === 'keyboard') removeKeyboardListener();
        removeSucceedKeyListener();
        console.error('Error starting data collection:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStartCollection')
        });
        throw error;
    });
}

// CurriculumPage 교정 흐름 진입점 — 외부에서 ref 로 호출. 내부 refs 를
// cfg 에 맞춰 동기화한 뒤 일반 record 흐름 (_doStartDataCollection) 으로
// 위임 → keyboard / succeed listener, moving_homepose 처리 등이 그대로
// 재사용된다. 직접 ``/:start_collection`` 을 호출하지 말 것 (listener 누락).
function startCorrectionRecording(cfg) {
    selectedDatasetId.value = cfg.datasetId;
    teleType.value = cfg.teleType || 'keyboard';
    collectionHz.value = Number(cfg.hz) || 20;
    languageInstruction.value = cfg.languageInstruction || '';
    // 교정은 실패 직후 위치에서 시연이므로 home pose 이동 강제 off.
    moveHomposeInDataCollection.value = false;
    return _doStartDataCollection(teleType.value, {
        forceNoHomepose: true,
        taskOverride: cfg.taskOverride,
        iter: 1,  // single-episode 모드 — 완료/버리기 후 record_episode 자동 종료.
    });
}

defineExpose({ startCorrectionRecording });

function cancelViveInit() {
    viveInitializing.value = false;
    api.post(`/dataset/${selectedDatasetId.value}/:stop_collection`).then(() => {
        collectingProgress.value = 0;
    });
}

const eeStepSize = ref(0.0005);

const assembly = computed(() => props.workspace?.assembly || {});

const leftArm = computed(() => {
    const ref_ = assembly.value.left_arm;
    if (!ref_) return null;
    return props.robots.find(r => r.id === ref_.id) || ref_;
});

const rightArm = computed(() => {
    const ref_ = assembly.value.right_arm;
    if (!ref_) return null;
    return props.robots.find(r => r.id === ref_.id) || ref_;
});

const isDualArm = computed(() => !!(leftArm.value && rightArm.value));

const keyboardSetting = computed(() => {
    const s = assembly.value.teleoperators?.find(tt => tt.type === 'keyboard')?.settings;
    return s && s.axis_map ? s : DEFAULT_KEYBOARD_SETTINGS;
});

// keyboard 설정의 step_size를 UI 인풋(eeStepSize)의 기본값으로 동기화.
watch(() => keyboardSetting.value.step_size, (v) => {
    if (typeof v === 'number') eeStepSize.value = v;
}, { immediate: true });

const activeArms = computed(() => [leftArm.value, rightArm.value].filter(Boolean));

function robotForSide(side) {
    if (isDualArm.value) {
        return side === 'right' ? rightArm.value : leftArm.value;
    }
    if (side === 'right') return null;
    return leftArm.value || rightArm.value;
}

// 별도 그리퍼(role=tool) agent. arm이 tool_inner=false 인데 그리퍼를 함께 텔레옵하고
// 싶을 때(예: fairino arm + robotiq) 키보드의 tool 축 입력을 이 agent로 라우팅한다.
// assembly.{left,right}_tool 은 assembly_model.to_dict 가 각 part 마다 별도 dict 로
// 만들어 보내서 handler 가 안 붙어있다 → props.robots(=handler 부착된 canonical
// 목록) 에서 id 로 다시 찾는다. leftArm/rightArm 와 동일 패턴.
function toolForSide(side) {
    const resolve = (partKey) => {
        const ref_ = assembly.value[partKey];
        if (!ref_) return null;
        return props.robots.find(r => r.id === ref_.id) || ref_;
    };
    const left = resolve('left_tool');
    const right = resolve('right_tool');
    if (isDualArm.value) {
        return side === 'right' ? right : left;
    }
    return left || right;
}

function sendDelta(robot, eeDelta) {
    if (!robot?.handler?.moveRobotEEDelta) return;
    try {
        robot.handler.moveRobotEEDelta({ ee: eeDelta });
    } catch (e) {
        console.error('moveRobotEEDelta error', e);
    }
}

const keyboardHandler = (event) => {
    // Home pose 이동 / vive init 중에는 사용자 키 입력 무시 — 그동안 로봇이 백엔드
    // 명령으로 움직이고 있으므로 사용자 입력과 충돌하면 충돌 또는 의도치 않은 위치로 감.
    if (movingHomepose.value || viveInitializing.value) return;
    // Step size 등 폼 입력란이 포커스된 상태에서, 숫자 편집 키는 입력에 양보하고
    // 그 외 키(WASD 등 로봇 제어키)는 자동으로 input을 blur 후 그대로 로봇 제어로 처리.
    const ae = document.activeElement;
    if (ae) {
        const tag = ae.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || ae.isContentEditable) {
            const isEditingKey = (
                /^[0-9]$/.test(event.key) ||
                event.key === '.' || event.key === 'e' || event.key === 'E' ||
                event.key === '+' || event.key === '-' ||
                event.key === 'Backspace' || event.key === 'Delete' ||
                event.key === 'Tab' || event.key === 'Enter' ||
                event.key === 'Home' || event.key === 'End' ||
                event.key.startsWith('Arrow')
            );
            if (isEditingKey) return;
            ae.blur();
        }
    }

    const map = keyboardSetting.value.axis_map || {};
    // UI step size override; fall back to assembly setting; finally default.
    const stepSize = Number(eeStepSize.value) || Number(keyboardSetting.value.step_size) || 0.003;

    if (event.key === ' ' || event.key === 'Space') {
        activeArms.value.forEach((robot) => {
            const len = robot.tool_inner ? 7 : 6;
            sendDelta(robot, Array(len).fill(0));
        });
        event.preventDefault();
        return;
    }

    const normKey = normalizeEventKey(event);
    const entry = map[normKey];
    if (!entry) return;

    const side = entry.side || 'left';
    const robot = robotForSide(side);

    const idx = AXIS_TO_EE_INDEX[entry.axis];
    if (idx === undefined) return;

    // tool 축: arm 내장 그리퍼면 EE delta[6] 으로 packing, 별도 gripper agent 가
    // 있으면 그 agent 의 joint delta 로 라우팅. 둘 다 없으면 무시.
    if (entry.axis === 'tool') {
        const delta = stepSize * (Number(entry.scale) || 1) * (Number(entry.sign) || 1);
        if (robot && robot.tool_inner) {
            const eeDelta = Array(7).fill(0);
            eeDelta[idx] = delta;
            sendDelta(robot, eeDelta);
            event.preventDefault();
            return;
        }
        const tool = toolForSide(side);
        if (!tool || !tool.handler || !tool.handler.moveRobotJointDelta) return;
        const len = Array.isArray(tool.joint_names) && tool.joint_names.length > 0
            ? tool.joint_names.length : 1;
        const jointDelta = Array(len).fill(0);
        jointDelta[0] = delta;
        try {
            tool.handler.moveRobotJointDelta(jointDelta);
        } catch (e) {
            console.error('moveRobotJointDelta error', e);
        }
        event.preventDefault();
        return;
    }

    if (!robot) return;
    const len = robot.tool_inner ? 7 : 6;
    if (idx >= len) return;
    const eeDelta = Array(len).fill(0);
    eeDelta[idx] = stepSize * (Number(entry.scale) || 1) * (Number(entry.sign) || 1);
    sendDelta(robot, eeDelta);
    event.preventDefault();
};

function addKeyboardListener() {
    // 중복 등록 방지를 위해 먼저 제거 후 등록
    window.removeEventListener('keydown', keyboardHandler);
    window.addEventListener('keydown', keyboardHandler);
}

function removeKeyboardListener() {
    window.removeEventListener('keydown', keyboardHandler);
}

function completeEpisode() {
    api.post(`/dataset/${selectedDatasetId.value}/:complete_episode`).catch((error) => {
        console.error('Error completing episode:', error);
        Notify.create({
            color: 'negative',
            message: t('errorCompleteEpisode'),
        });
    });
    if (props.correctionMode) emit('correction-done');
}

function setSucceed() {
    if (succeedFlag.value) return;
    succeedFlag.value = true;
    api.post(`/dataset/${selectedDatasetId.value}/:set_succeed`).catch((error) => {
        console.error('Error setting succeed flag:', error);
    });
}

function throwEpisode() {
    api.post(`/dataset/${selectedDatasetId.value}/:throw_episode`).catch((error) => {
        console.error('Error throwing episode:', error);
    });
    if (props.correctionMode) emit('correction-throw');
}

const succeedKeyHandler = (event) => {
    // 폼 입력 중일 땐 무시
    const ae = document.activeElement;
    if (ae && (ae.tagName === 'INPUT' || ae.tagName === 'TEXTAREA' || ae.isContentEditable)) return;
    // 1 = Done (현재 episode 를 succeed 로 마크)
    // 2 = Finish (현재 episode 저장 후 다음 에피소드)
    // 3 = Throw (현재 episode 저장 없이 버리고 다음 에피소드)
    // 4 = Stop (전체 collection 중단)
    const k = event.key;
    if (k === '1') {
        setSucceed();
        event.preventDefault();
    } else if (k === '2') {
        completeEpisode();
        event.preventDefault();
    } else if (k === '3') {
        throwEpisode();
        event.preventDefault();
    } else if (k === '4') {
        stopDataCollection();
        event.preventDefault();
    }
};

function addSucceedKeyListener() {
    window.removeEventListener('keydown', succeedKeyHandler);
    window.addEventListener('keydown', succeedKeyHandler);
}

function removeSucceedKeyListener() {
    window.removeEventListener('keydown', succeedKeyHandler);
}

function stopDataCollection() {
    if (teleType.value === 'keyboard') {
        removeKeyboardListener();
    }
    removeSucceedKeyListener();
    succeedFlag.value = false;
    viveInitializing.value = false;
    movingHomepose.value = false;
    // 백엔드 워커가 실제로 멈출 때까지 시간이 걸릴 수 있으므로 progress UI는
    // 즉시 정리해서 사용자에게 멈춘 인상을 즉각 준다. backend가 이후에 보낼
    // 수도 있는 'moving_homepose: true' 등 잔여 이벤트는 onUnmounted/socket.off
    // 또는 아래 stop_process listener가 정리.
    collectingProgress.value = 0;
    api.post(`/dataset/${selectedDatasetId.value}/:stop_collection`).then(() => {
        collectingProgress.value = 0;
    })
    if (props.correctionMode) emit('correction-stop');
}

const hz = ref(10);
const moveHomposeSettleSec = ref(0);
const showInferenceDialog = ref(false);
const inferenceForm = ref([
    {
        key: 'hz',
        label: t('frequencyHz'),
        type: 'number',
        value: 10,
    },
    {
        key: 'move_homepose',
        label: t('plannerMoveHomepose'),
        type: 'checkbox',
        value: false,
    },
    {
        key: 'inference_episode_len',
        label: t('inferenceEpisodeLen'),
        type: 'number',
        value: null,
        show: (form) => form.find(f => f.key === 'move_homepose')?.value === true,
    },
    {
        key: 'move_homepose_duration',
        label: t('homeposeDurationSec'),
        type: 'number',
        value: 5,
        show: (form) => form.find(f => f.key === 'move_homepose')?.value === true,
    },
    {
        key: 'move_homepose_settle_sec',
        label: t('homeposeSettleSec'),
        type: 'number',
        value: 0,
        show: (form) => form.find(f => f.key === 'move_homepose')?.value === true,
    },
    {
        key: 're_inference_steps',
        label: t('reInferenceSteps'),
        type: 'number',
        value: 1,
    },
    {
        key: 'temporal_ensemble_coeff',
        label: t('temporalEnsembleCoeff'),
        type: 'number',
        value: 0.01,
        show: (form) => form.find(f => f.key === 're_inference_steps')?.value === 1,
    },
    {
        // VLA (PI05) language prompt. Empty string → backend falls back to task.name.
        key: 'language_instruction',
        label: 'Language Prompt (VLA, e.g. PI0.5)',
        type: 'text',
        value: '',
        placeholder: 'e.g. "pick up the red cup" (leave empty to use task name)',
        show: () => checkpoint.value?.policy?.type === 'PI05',
    },
]);

function startInference() {
    // 다이얼로그 열기 직전, 현재 ref 값으로 form 초기값을 동기화 — 이전에 바꾼 설정이 유지된다.
    const setField = (key, value) => {
        const f = inferenceForm.value.find(x => x.key === key);
        if (f) f.value = value;
    };
    setField('hz', hz.value);
    setField('move_homepose', moveHomposeInDataCollection.value);
    setField('inference_episode_len', inferenceEpisodeLen.value);
    setField('move_homepose_duration', Number(moveHomposeDuration.value) || 5.0);
    setField('move_homepose_settle_sec', Number(moveHomposeSettleSec.value) || 0);
    showInferenceDialog.value = true;
}

function onInferenceSubmit(formData) {
    // form 값을 ref 에 반영해 다음 호출/데이터 수집과 공유한다.
    hz.value = Number(formData.hz) || 10;
    moveHomposeInDataCollection.value = !!formData.move_homepose;
    inferenceEpisodeLen.value = formData.inference_episode_len;
    moveHomposeDuration.value = Number(formData.move_homepose_duration) || 5.0;
    moveHomposeSettleSec.value = Number(formData.move_homepose_settle_sec) || 0;

    showProcessConsole.value = true;
    inferenceProgress.value = { progress: 0, step: 0, episodeLen: 0 };
    api.post(`/checkpoint/${selectedCheckpointId.value}/:start_test`, {
        task: props.workspace,
        policy: checkpoint.value.policy,
        robot_ids: props.robots.map(r => r.id),
        sensors: expandSensorsForBackend(),
        checkpoint: checkpoint.value,
        move_homepose: !!formData.move_homepose,
        move_homepose_duration: Number(formData.move_homepose_duration) || 5.0,
        move_homepose_settle_sec: Number(formData.move_homepose_settle_sec) || 0,
        hz: Number(formData.hz) || 10,
        re_inference_steps: formData.re_inference_steps,
        temporal_ensemble_coeff: formData.re_inference_steps === 1 ? formData.temporal_ensemble_coeff : null,
        // PI0.5 / VLA prompt; backend uses task.name as fallback if empty
        language_instruction: formData.language_instruction || '',
        // Planner feature: inference 시 episode_len 지정
        inference_episode_len: formData.move_homepose ? formData.inference_episode_len : null,
    }).catch((error) => {
        console.error('Error starting test:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStartTest')
        });
    });
}

function stopInference() {
    return api.post(`/checkpoint/${selectedCheckpointId.value}/:stop_test`).catch((error) => {
        console.error('Error stopping test:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStopTest')
        });
    });
}

// ─── DAgger flow ─────────────────────────────────────────────────────────
// 추론 중 stuck 상태에서 사용자가 "여기서부터 내가 직접 빠져나오는 걸
// 보여줄게" 라고 선언하는 흐름. 흐름은 두 단계:
//   1. 현재 추론 프로세스를 중지 (stopInference)
//   2. home pose 이동 없이 (forceNoHomepose) 곧바로 데이터 수집 시작
// 백엔드는 record_episode 가 시작될 때 첫 프레임의 로봇 자세를 episode
// 시작점으로 잡으므로, robot이 어디에 있든 그 위치부터 텔레옵 데이터가
// 기록된다 — 별도 백엔드 변경 없이 동작.
const showDaggerDialog = ref(false);
const daggerBusy = ref(false);

function openDaggerDialog() {
    if (!selectedDatasetId.value && props.datasets?.length) {
        // 다이얼로그 열릴 때 데이터셋이 비어있으면 첫 항목으로 prefill
        selectedDatasetId.value = props.datasets[0].id;
    }
    // DAgger 의 핵심은 "stuck 상태에서 즉시 탈출 시연"이므로 버튼을 누르는
    // 순간 추론을 끊는다. 사용자가 다이얼로그를 취소해도 추론은 다시 자동
    // 시작되지 않는데 — 어차피 stuck 인 정책을 계속 두는 것보단 멈춰두는
    // 게 안전. 다이얼로그에서 폼 입력하는 동안 백엔드 process_manager 가
    // :stop_test 정리를 끝낸다 → submitDagger 시점엔 start_collection 이
    // 깔끔하게 받아간다.
    stopInference();
    showDaggerDialog.value = true;
}

async function submitDagger() {
    if (!selectedDatasetId.value) {
        Notify.create({ color: 'negative', message: t('selectDatasetRequired') });
        return;
    }
    // vive_external 은 실로봇 동행 여부를 별도로 묻는 패턴이 있는데, DAgger
    // 는 "현재 자세에서 즉시 텔레옵" 이 핵심이라 vive_only(이미지+EE만) 로
    // 강등하면 의미가 깨진다. vive 가 필요하면 사용자가 미리 다른 텔레옵
    // 으로 바꾸도록 안내 — 다이얼로그 안의 q-select 에서 바로 바꿀 수 있다.
    daggerBusy.value = true;
    try {
        // 추론은 openDaggerDialog 시점에 이미 끊었음. 여기선 곧바로 수집 시작.
        if (teleType.value === 'vive_external') {
            showDaggerDialog.value = false;
            // 기존 vive 확인 흐름으로 이양 — confirmViveMode 가 _doStartDataCollection
            // 호출. forceNoHomepose 옵션 전달이 필요하므로 한 번만 쓰는 플래그로
            // 우회한다.
            _pendingDaggerVive = true;
            viveRobotDialog.value = true;
            daggerBusy.value = false;
            return;
        }
        await _doStartDataCollection(teleType.value, { forceNoHomepose: true });
        showDaggerDialog.value = false;
    } catch (err) {
        console.error('DAgger start failed:', err);
        Notify.create({ color: 'negative', message: t('errorStartDagger') });
    } finally {
        daggerBusy.value = false;
    }
}

// vive_external 경유 DAgger 흐름: confirmViveMode 콜백이 _doStartDataCollection
// 을 호출할 때 forceNoHomepose 를 전달해야 한다. 한 번 쓰고 비우는 단방향
// 플래그라서 ref 가 아닌 모듈 스코프 변수로 둔다 — 반응성 필요 없음.
let _pendingDaggerVive = false;

function startReplay() {
    replayProgress.value = 0;
    isReplaying.value = true;
    const payload = {
        episode: selectedEpisode.value,
        robot_ids: props.robots.map(r => r.id),
        sensors: expandSensorsForBackend(),
        task: props.workspace,
        action_type: replayActionType.value,
        hz: replayHz.value,
    }
    if (replayCapture.value && replayCaptureDatasetId.value) {
        payload.capture_dataset_id = replayCaptureDatasetId.value
    }
    api.post(`/dataset/${selectedDatasetId.value}/${selectedEpisode.value.name}/:start_replay_episode`, payload).catch((error) => {
        console.error('Error starting replay:', error);
        isReplaying.value = false;
        Notify.create({
            color: 'negative',
            message: t('errorStartReplay')
        });
    });
}

function stopReplay() {
    api.post(`/dataset/${selectedDatasetId.value}/${selectedEpisode.value.name}/:stop_replay_episode`).catch((error) => {
        console.error('Error stopping replay:', error);
        Notify.create({
            color: 'negative',
            message: t('errorStopReplay')
        });
    });
}

onMounted(() => {

    console.log(props.workspace);

    socket.on('stop_process', (data) => {
        if (data.id === 'record_episode') {
            if (data.episode_saved) {
                Notify.create({
                    color: 'positive',
                    message: t('episodeSaved'),
                });
            }
            collectingProgress.value = 0;
            viveInitializing.value = false;
            movingHomepose.value = false;
            succeedFlag.value = false;
            removeSucceedKeyListener();
        }
        if (data.id === 'replay_episode') {
            replayProgress.value = 0;
            isReplaying.value = false;
        }
        if (data.id === 'checkpoint_test') {
            inferenceSucceed.value = false;
            succeedScore.value = null;
            oodScore.value = null;
            inferenceProgress.value = { progress: 0, step: 0, episodeLen: 0 };
            inferenceHeatmaps.value = {};
            Notify.create({
                color: 'positive',
                message: t('inferenceStopped')
            });
        }
    });

    socket.on('record_episode_progress', (data) => {
        collectingProgress.value = data.progress;
    });

    socket.on('inference_progress', (data) => {
        inferenceProgress.value = {
            progress: Number(data.progress) || 0,
            step: Number(data.step) || 0,
            episodeLen: Number(data.episode_len) || 0,
        };
    });

    socket.on('moving_homepose', (data) => {
        movingHomepose.value = data.moving;
    });

    socket.on('episode_saved', () => {
        succeedFlag.value = false;
    });

    socket.on('episode_thrown', () => {
        succeedFlag.value = false;
    });

    // lerobot_append_episode 저장 중 — progress bar 대신 spinner 보여주기 위한 신호.
    socket.on('episode_saving', (data) => {
        savingEpisode.value = !!data.saving;
        savingEpisodeName.value = data.saving ? (data.name || '') : '';
    });

    socket.on('inference_succeed', (data) => {
        inferenceSucceed.value = data.succeed;
        succeedScore.value = data.score;
    });

    socket.on('ood_score', (data) => {
        oodScore.value = data;
    });

    socket.on('vive_node_ready', () => {
        viveInitializing.value = false;
    });

    socket.on('vive_node_error', (data) => {
        viveInitializing.value = false;
        Notify.create({
            color: 'negative',
            message: `${t('viveConnectFail')}: ${data?.message || t('unreadable')}`,
        });
    });

    socket.on('replay_progress', (data) => {
        replayProgress.value = data.progress;
    });

    socket.on('inference_vision_map', (data) => {
        // Backend emits {step, method, heatmaps:{sensor_X: data-url}} per step
        // when task_control['vision_map_method'] is set. Only render if the
        // user still has the toggle on (avoids stale frame appearing after
        // toggle-off but before the backend sees the null).
        if (!inferenceVisionMapOn.value) return;
        if (data && data.heatmaps) {
            inferenceHeatmaps.value = data.heatmaps;
        }
    });
});

onUnmounted(() => {
    socket.off('vive_node_ready');
    socket.off('vive_node_error');
    socket.off('inference_succeed');
    socket.off('ood_score');
    socket.off('inference_progress');
    socket.off('episode_saving');
    socket.off('episode_saved');
    socket.off('episode_thrown');
    socket.off('inference_vision_map');
    removeSucceedKeyListener();
});
</script>