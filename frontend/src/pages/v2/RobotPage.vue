<template>
    <q-page class="q-pt-lg q-pr-lg full-height">
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="text-h5 text-primary text-bold q-mb-lg">{{ $t('robotIntroTitle') }}</div>
                <div class="text-body text-white">{{ $t('robotIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('robotIntroBody2') }}</div>
            </div>
        </div>

        <TutorialHint class="q-mb-md" :text="$t('tutorialRobotIntro')" />

        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="robot in robots" :key="robot.id">
                <q-card class="q-pa-md bg-secondary border-rounded border-white text-white full-height" :class="watchingRobot && watchingRobot.status === 'on' && robot.id === watchingRobot.id ? 'border-primary' : ''">
                    <q-menu context-menu>
                        <q-list bordered separator>
                            <q-item clickable v-ripple v-close-popup @click="openEditRobotForm(robot)">
                                <q-item-section>{{ $t('robotContextEdit') }}</q-item-section>
                                <q-item-section side>
                                    <q-icon name="edit" size="xs" />
                                </q-item-section>
                            </q-item>
                            <q-item clickable v-ripple class="text-negative" @click="deleteRobot(robot)">
                                <q-item-section>{{ $t('robotContextHide') }}</q-item-section>
                                <q-item-section side>
                                    <q-icon color="negative" name="visibility" size="xs" />
                                </q-item-section>
                            </q-item>
                        </q-list>
                    </q-menu>
                    <q-img :src="robot.image" @click="watchRobot(robot)" class="cursor-pointer bg-white" ratio="1.5" fit="contain">
                    </q-img>

                    <q-card-section class="q-pa-none q-mt-sm">
                        <div class="text-bold">{{ robot.name }}</div>
                        <div class="text-grey-6 text-caption">{{ robot.type }}</div>
                    </q-card-section>
                    <q-card-section class="q-pa-none q-mt-sm row" v-if="robot.type !== 'custom'">
                        <div class="text-primary text-caption" v-if="robot.status === 'on'">{{ $t('statusOnline') }}</div>
                        <div class="text-orange text-caption" v-else-if="robot.status === 'loading'">{{ $t('statusLoading') }}</div>
                        <div class="text-negative text-caption" v-else-if="robot.status === 'error'">{{ $t('statusError') }}</div>
                        <div class="text-grey-7 text-caption" v-else>{{ $t('statusOffline') }}</div>
                        <q-space></q-space>
                        <q-toggle
                            :model-value="robot.status === 'on'"
                            color="primary"
                            dense
                            @click.stop="toggleRobot(robot)"
                        />
                    </q-card-section>
                    <q-card-section class="q-pa-none q-mt-sm row" v-else>
                        <div class="text-primary text-caption" v-if="robot.status === 'on'">{{ $t('topicOn') }}</div>
                        <div class="text-negative text-caption" v-else-if="robot.status === 'error'">{{ $t('statusError') }}</div>
                        <div class="text-grey-7 text-caption" v-else-if="robot.status === 'off'">{{ $t('topicOff') }}</div>
                        <div class="text-grey-7 text-caption" v-else>{{ $t('statusLoading') }}</div>
                    </q-card-section>
                    <TutorialHint
                        v-if="tutorial.running && tutorial.robotId === robot.id"
                        step="1"
                        class="q-mt-sm"
                        :text="$t('tutorialRobotCard')"
                    />
                    <q-inner-loading :showing="robot.status === 'loading'">
                        <q-spinner-gears size="50px" color="primary" />
                    </q-inner-loading>
                </q-card>
            </div>
            <div class="col" v-if="!robots.length">
                <q-card class="full-height border-rounded border-white bg-dark text-white" flat bordered>
                    <q-card-section class="text-center">
                        <div class="text-h6">{{ $t('noRobotTitle') }}</div>
                        <div class="text-subtitle2">{{ $t('noRobotBody') }}</div>
                    </q-card-section>
                </q-card>
            </div>
            <div class="col-6 col-sm-4 col-md-3  col-lg-2" :style="!robots.length ? 'min-height: 220px;' : ''" >
                <q-btn color="white" class="full-height full-width border-rounded" outline size="lg" icon="add" @click="openAddSensorForm"></q-btn>
                <TutorialHint step="2" class="q-mt-sm" :text="$t('tutorialRobotAdd')" />
            </div>
        </div>

        <bottom-terminal
            :tabs="robots"
            tab-label="name"
            tab-value="id"
            v-model="watchingRobot"
            v-if="watchingRobot"
            @update:model-value="watchRobot($event)"
        >
            <template v-for="robot in robots" :key="robot.id" v-slot:[robot.id]>
                <div class="row row q-gutter-x-md">
                    <div class="col-4">
                        <TutorialHint class="q-mb-sm" :text="$t('tutorialRobotPendant')" />
                        <robot-pendant
                            :robot="watchingRobot"
                        />
                    </div>
                    <!-- SimView: 시뮬레이션 안정화 후 활성화
                    <div class="col-3" v-if="watchingRobot && watchingRobot.type !== 'custom'" style="min-height: 300px;">
                        <SimView :robot-id="watchingRobot.id" :auto-start="true" />
                    </div>
                    -->
                    <div class="col">
                        <process-console
                            :process="robot.process_id"
                            style="height: 100%; min-height: 300px;"
                        />
                    </div>
                </div>

            </template>
        </bottom-terminal>
        <form-dialog
            v-model="showRobotForm"
            :title="$t(robotForm.find((e) => e.key === 'id').value ? 'robotEditFormTitle' : 'robotAddFormTitle')"
            :form="robotForm"
            @submit="saveRobot"
            :ok-button-label="$t(robotForm.find((e) => e.key === 'id').value ? 'save' : 'add')"
        >
            <TutorialHint class="q-mb-md" :text="$t('tutorialRobotForm')" />
            <template v-slot:joint_names>
                <div class="row q-mb-md q-col-gutter-sm">
                    <div
                        class="col-3"
                        v-for="(joint, i) in robotForm.find((e) => e.key === 'joint_names').value"
                        :key="i"
                    >
                        <q-input
                            outlined
                            dense
                            dark
                            bg-color="dark"
                            v-model="robotForm.find((e) => e.key === 'joint_names').value[i]"
                        >
                            <template v-slot:append>
                                <q-icon size="xs" name="close" @click="removeJoint(i)" class="cursor-pointer" />
                            </template>
                        </q-input>
                        <q-checkbox
                            v-model="robotForm.find((e) => e.key === 'tool_index').value"
                            :val="i"
                            dense
                            class="q-ml-sm text-white"
                            size="xs"
                            dark
                            :label="$t('robotToolJoint')"
                            v-if="robotForm.find((e) => e.key === 'role').value !== 'tool'"
                        ></q-checkbox>
                    </div>

                    <div class="col-3">
                        <q-btn
                            dense
                            outline
                            color="primary"
                            :label="$t('robotAddJoint')"
                            class="full-width full-height"
                            @click="addJoint"
                        ></q-btn>

                    </div>
                </div>
            </template>
            <template
                v-slot:joint_lower_bounds
            >
                <div class="row q-mb-md q-col-gutter-sm">
                    <q-input
                        outlined
                        dense
                        dark
                        bg-color="dark"
                        v-model.number="robotForm.find((e) => e.key === 'joint_lower_bounds').value[i]"
                        class="col-3"
                        v-for="(joint, i) in robotForm.find((e) => e.key === 'joint_lower_bounds').value"
                        :key="i"
                    ></q-input>
                </div>
            </template>
            <template
                v-slot:joint_upper_bounds
            >
                <div class="row q-mb-md q-col-gutter-sm">
                    <q-input
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model.number="robotForm.find((e) => e.key === 'joint_upper_bounds').value[i]"
                        class="col-3"
                        v-for="(joint, i) in robotForm.find((e) => e.key === 'joint_upper_bounds').value"
                        :key="i"
                    ></q-input>
                </div>
            </template>
            <template v-slot:ik_json>
                <q-input
                    outlined dark bg-color="dark"
                    v-model="robotForm.find((e) => e.key === 'ik_json').value"
                    type="textarea"
                    rows="10"
                    class="q-mb-md q-mt-xs"
                    :placeholder="$t('robotIkPlaceholder')"
                    :error="ikJsonError !== ''"
                    :error-message="ikJsonError"
                    :hint="$t('robotIkHint')"
                    style="font-family: monospace;"
                />
            </template>
        </form-dialog>
        <tele-setting-dialog
            v-if="showTeleSetting"
            v-model="showTeleSetting" 
            :robot="teleSettingRobot"
            @hide="closeTeleSetting"
        />
    </q-page>
</template>

<script setup>
import { onMounted, onUnmounted, ref, watch, computed } from 'vue';

import { useSocket } from 'src/composables/useSocket';
import { useLeaderTeleoperation } from 'src/composables/useLeaderTeleoperation';
import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/v2/ProcessConsole.vue';
import TeleSettingDialog from 'src/components/v2/TeleSettingDialog.vue';
import { Notify } from 'quasar';
import BottomTerminal from 'src/components/v2/BottomTerminal.vue';
import { useRobot } from 'src/composables/useRobot';
import FormDialog from 'src/components/v2/FormDialog.vue';
import RobotPendant from 'src/components/v2/RobotPendant.vue';
import TutorialHint from 'src/components/v2/TutorialHint.vue';
import { useTutorialStore } from 'src/stores/tutorialStore.js';
import { useI18n } from 'vue-i18n';
// import SimView from 'src/components/v2/SimView.vue'; // 시뮬레이션 안정화 후 활성화

const { t } = useI18n();
const { socket } = useSocket();
const { leaderTeleStarted, stopLeaderTele } = useLeaderTeleoperation();
const tutorial = useTutorialStore();

const robots = ref([]);

const supportingRobots = ref([])

function getSupportingRobots() {
    return api.get('/robots:supporting').then((response) => {
        supportingRobots.value = [
            ...response.data.robots,
            {
                name: 'custom',
                company: null,
            }
        ]
    }).catch((error) => {
        console.error('Error fetching supporting robots:', error);
    });
}

function getFormRobotInfo(form) {
    return supportingRobots.value.find(r => r.name === form.find((e) => e.key === 'type').value);
}

const robotForm = ref([
    { key: 'id', value: null },
    { label: t('robotName'), key: 'name', type: 'text', value: '', default: '' },
    { label: t('robotType'), key: 'type', type: 'select', value: '', default: '', options: computed(() => supportingRobots.value.map((robot) => ({
        label: robot.name + (robot.company ? ` (${robot.company})` : ''),
        value: robot.name
    }))) },
    // Custom fields based on robot type
    { label: t('robotFieldCanPort'), key: 'can_port', type: 'text', value: 'can0', default: 'can0', show: (form) => getFormRobotInfo(form) && getFormRobotInfo(form).custom_fields && getFormRobotInfo(form).custom_fields && getFormRobotInfo(form).custom_fields.includes('can_port') },
    { label: t('ipAddress'), key: 'ip_address', type: 'text', value: '10.0.2.27', default: '10.0.2.27', show: (form) => getFormRobotInfo(form) && getFormRobotInfo(form).custom_fields && getFormRobotInfo(form).custom_fields.includes('ip_address') },
    { label: t('robotFieldPort'), key: 'port', type: 'number', value: 502, default: 502, show: (form) => getFormRobotInfo(form) && getFormRobotInfo(form).custom_fields && getFormRobotInfo(form).custom_fields.includes('port') },
    { label: t('robotFieldChangerAddress'), key: 'changer_address', type: 'number', value: 5, default: 5, show: (form) => getFormRobotInfo(form) && getFormRobotInfo(form).custom_fields && getFormRobotInfo(form).custom_fields.includes('changer_address') },
    { label: t('robotFieldSerialPort'), key: 'serial_port', type: 'text', value: '/dev/ttyUSB0', default: '/dev/ttyUSB0', show: (form) => getFormRobotInfo(form) && getFormRobotInfo(form).custom_fields && getFormRobotInfo(form).custom_fields.includes('serial_port') },
    // { label: 'tool_inner', key: 'tool_inner', type: 'custom', value: computed(() => robotForm.value.find((e) => e.key === 'tool_index').length > 0), default: false, show: () => false },
    // Fields for custom robot
    { label: t('robotFieldRole'), key: 'role', type: 'select', value: 'single_arm', default: 'dual_arm',
        options: [
            { label: t('robotRoleSingleArm'), value: 'single_arm' },
            { label: t('robotRoleDualArm'), value: 'dual_arm' },
            { label: t('robotRoleTool'), value: 'tool' },
        ],
        show: (form) => form.find((e) => e.key === 'type').value === 'custom'
    },
    { label: t('robotFieldSimulation'), key: 'is_sim', type: 'checkbox', value: false, default: false, show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: t('robotFieldJointNames'), key: 'joint_names', type: 'custom', value: [], default: [] , show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: '', key: 'tool_index', type: 'custom', value: [], default: [], show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: t('robotFieldJointLowerBounds'), key: 'joint_lower_bounds', type: 'custom', value: [], default: [] , show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: t('robotFieldJointUpperBounds'), key: 'joint_upper_bounds', type: 'custom', value: [], default: [] , show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: t('robotFieldReadTopic'), key: 'read_topic', type: 'text', value: '', default: '', show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: t('robotFieldReadTopicMsg'), key: 'read_topic_msg', type: 'select', value: '', default: 'sensor_msgs/JointState',
        options: [
            { label: 'sensor_msgs/JointState', value: 'sensor_msgs/JointState' },
        ],
        show: (form) => form.find((e) => e.key === 'type').value === 'custom'
    },
    { label: t('robotFieldWriteType'), key: 'write_type', type: 'select', value: '', default: 'topic',
        options: [
            { label: 'Topic', value: 'topic' },
            { label: 'Service', value: 'service' },
            { label: 'Action', value: 'action' },
        ],
        show: (form) => form.find((e) => e.key === 'type').value === 'custom'
    },
    { label: t('robotFieldWriteTopic'), key: 'write_topic', type: 'text', value: '', default: '', show: (form) => form.find((e) => e.key === 'type').value === 'custom' },
    { label: t('robotFieldWriteTopicMsg'), key: 'write_topic_msg', type: 'select', value: '', default: 'sensor_msgs/JointState',
        options: [
            { label: 'sensor_msgs/JointState', value: 'sensor_msgs/JointState' },
            { label: 'std_msgs/Float64MultiArray', value: 'std_msgs/Float64MultiArray' },
            { label: 'control_msgs/action/GripperCommand', value: 'control_msgs/action/GripperCommand' },
        ],
        show: (form) => form.find((e) => e.key === 'type').value === 'custom'
    },
    // custom robot 은 외부 ROS2 노드가 자체적으로 명령 토픽을 처리한다는 전제이므로
    // interpolation_node 를 끼워넣지 않는다. 평활화가 필요하면 외부 노드에서 처리.
    { label: t('robotFieldIkSettings'), key: 'ik_json', type: 'custom', value: '', default: '', optional: true,
        show: (form) => form.find((e) => e.key === 'type').value === 'custom'
    },
]);
const watchingRobot = ref(null);

function listRobots() {                                                                                                     
    return api.get('/robots').then((response) => {
        robots.value = response.data.robots || [];
        robots.value.forEach(robot => {
            robot.image = '/images/' + robot.company + '.png'; // Default image if not provided
            console.log(robot.image)

            robot.loading = false;
            robot.handler = useRobot(robot, () => {
            });
            // Topic visibility는 topicStore가 push로 추적하므로 type별 분기 불필요.
            // (custom robot의 외부 토픽도 동일 경로로 자동 감지)
            robot.joint_pos = []
            robot.joint_names.forEach((joint, i) => {
                robot.joint_pos[i] = 0
            })
        });
    }).catch((error) => {
        console.error('Error fetching robots:', error);
    });
}

function openAddSensorForm() {
    robotForm.value.forEach(field => {
        field.value = field.default;
    });
    showRobotForm.value = true;
}

function openEditRobotForm(robot) {
    ikJsonError.value = ''
    robotForm.value.forEach(field => {
        field.value = robot[field.key] || field.default;
    });
    robotForm.value.find((e) => e.key === 'id').value = robot.id;
    // Load existing IK settings as JSON string for custom robots
    if (robot.type === 'custom' && robot.settings && robot.settings.ik_setting) {
        const ikObj = {
            urdf_path: robot.settings.urdf_path || '',
            urdf_package_dir: robot.settings.urdf_package_dir || '',
            ik_setting: robot.settings.ik_setting,
        }
        robotForm.value.find((e) => e.key === 'ik_json').value = JSON.stringify(ikObj, null, 2)
    }
    showRobotForm.value = true;
}

function addJoint() {
    const jointNamesField = robotForm.value.find((e) => e.key === 'joint_names');
    const jointLowerBoundsField = robotForm.value.find((e) => e.key === 'joint_lower_bounds');
    const jointUpperBoundsField = robotForm.value.find((e) => e.key === 'joint_upper_bounds');
    jointNamesField.value.push('');
    jointLowerBoundsField.value.push(-3.14);
    jointUpperBoundsField.value.push(3.14);
}

function removeJoint(index) {
    const jointNamesField = robotForm.value.find((e) => e.key === 'joint_names');
    const jointLowerBoundsField = robotForm.value.find((e) => e.key === 'joint_lower_bounds');
    const jointUpperBoundsField = robotForm.value.find((e) => e.key === 'joint_upper_bounds');
    jointNamesField.value.splice(index, 1);
    jointLowerBoundsField.value.splice(index, 1);
    jointUpperBoundsField.value.splice(index, 1);
}

function saveRobot(formData) {
    ikJsonError.value = ''
    if (formData.ik_json && typeof formData.ik_json === 'string') {
        try {
            formData.ik_json = JSON.parse(formData.ik_json)
        } catch {
            ikJsonError.value = t('robotIkInvalidJson')
            showRobotForm.value = true
            return
        }
    }
    if (robotForm.value.find((e) => e.key === 'id').value) {
        return api.put(`/robot/${formData.id}`, formData).then(() => {
            robotForm.value.forEach(field => field.value = field.default);
            listRobots()
        })
    }
    return api.post(`/robot`, formData).then(() => {
        robotForm.value.forEach(field => field.value = field.default);
        listRobots()
    })
}

function deleteRobot(robot) {
    if (robot.status === 'on') {
        Notify.create({
            color: 'negative',
            message: t('errorTurnOffRobotFirst')
        })
        return;
    }
    return api.delete(`/robot/${robot.id}`).then(() => {
        initialize()
    })
}

function toggleRobot(robot) {
    const startFlow = () => {
        robot.handler.startRobot().then(() => {
            watchRobot(robot); // Start watching the robot after it is started
        });
    };

    if (robot.status === 'on') {
        robot.handler.stopRobot().then(() => {
            // Keep watching to preserve logs/pendant view
        });
    } else if (robot.status === 'error') {
        // clean up lingering processes then retry start
        robot.handler.stopRobot().finally(() => {
            startFlow();
        });
    } else {
        startFlow();
    }
}

function watchRobot(robot) {
    canControl.value = false;
    if (!robot) {
        return;
    }
    robot.handler.subscribeRobot((js) => {
        if (js) {
            robot.joint_pos = js
            if (!canControl.value) {
                canControl.value = true
            }
        }
    })
    watchingRobot.value = robot
}

const canControl = ref(false)
const showRobotForm = ref(false)
const ikJsonError = ref('')

const showTeleSetting = ref(false)
const teleSettingRobot = ref(null)


watch(watchingRobot, (newVal, oldVal) => {
    if (oldVal) {
        stopLeaderTele(); // Stop leader teleoperation when switching robots
    }
});


function initialize() {
    listRobots()
    getSupportingRobots()
}

onMounted(() => {
    socket.on('start_process', (data) => {
        if (data.id === 'leader_teleoperation') {
            leaderTeleStarted.value = true;
        }
    });

    socket.on('stop_process', (data) => {
        if (data.id === 'leader_teleoperation') {
            leaderTeleStarted.value = false;
        }
    });


    initialize()
})

onUnmounted(() => {
    if (watchingRobot.value) {
        watchingRobot.value.handler.unSubscribeRobot();
    }
});

</script>
