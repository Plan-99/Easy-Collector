<template>
    <q-page class="q-pt-md q-pr-md full-height">
        <div class="border-rounded bg-secondary q-pa-md q-mb-md row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="text-h5 text-primary text-bold q-mb-md">{{ $t('teleopIntroTitle') }}</div>
                <div class="text-body text-white">{{ $t('teleopIntroBody') }}</div>
            </div>
        </div>

        <TutorialHint class="q-mb-md" :text="$t('tutorialTeleopIntro')" />

        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="(assembly, idx) in assembliesWithRobots" :key="assembly.id">
                <q-card
                    class="q-pa-md bg-secondary border-rounded border-white text-white cursor-pointer"
                    :class="{ 'border-primary': watchingAssembly && watchingAssembly.id === assembly.id }"
                    @click="watchingAssembly = assembly"
                >
                    <q-card-section class="q-pa-none row items-center">
                        <div class="text-bold text-h6 col">{{ assembly.name }}</div>
                        <q-btn
                            flat round dense icon="settings" color="grey-4" size="sm"
                            @click.stop="openTeleopSetting(assembly)"
                        >
                            <q-tooltip>{{ $t('teleopSettingsTooltip') }}</q-tooltip>
                        </q-btn>
                    </q-card-section>
                    <q-card-section class="q-pa-none q-mt-md">
                        <div v-for="part in ['left_arm', 'left_tool', 'right_arm', 'right_tool']" :key="part">
                            {{ part }}
                            <span class="text-primary">{{ assembly[part]?.name }}</span>
                        </div>
                    </q-card-section>
                    <q-separator dark class="q-my-sm" />
                    <q-card-section class="q-pa-none row q-gutter-xs">
                        <q-icon name="keyboard" size="sm" color="primary">
                            <q-tooltip>{{ $t('teleopModeKeyboardTip') }}</q-tooltip>
                        </q-icon>
                        <q-icon
                            name="sports_esports"
                            size="sm"
                            :color="hasLeader(assembly) ? 'positive' : 'grey-7'"
                        >
                            <q-tooltip>{{ hasLeader(assembly) ? $t('teleopModeLeaderOnTip') : $t('teleopModeLeaderOffTip') }}</q-tooltip>
                        </q-icon>
                    </q-card-section>
                    <TutorialHint
                        v-if="idx === 0"
                        step="1"
                        class="q-mt-sm"
                        :text="$t('tutorialTeleopAssemblyCard')"
                    />
                </q-card>
            </div>
        </div>

        <bottom-terminal
            v-if="watchingAssembly"
            v-model="watchingAssembly"
            :tabs="assembliesWithRobots"
            tab-label="name"
            tab-value="id"
            @close="watchingAssembly = null"
        >
            <template v-slot:[assembly.id] v-for="assembly in assembliesWithRobots" :key="assembly.id">
                <teleop-console :assembly="assembly" />
            </template>
        </bottom-terminal>

        <tele-setting-dialog
            v-if="teleopSettingAssembly"
            v-model="showTeleopSetting"
            :assembly="teleopSettingAssembly"
            @hide="closeTeleopSetting"
        />
    </q-page>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { api } from 'src/boot/axios'
import BottomTerminal from 'src/components/v2/BottomTerminal.vue'
import TeleSettingDialog from 'src/components/v2/TeleSettingDialog.vue'
import TeleopConsole from 'src/components/v2/TeleopConsole.vue'
import TutorialHint from 'src/components/v2/TutorialHint.vue'
import { useRobot } from 'src/composables/useRobot'

const assemblies = ref([])
const robots = ref([])
const watchingAssembly = ref(null)

// Each robot is wrapped via useRobot() so it has handler.moveRobotEEDelta + jointState
const robotHandlers = new Map()

function ensureRobotHandler(robot) {
    if (robotHandlers.has(robot.id)) return robotHandlers.get(robot.id)
    // useRobot mutates robot in-place and auto-subscribes if status==='on'
    const handler = useRobot(robot)
    robot.handler = handler
    robotHandlers.set(robot.id, handler)
    return handler
}

const assembliesWithRobots = computed(() => {
    return assemblies.value.map((a) => {
        const out = { ...a }
        ;['left_arm', 'left_tool', 'right_arm', 'right_tool'].forEach((part) => {
            const id = a[part + '_id']
            const r = id ? robots.value.find((x) => x.id === id) : null
            out[part] = r || null
        })
        return out
    })
})

function hasLeader(assembly) {
    return !!assembly.teleoperators?.find((t) => t.type === 'leader')
}

function listAssemblies() {
    return api.get('/assemblies').then((res) => {
        assemblies.value = res.data.assemblies || []
    })
}

function listRobots() {
    return api.get('/robots').then((res) => {
        // 1) ref에 array를 먼저 할당해서 element가 reactive proxy가 되게 함
        robots.value = res.data.robots || []
        // 2) proxy 상태에서 handler 부착 — useRobot이 proxy를 closure에 가두므로
        //    이후 robot.status mutation이 reactive로 추적된다.
        robots.value.forEach((r) => ensureRobotHandler(r))
    })
}

const showTeleopSetting = ref(false)
const teleopSettingAssembly = ref(null)
function openTeleopSetting(assembly) {
    teleopSettingAssembly.value = assembly
    showTeleopSetting.value = true
}
function closeTeleopSetting() {
    showTeleopSetting.value = false
    // refresh assembly settings (teleoperators may have been edited)
    listAssemblies()
}

onMounted(async () => {
    await Promise.all([listRobots(), listAssemblies()])
})

onUnmounted(() => {
    robotHandlers.forEach((h) => {
        try {
            h.unSubscribeRobot && h.unSubscribeRobot()
        } catch {
            /* ignore */
        }
    })
    robotHandlers.clear()
})
</script>
