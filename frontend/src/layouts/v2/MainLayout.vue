<template>
  <q-layout view="hHh Lpr lFf">
    <q-header elevated class="bg-secondary text-white">
      <q-toolbar>
        <q-btn
          flat
          dense
          round
          icon="menu"
          :aria-label="$t('ariaMenu')"
          @click="toggleLeftDrawer"
        />

        <q-toolbar-title>
          {{ $t('appTitle') }}
        </q-toolbar-title>
        <q-space></q-space>

        <q-chip
          v-if="tutorial.running"
          color="primary"
          text-color="dark"
          icon="school"
          size="sm"
          class="q-mr-sm"
        >
          {{ $t('tutorialModeLabel') }}
        </q-chip>

        <q-btn
          flat
          dense
          round
          icon="view_in_ar"
          :color="demo.runningDemoId ? 'primary' : 'white'"
          class="q-mr-xs"
          :aria-label="$t('simActivationTitle')"
          @click="showSim = true"
        >
          <q-tooltip>{{ $t('simActivationTitle') }}</q-tooltip>
        </q-btn>

        <q-btn
          flat
          dense
          round
          icon="memory"
          color="white"
          class="q-mr-xs"
          :aria-label="$t('gpuManagerTitle')"
          @click="showGpu = true"
        >
          <q-tooltip>{{ $t('gpuManagerTitle') }}</q-tooltip>
        </q-btn>

        <q-btn-dropdown
          flat
          dense
          no-caps
          icon="translate"
          :label="currentLocaleLabel"
          color="white"
          class="q-mr-sm"
          content-class="bg-dark text-white"
          dropdown-icon="arrow_drop_down"
        >
          <q-list dark class="bg-dark text-white" style="min-width: 140px;">
            <q-item
              v-for="opt in localeOptions"
              :key="opt.value"
              clickable
              v-close-popup
              :active="opt.value === currentLocale"
              active-class="bg-primary text-dark"
              @click="onLocaleChange(opt.value)"
            >
              <q-item-section>
                <q-item-label>{{ opt.label }}</q-item-label>
              </q-item-section>
              <q-item-section side v-if="opt.value === currentLocale">
                <q-icon name="check" size="xs" :color="opt.value === currentLocale ? 'dark' : 'white'" />
              </q-item-section>
            </q-item>
          </q-list>
        </q-btn-dropdown>
      </q-toolbar>
    </q-header>
    <q-drawer
      v-model="leftDrawerOpen"
      show-if-above
      dark
      class="q-pa-md"
    >
      <div class="full-height bg-secondary border-rounded column">
        <q-list class="q-pa-md col">
          <EssentialLink
            v-for="link in linksList"
            :key="link.title"
            v-bind="link"
          />
        </q-list>

        <q-separator dark class="q-mx-md" />

        <div class="q-pa-md">
          <q-btn
            class="border-rounded bg-dark q-pa-md full-width q-mb-md text-left"
            flat
            no-caps
            @click="showPipelineGuide = true"
          >
            <div class="row items-center no-wrap full-width">
              <q-icon name="help_outline" color="primary" size="md" class="q-mr-md" />
              <div class="column items-start">
                <div class="text-white">{{ $t('pipelineGuideTitle') }}</div>
                <div class="text-caption text-grey-5">{{ $t('pipelineGuideSubtitle') }}</div>
              </div>
            </div>
          </q-btn>
          <q-item class="border-rounded bg-dark">
            <q-item-section avatar>
              <q-icon name="school" :color="tutorial.running ? 'primary' : 'grey-5'" />
            </q-item-section>
            <q-item-section>
              <q-item-label class="text-white">{{ $t('tutorialModeLabel') }}</q-item-label>
              <q-item-label caption>
                <span v-if="tutorial.busy" class="text-orange">{{ $t('tutorialSwitching') }}</span>
                <span v-else-if="tutorial.running" class="text-primary">{{ $t('tutorialSimRunning') }}</span>
                <span v-else class="text-grey-5">{{ $t('tutorialOff') }}</span>
              </q-item-label>
            </q-item-section>
            <q-item-section side>
              <q-toggle
                :model-value="tutorial.running"
                color="primary"
                :disable="tutorial.busy"
                @update:model-value="onTutorialToggle"
              />
            </q-item-section>
          </q-item>
          <div v-if="tutorial.lastError" class="text-negative text-caption q-mt-xs q-px-sm">
            {{ tutorial.lastError }}
          </div>
        </div>
      </div>
    </q-drawer>

    <q-page-container class="bg-dark">
      <router-view />
    </q-page-container>

    <PipelineGuideDialog v-model="showPipelineGuide" />
    <GpuManagerDialog v-model="showGpu" />
    <SimActivationDialog v-model="showSim" />
  </q-layout>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue'
import { Notify } from 'quasar'
import { useI18n } from 'vue-i18n'
import EssentialLink from 'components/v2/EssentialLink.vue'
import PipelineGuideDialog from 'components/v2/PipelineGuideDialog.vue'
import GpuManagerDialog from 'components/v2/GpuManagerDialog.vue'
import SimActivationDialog from 'components/v2/SimActivationDialog.vue'
import { useTutorialStore } from 'src/stores/tutorialStore.js'
import { useDemoStore } from 'src/stores/demoStore.js'
import { setLocale } from 'src/boot/i18n'

const { t, locale } = useI18n()

const localeOptions = computed(() => [
  { value: 'en-US', label: t('languageEnglish') },
  { value: 'ko-KR', label: t('languageKorean') },
])

const currentLocale = computed(() => locale.value)
const currentLocaleLabel = computed(() => {
  const opt = localeOptions.value.find((o) => o.value === locale.value)
  return opt ? opt.label : locale.value
})

function onLocaleChange (next) {
  if (next === locale.value) return
  setLocale(next)
}

const linksList = computed(() => [
  {
    title: t('navSensors'),
    icon: 'camera',
    link: '/sensors'
  },
  {
    title: t('navRobots'),
    icon: 'adb',
    children: [
      {
        title: t('navManagement'),
        icon: 'file_upload',
        link: '/robots/management'
      },
      {
        title: t('navAssemble'),
        icon: 'build',
        link: '/robots/assemble'
      },
      {
        title: t('navTeleoperation'),
        icon: 'sports_esports',
        link: '/robots/teleoperation'
      }
    ]
  },
  {
    title: t('navWorkspace'),
    icon: 'cleaning_services',
    link: '/workspace'
  },
  {
    title: t('navTrain'),
    icon: 'school',
    link: '/train'
  },
  {
    title: t('navDatasets'),
    icon: 'dataset',
    link: '/datasets'
  },
  {
    title: t('navPlanner'),
    icon: 'event_note',
    link: '/planner'
  },
  {
    title: t('navCurriculum'),
    icon: 'auto_mode',
    link: '/curriculum'
  }
])

const leftDrawerOpen = ref(false)
const showPipelineGuide = ref(false)
const showGpu = ref(false)
const showSim = ref(false)
const tutorial = useTutorialStore()
const demo = useDemoStore()

onMounted(() => {
  // Reflect a demo sim that's already running (e.g. after a page reload) in the
  // top-bar button color without waiting for the dialog to be opened.
  demo.refreshStatus()
})

function toggleLeftDrawer () {
  leftDrawerOpen.value = !leftDrawerOpen.value
}

async function onTutorialToggle (next) {
  try {
    if (next) {
      await tutorial.start()
      Notify.create({
        type: 'positive',
        message: t('tutorialStartedNotify'),
        timeout: 4000,
      })
    } else {
      await tutorial.stop()
      Notify.create({
        type: 'info',
        message: t('tutorialStoppedNotify'),
        timeout: 2500,
      })
    }
  } catch (e) {
    Notify.create({
      type: 'negative',
      message: t('tutorialToggleFailed', { error: e?.response?.data?.message || e.message || e }),
      timeout: 5000,
    })
  }
}
</script>
