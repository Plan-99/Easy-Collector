<template>
  <q-layout view="hHh Lpr lFf">
    <q-header elevated class="bg-secondary text-white">
      <q-toolbar>
        <q-btn
          flat
          dense
          round
          icon="menu"
          aria-label="Menu"
          @click="toggleLeftDrawer"
        />

        <q-toolbar-title>
          Easy Trainer
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
          Tutorial Mode
        </q-chip>
      </q-toolbar>
    </q-header>
    <q-drawer
      v-model="leftDrawerOpen"
      show-if-above
      dark
      class="q-pa-lg"
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
          <q-item class="border-rounded bg-dark">
            <q-item-section avatar>
              <q-icon name="school" :color="tutorial.running ? 'primary' : 'grey-5'" />
            </q-item-section>
            <q-item-section>
              <q-item-label class="text-white">Tutorial Mode</q-item-label>
              <q-item-label caption>
                <span v-if="tutorial.busy" class="text-orange">Switching…</span>
                <span v-else-if="tutorial.running" class="text-primary">Sim running</span>
                <span v-else class="text-grey-5">Off</span>
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
  </q-layout>
</template>

<script setup>
import { ref } from 'vue'
import { Notify } from 'quasar'
import EssentialLink from 'components/v2/EssentialLink.vue'
import { useTutorialStore } from 'src/stores/tutorialStore.js'

const linksList = [
  {
    title: 'Sensors',
    icon: 'camera',
    link: '/sensors'
  },
  {
    title: 'Robots',
    icon: 'adb',
    children: [
      {
        title: 'Management',
        icon: 'file_upload',
        link: '/robots/management'
      },
      {
        title: 'Assemble',
        icon: 'build',
        link: '/robots/assemble'
      }
    ]
  },
  {
    title: 'Workspace',
    icon: 'cleaning_services',
    link: '/workspace'
  },
  {
    title: 'Train',
    icon: 'school',
    link: '/train'
  }
]

const leftDrawerOpen = ref(false)
const tutorial = useTutorialStore()

function toggleLeftDrawer () {
  leftDrawerOpen.value = !leftDrawerOpen.value
}

async function onTutorialToggle (next) {
  try {
    if (next) {
      await tutorial.start()
      Notify.create({
        type: 'positive',
        message: 'Tutorial world is starting up — robot/sensor topics will appear shortly.',
        timeout: 4000,
      })
    } else {
      await tutorial.stop()
      Notify.create({
        type: 'info',
        message: 'Tutorial world stopped.',
        timeout: 2500,
      })
    }
  } catch (e) {
    Notify.create({
      type: 'negative',
      message: 'Tutorial mode toggle failed: ' + (e?.response?.data?.message || e.message || e),
      timeout: 5000,
    })
  }
}
</script>
