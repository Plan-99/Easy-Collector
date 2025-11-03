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
      </q-toolbar>
    </q-header>
    <q-drawer
      v-model="leftDrawerOpen"
      show-if-above
      dark
      class="q-pa-lg"
    >
      <div class="full-height bg-secondary border-rounded">
        <q-list class="q-pa-md">
          <EssentialLink
            v-for="link in linksList"
            :key="link.title"
            v-bind="link"
          />
        </q-list>
      </div>
    </q-drawer>

    <q-page-container class="bg-dark">
      <router-view />
    </q-page-container>
  </q-layout>
</template>

<script setup>
import { ref } from 'vue'
import EssentialLink from 'components/EssentialLink.vue'

const linksList = [
  {
    title: 'Sensors',
    icon: 'camera',
    link: '/v2/sensors'
  },
  {
    title: 'Robots',
    icon: 'adb',
    link: '/v2/robots'
  },
  {
    title: 'Tasks',
    icon: 'cleaning_services',
    link: '/v2/tasks'
  },
  // {
  //   title: 'VLA',
  //   icon: 'psychology',
  //   link: '/vla/execute'
  // }
  // {
  //   title: 'Datasets',
  //   icon: 'folder_open',
  //   link: '/datasets'
  // },
]

const leftDrawerOpen = ref(false)

function toggleLeftDrawer () {
  leftDrawerOpen.value = !leftDrawerOpen.value
}

</script>
