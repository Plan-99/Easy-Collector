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
        <!-- <q-btn
          flat
          dense
          round
          icon="refresh"
          @click="$router.go()"
        ></q-btn> -->
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
import EssentialLink from 'components/v2/EssentialLink.vue' // 경로 주의 (components/v2/... 라면 그에 맞게 수정)

const linksList = [
  {
    title: 'Sensors',
    icon: 'camera',
    link: '/sensors'
  },
  {
    title: 'Robots',
    icon: 'adb',
    // link: '/robots', // 상위 메뉴는 링크를 제거하거나 '#'으로 둡니다.
    children: [
      {
        title: 'Management',
        icon: 'file_upload', // 적절한 아이콘 예시
        link: '/robots/management'
      },
      {
        title: 'Assemble',
        icon: 'build', // 적절한 아이콘 예시
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

function toggleLeftDrawer () {
  leftDrawerOpen.value = !leftDrawerOpen.value
}


</script>