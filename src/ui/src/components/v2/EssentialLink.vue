<template>
  <div v-if="children && children.length > 0">
    <q-expansion-item
      :icon="icon"
      :label="title"
      :caption="caption"
      :default-opened="hasActiveChild"
      class="border-rounded overflow-hidden"
      header-class="text-white"
    >
      <q-item
        v-for="child in children"
        :key="child.title"
        clickable
        :to="child.link"
        class="q-pl-xl border-rounded"
        :class="{ 'bg-primary text-dark': isChildActive(child.link) }"
      >
        <q-item-section v-if="child.icon" avatar>
          <q-icon :name="child.icon" />
        </q-item-section>
        
        <q-item-section>
          <q-item-label>{{ child.title }}</q-item-label>
          <q-item-label caption>{{ child.caption }}</q-item-label>
        </q-item-section>
      </q-item>
    </q-expansion-item>
  </div>

  <q-item
    v-else
    clickable
    :to="link"
    class="border-rounded"
    :class="{ 'bg-primary text-dark': isActive }"
  >
    <q-item-section
      v-if="icon"
      avatar
    >
      <q-icon :name="icon" />
    </q-item-section>

    <q-item-section>
      <q-item-label>{{ title }}</q-item-label>
      <q-item-label caption>{{ caption }}</q-item-label>
    </q-item-section>
  </q-item>
</template>

<script setup>
import { computed } from 'vue'
import { useRoute } from 'vue-router'

const props = defineProps({
  title: {
    type: String,
    required: true
  },

  caption: {
    type: String,
    default: ''
  },

  link: {
    type: String,
    default: '#'
  },

  icon: {
    type: String,
    default: ''
  },

  // 하위 메뉴를 위한 prop 추가
  children: {
    type: Array,
    default: () => []
  }
})

const route = useRoute()

// 기존 isActive 로직 (단일 링크용)
const isActive = computed(() => {
  if (props.link === '/tasks') {
    return route.path.startsWith('/tasks')
  }
  return route.path === props.link
})

// 하위 메뉴 중 활성화된 것이 있는지 확인 (메뉴 자동 펼침용)
const hasActiveChild = computed(() => {
  return props.children.some(child => child.link === route.path)
})

// 하위 메뉴 각각의 활성화 여부 체크 함수
function isChildActive (link) {
  return route.path === link
}
</script>