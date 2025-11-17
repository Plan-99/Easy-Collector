<template>
  <q-item
    clickable
    :to="link"
    :active="isActive"
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
  }
})

const route = useRoute()

const isActive = computed(() => {
  if (props.link === '/tasks') {
    return route.path.startsWith('/tasks')
  }
  return route.path === props.link
})
</script>

<style>
.q-router-link--active .q-item__label {
  color: blue;
}
</style>
