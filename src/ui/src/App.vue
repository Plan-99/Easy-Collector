<template>
  <router-view />
</template>

<script setup>
import { onMounted } from 'vue';
import { useProcessStore } from './stores/processStore';

const processStore = useProcessStore();

onMounted(() => {
  processStore.initialize();
});
</script>
