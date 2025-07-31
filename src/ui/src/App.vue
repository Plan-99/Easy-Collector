<template>
  <router-view />
</template>

<script setup>
import { onMounted } from 'vue';
import { useProcessStore } from 'src/stores/processStore';
import { api } from 'src/boot/axios';

const processStore = useProcessStore();

function cleanup() {
  processStore.processIds.forEach((processId) => {
    if (processId.includes('read_hdf5')) {
      api.post('/stop_process', { name: processId })
    }
    if (processId.includes('leader_teleoperation')) {
      api.post('/stop_process', { name: processId })
    }
  });
}

onMounted(() => {
  processStore.initialize().then(() => {
    console.log('Process store initialized');
    cleanup();
  }).catch((error) => {
    console.error('Error initializing process store:', error);
  });

  // onMounted(() => {
  //   window.addEventListener('beforeunload', cleanup);
  // });
});
</script>
