<template>
  <router-view />
</template>

<script setup>
import { onMounted } from 'vue';
import { useProcessStore } from 'src/stores/processStore';
import { useTopicStore } from 'src/stores/topicStore';
import { api } from 'src/boot/axios';

const processStore = useProcessStore();
const topicStore = useTopicStore();

function cleanup() {
  processStore.processIds.forEach((processId) => {
    if (processId.includes('read_dataset')) {
      api.post('/stop_process', { name: processId })
    }
    if (processId.includes('leader_teleoperation')) {
      api.post('/stop_process', { name: processId })
    }
    if (processId.includes('record_episode')) {
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

  // 활성 ROS2 토픽 push 구독 — useRobot/useSensor가 이 store를 watch하여
  // 토픽 가시성 변화에 자동 반응한다.
  topicStore.initialize().then(() => {
    console.log('Topic store initialized');
  }).catch((error) => {
    console.error('Error initializing topic store:', error);
  });

  // onMounted(() => {
  //   window.addEventListener('beforeunload', cleanup);
  // });
});
</script>
