import { defineStore } from 'pinia';
import { ref } from 'vue';
import { api } from 'boot/axios';
import { useSocket } from 'src/composables/useSocket';

export const useProcessStore = defineStore('process', () => {
  const processIds = ref([]);
  const { socket } = useSocket();

  // Action to fetch initial processes
  async function fetchProcesses() {
    try {
      const response = await api.get('/processes');
      processIds.value = response.data.processes || [];
    } catch (error) {
      console.error('Failed to fetch processes:', error);
      processIds.value = [];
    }
  }

  // Action to add a process ID
  function addProcess(processId) {
    if (!processIds.value.includes(processId)) {
      processIds.value.push(processId);
    }
  }

  // Action to remove a process ID
  function removeProcess(processId) {
    processIds.value = processIds.value.filter((id) => id !== processId);
  }

  // Initialize and set up socket listeners
  function initialize() {

    socket.on('start_process', (data) => {
      if (data && data.id) {
        addProcess(data.id);
      }
    });

    socket.on('stop_process', (data) => {
      if (data && data.id) {
        removeProcess(data.id);
      }
    });

    return fetchProcesses();
  }

  // Getter to check if a process is running
  const isRunning = (processId) => {
    return processIds.value.includes(processId);
  };

  return {
    processIds,
    initialize,
    isRunning,
  };
});
