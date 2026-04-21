import { defineStore } from 'pinia';
import { ref } from 'vue';
import { api } from 'boot/axios';
import { useSocket } from 'src/composables/useSocket';

export const useProcessStore = defineStore('process', () => {
  const processIds = ref([]);
  const { socket } = useSocket();
  
  // 1. 로그를 저장할 상태 추가 (프로세스 ID별로 관리)
  const taskLogs = ref({});
  const initialized = ref(false);

  // 로그를 추가하는 Action
  function addLog(processId, message, type='stdout') {
    if (!taskLogs.value[processId]) {
      taskLogs.value[processId] = [];
    }
    // 최신 로그를 상단에 두거나 배열에 추가
    taskLogs.value[processId].push({
      message,
      type: type,
    });
    
    // (선택 사항) 로그가 너무 많아지면 오래된 로그 삭제
    if (taskLogs.value[processId].length > 100) {
      taskLogs.value[processId].shift();
    }
  }

  async function fetchProcesses() {
    try {
      const response = await api.get('/processes');
      processIds.value = response.data.processes || [];
    } catch (error) {
      console.error('Failed to fetch processes:', error);
      processIds.value = [];
    }
  }

  function addProcess(processId) {
    if (!processIds.value.includes(processId)) {
      processIds.value.push(processId);
    }
  }

  function removeProcess(processId) {
    processIds.value = processIds.value.filter((id) => id !== processId);
  }

  // 2. initialize 함수에 task_log 리스너 추가
  function initialize() {
    if (initialized.value) return Promise.resolve();

    socket.on('start_process', (data) => {
      if (data && data.id) addProcess(data.id);
    });

    socket.on('stop_process', (data) => {
      if (data && data.id) removeProcess(data.id);
    });

    // --- 추가된 부분 ---
    socket.on('task_log', (data) => {
      // 서버에서 { id: 'process_1', log: '처리 중...' } 형태로 온다고 가정
      if (data && data.id && data.message) {
        addLog(data.id, data.message, data.type);
      }
    });
    // ----------------
    initialized.value = true;

    return fetchProcesses();
  }

  const isRunning = (processId) => processIds.value.includes(processId);

  return {
    processIds,
    taskLogs, // 로그 노출
    initialize,
    isRunning,
    addLog // 외부에서 수동으로 로그를 남길 경우를 대비해 노출
  };
});