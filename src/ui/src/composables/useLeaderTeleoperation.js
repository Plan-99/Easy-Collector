
import { onUnmounted, computed } from 'vue';
import { api } from 'boot/axios';
import { useQuasar } from 'quasar';
import { useProcessStore } from 'src/stores/processStore';

const processStore = useProcessStore();

export function useLeaderTeleoperation() {
  const $q = useQuasar();
  const leaderTeleStarted = computed(() => processStore.isRunning('leader_teleoperation'));

  function startLeaderTele(assembly_id, logEmitId = null) {
    api.post('/teleoperator:leader_start', {
      assembly_id: assembly_id,
      log_emit_id: logEmitId
    })
  }

  function stopLeaderTele() {
    api.post('/leader_robot:tele_stop')
      .catch((error) => {
        console.error('Error stopping leader teleoperation:', error);
        $q.notify({
          color: 'negative',
          position: 'top',
          message: 'Failed to stop leader teleoperation',
          icon: 'report_problem'
        });
      });
  }

  onUnmounted(() => {
    if (leaderTeleStarted.value) {
      stopLeaderTele();
    }
  });

  return {
    leaderTeleStarted,
    startLeaderTele,
    stopLeaderTele
  };
}
