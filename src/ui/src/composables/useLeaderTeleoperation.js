
import { ref, onUnmounted } from 'vue';
import { api } from 'boot/axios';
import { useQuasar } from 'quasar';

export function useLeaderTeleoperation() {
  const $q = useQuasar();
  const leaderTeleStarted = ref(false);

  function startLeaderTele(robotId, logEmitId = null) {
    const payload = { robot_id: robotId };
    if (logEmitId) {
      payload.log_emit_id = logEmitId;
    }
    api.post('/leader_robot:tele_start', payload)
      .catch((error) => {
        console.error('Error starting leader teleoperation:', error);
        $q.notify({
          color: 'negative',
          position: 'top',
          message: 'Failed to start leader teleoperation',
          icon: 'report_problem'
        });
      });
  }

  function stopLeaderTele(robotId, logEmitId = null) {
    const payload = { robot_id: robotId };
    if (logEmitId) {
      payload.log_emit_id = logEmitId;
    }
    api.post('/leader_robot:tele_stop', payload)
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
