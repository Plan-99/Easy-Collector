
import { ref, onUnmounted } from 'vue';
import { api } from 'boot/axios';
import { useQuasar } from 'quasar';

export function useLeaderTeleoperation() {
  const $q = useQuasar();
  const leaderTeleStarted = ref(false);

  function startLeaderTele(robot, preset, logEmitId = null) {
    // if (logEmitId) {
    //   payload.log_emit_id = logEmitId;
    // }
    console.log('Starting leader teleoperation for robot:', robot, 'with preset:', preset, 'and logEmitId:', logEmitId);
    api.post('/leader_robot:tele_start', {
      robot: robot,
      preset: preset,
      log_emit_id: logEmitId
    })
      .then(() => {
        leaderTeleStarted.value = true;
        $q.notify({
          color: 'positive',
          position: 'top',
          message: 'Leader teleoperation started successfully',
          icon: 'check_circle'
        });
    })
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
