import { watch } from 'vue';
import { api } from 'boot/axios';
import { Notify } from 'quasar';
import { t } from 'boot/i18n';
import { useTopicStore } from 'src/stores/topicStore';

/**
 * Sensor composable — push 모델 기반.
 *
 * `topicStore.isPublished(sensor.read_topic)`을 watch해서 sensor.status를
 * 자동으로 'on'/'off'로 동기화. start/stop 버튼은 driver spawn/kill만 담당.
 *
 * 호환성: `checkSensorTopic`은 no-op stub로 유지.
 */
export function useSensor(sensor, sensorOnCallback = () => {}) {
  const topicStore = useTopicStore();

  const formatError = (error) => {
    if (error?.response?.data?.message) return error.response.data.message;
    if (error?.message) return error.message;
    try {
      return JSON.stringify(error);
    } catch {
      return String(error);
    }
  };

  sensor.status = sensor.status || 'off';
  if (!sensor.lastError) sensor.lastError = null;

  let startWatchdog = null;
  const START_TIMEOUT_MS = 20000;

  function clearStartWatchdog() {
    if (startWatchdog) {
      clearTimeout(startWatchdog);
      startWatchdog = null;
    }
  }

  watch(
    () => topicStore.isPublished(sensor.read_topic),
    (isPublished, wasPublished) => {
      if (isPublished && !wasPublished) {
        clearStartWatchdog();
        sensor.status = 'on';
        sensor.lastError = null;
        try {
          sensorOnCallback();
        } catch (e) {
          console.error('sensorOnCallback error:', e);
        }
      } else if (!isPublished && wasPublished) {
        sensor.status = 'off';
      }
    },
    { immediate: true },
  );

  function status() {
    return sensor.status;
  }

  function startSensor() {
    clearStartWatchdog();
    sensor.status = 'loading';
    sensor.lastError = null;
    startWatchdog = setTimeout(() => {
      if (sensor.status === 'loading') {
        const msg = t('errorStartSensorTimeout', { name: sensor.name });
        sensor.lastError = msg;
        sensor.status = 'error';
        Notify.create({ color: 'negative', message: msg });
      }
      startWatchdog = null;
    }, START_TIMEOUT_MS);
    return api.post('/sensor:start', sensor).catch((error) => {
      clearStartWatchdog();
      const msg = formatError(error);
      console.error('Error starting sensor:', msg);
      sensor.lastError = msg;
      sensor.status = 'error';
      Notify.create({
        color: 'negative',
        message: t('errorStartSensorFailed', { name: sensor.name, error: msg }),
      });
    });
  }

  function stopSensor() {
    clearStartWatchdog();
    sensor.status = 'loading';
    return api
      .post('/sensor:stop', sensor)
      .then(() => {
        sensor.status = 'off';
        sensor.lastError = null;
      })
      .catch((error) => {
        console.error('Error stopping sensor:', error);
        sensor.status = 'on';
      });
  }

  function checkSensorTopic() {
    /* no-op: topicStore가 push 받아 자동으로 상태 동기화 */
  }

  return {
    startSensor,
    stopSensor,
    status,
    checkSensorTopic,
  };
}
