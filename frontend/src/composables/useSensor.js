import { watch } from 'vue';
import { api } from 'boot/axios';
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

  sensor.status = sensor.status || 'off';

  watch(
    () => topicStore.isPublished(sensor.read_topic),
    (isPublished, wasPublished) => {
      if (isPublished && !wasPublished) {
        sensor.status = 'on';
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
    sensor.status = 'loading';
    return api.post('/sensor:start', sensor).catch((error) => {
      console.error('Error starting sensor:', error);
      sensor.status = 'off';
    });
  }

  function stopSensor() {
    sensor.status = 'loading';
    return api.post('/sensor:stop', sensor).then(() => {
      sensor.status = 'off';
    }).catch((error) => {
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
