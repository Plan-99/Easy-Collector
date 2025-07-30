import { api } from 'boot/axios';

export function useSensor(sensor, sensorOnCallback=() => {}) {

  let sensorTopicChecker = null;

  checkSensorTopic(1);

  function status() {
    return sensor.status
  }

  function startSensor() {
    sensor.status = 'loading';
    return api.post('/sensor:start', sensor).then(() => {
      checkSensorTopic();
    }).catch((error) => {
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

  function checkSensorTopic(maxSteps = 10) {
    if (sensorTopicChecker) {
      clearInterval(sensorTopicChecker);
    }
    let steps = 0;
    sensorTopicChecker = setInterval(() => {
      api.get(`/topics`)
      .then((res) => {
        const isPublished = Boolean(res.data.topics.find(topic => topic.name === sensor.topic));
        if (isPublished) {
          sensor.status = 'on';
          sensorOnCallback();
          clearInterval(sensorTopicChecker);
        }
      })
      .catch(error => {
        console.error('Error setting up WebRTC:', error);
      });
      steps++;
      if (steps >= maxSteps) {
        clearInterval(sensorTopicChecker);
        sensor.status = 'off';
        if (sensor.status === 'on') {
          stopSensor();
        }
      }
    }, 1000);
  }

  return {
    startSensor,
    stopSensor,
    status,
  };
}
