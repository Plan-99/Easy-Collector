import { api } from 'boot/axios';
import { useProcessStore } from '../stores/processStore';

export function useSensor(sensor) {
  const processStore = useProcessStore();

  function status() {
    if (sensor.loading) return 'loading';
    return processStore.processIds.find((id) => `sensor_${sensor.id}` === id) ? 'on' : 'off';
  }

  function startSensor(sensor) {
    sensor.status = 'loading';
    return api.post('/sensor:start', sensor).catch((error) => {
      console.error('Error starting sensor:', error);
      sensor.status = 'off';
    });
  }

  function stopSensor(sensor) {
    sensor.status = 'loading';
    return api.post('/sensor:stop', sensor).catch((error) => {
      console.error('Error stopping sensor:', error);
      sensor.status = 'on';
    });
  }

  // function subsSensor(sensor) {
  //   if (!processStore.isRunning(sensor.process_id)) {
  //     return;
  //   }
  //   watchingSensor.value = sensor;
  //   connect(sensor, (event) => {
  //     const newStream = new MediaStream();
  //     newStream.addTrack(event.track);
  //     if (sensorVideo.value) {
  //       sensorVideo.value.srcObject = newStream;
  //     }
  //   });
  // }
  return {
    startSensor,
    stopSensor,
    status,
  };
}
