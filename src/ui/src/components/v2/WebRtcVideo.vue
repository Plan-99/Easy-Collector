<template>
  <div class="relative-position" style="overflow: hidden;">
    <video ref="videoElement" autoplay playsinline muted style="height: 100%; width: 100%;"></video>
    <img
      v-if="gradcamOn && gradcamSrc"
      :src="gradcamSrc"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; object-fit: contain; opacity: 0.6; pointer-events: none;"
    />
    <q-btn
      v-if="sensorId"
      round
      dense
      size="sm"
      :icon="gradcamOn ? 'visibility' : 'visibility_off'"
      :color="gradcamOn ? 'orange' : 'grey-7'"
      style="position: absolute; bottom: 8px; right: 8px; z-index: 1;"
      @click.stop="gradcamOn = !gradcamOn"
    >
      <q-tooltip>Grad-CAM</q-tooltip>
    </q-btn>
    <q-inner-loading :showing="props.loading">
    </q-inner-loading>
  </div>
</template>

<script setup>
import { ref, watch, onMounted, onUnmounted } from 'vue';
import { useWebRTC } from 'src/composables/useWebRTC';
import { useSocket } from 'src/composables/useSocket.js';

const props = defineProps({
  topic: {
    type: String,
    required: true,
  },
  resize: {
    type: Array,
    default: () => []
  },
  loading: {
    type: Boolean,
    default: false,
  },
  cropped_area: {
    type: Object,
    default: () => ({}),
  },
  rotate: {
    type: Number,
    default: 0,
  },
  msgType: {
    type: String,
    default: 'sensor_msgs/CompressedImage',
  },
  sensorId: {
    type: [String, Number],
    default: null,
  },
});

const videoElement = ref(null);
const { connect, disconnect, addConfig } = useWebRTC();
const { socket } = useSocket();

// Grad-CAM overlay
const gradcamOn = ref(false);
const gradcamSrc = ref(null);

function onGradcam(data) {
  const key = `sensor_${props.sensorId}`;
  if (data[key]) {
    gradcamSrc.value = 'data:image/jpeg;base64,' + data[key];
  }
}
let stream_id = '';

const setupWebRTC = () => {
  connect(props.topic, {
    resize: props.resize.length ? props.resize : null,
    cropped_area: props.cropped_area.length ? props.cropped_area : null,
    rotate: props.rotate,
    msg_type: props.msgType,
  }, (event) => {
    if (videoElement.value) {
      const newStream = new MediaStream();
      newStream.addTrack(event.track);
      videoElement.value.srcObject = newStream;
    }
  }).then((res) => {
    stream_id = res.stream_id;
    console.log(stream_id)
  })
};


watch(() => props.loading, (newVal) => {
  if (!newVal) {
    console.log(newVal, 'Loading finished, setting up WebRTC');
    setupWebRTC();
  }
}, { immediate: true });


watch(() => props.resize, (newResize, oldResize) => {
  if (newResize[0] === oldResize[0] && newResize[1] === oldResize[1]) {
    return;
  }
  addConfig(stream_id, {
    resize: newResize.length ? newResize : null,
    cropped_area: props.cropped_area.length ? props.cropped_area : null,
    rotate: props.rotate,
  });
});

watch(() => props.cropped_area, (newCroppedArea, oldCroppedArea) => {
  if (JSON.stringify(newCroppedArea) === JSON.stringify(oldCroppedArea)) {
    return;
  }
  addConfig(stream_id, {
    resize: props.resize.length ? props.resize : null,
    cropped_area: newCroppedArea,
    rotate: props.rotate,
  });
});

watch (() => props.rotate, (newRotate, oldRotate) => {
  if (newRotate === oldRotate) {
    return;
  }
  addConfig(stream_id, {
    resize: props.resize.length ? props.resize : null,
    cropped_area: props.cropped_area.length ? props.cropped_area : null,
    rotate: newRotate,
  })
});

onMounted(() => {
  if (props.loading) {
    console.log('Component mounted, waiting for loading to finish');
  }
  if (props.sensorId) {
    socket.on('gradcam', onGradcam);
    socket.on('stop_process', (data) => {
      if (data.id === 'checkpoint_test') {
        gradcamSrc.value = null;
      }
    });
  }
});

onUnmounted(() => {
  disconnect();
  if (props.sensorId) {
    socket.off('gradcam', onGradcam);
  }
});

</script>
