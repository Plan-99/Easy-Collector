<template>
  <div class="relative-position">
    <video ref="videoElement" autoplay playsinline muted style="height: 100%; width: 100%;"></video>
    <q-inner-loading :showing="props.loading">
    </q-inner-loading>
  </div>
</template>

<script setup>
import { ref, watch, onMounted, onUnmounted } from 'vue';
import { useWebRTC } from 'src/composables/useWebRTC';

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
  }
});

const videoElement = ref(null);
const { connect, disconnect, addConfig } = useWebRTC();
let stream_id = '';

const setupWebRTC = () => {
  connect(props.topic, {
    resize: props.resize.length ? props.resize : null,
    cropped_area: props.cropped_area.length ? props.cropped_area : null,
    rotate: props.rotate,
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
});

onUnmounted(() => {
  disconnect();
});

</script>
