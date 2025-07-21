<template>
  <div class="relative-position">
    <video ref="videoElement" autoplay playsinline muted style="height: 100%; width: 100%;"></video>
    <q-inner-loading :showing="!isPublished">
      <q-spinner-gears size="50px" color="primary" />
      <div class="q-mt-md">Waiting for topic to be published...</div>
    </q-inner-loading>
  </div>
</template>

<script setup>
import { onMounted, onUnmounted, ref, watch } from 'vue';
import { useWebRTC } from '../composables/useWebRTC';
import { useSocket } from '../composables/useSocket';
import { api } from 'src/boot/axios';

const props = defineProps({
  processId: {
    type: String,
    required: true,
  },
  topic: {
    type: String,
    required: true,
  },
  resize: {
    type: Array,
    default: () => []
  },
});

const videoElement = ref(null);
const { socket } = useSocket();
const { connect, addConfig } = useWebRTC(socket);
let stream_id = '';

const setupWebRTC = () => {
  connect(props.topic, {
    resize: props.resize.length ? props.resize : null,
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

const isPublished = ref(false);
let interval = null;

onMounted(() => {
  interval = setInterval(() => {
    api.get(`/topics`)
      .then((res) => {
        isPublished.value = Boolean(res.data.topics.find(topic => topic.name === props.topic));
        if (isPublished.value) {
          setupWebRTC();
          clearInterval(interval);
        }
      })
      .catch(error => {
        console.error('Error setting up WebRTC:', error);
      });
  }, 1000); // Delay to ensure the video element is ready
});

onUnmounted(() => {
  clearInterval(interval);
});

watch(() => props.resize, (newResize) => {
  addConfig(stream_id, {
    resize: newResize.length ? newResize : null,
  });
}, { deep: true });


</script>
