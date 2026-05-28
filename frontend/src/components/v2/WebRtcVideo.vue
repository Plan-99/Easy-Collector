<template>
  <div class="relative-position" style="overflow: hidden;">
    <video
      ref="videoElement"
      autoplay
      playsinline
      muted
      style="height: 100%; width: 100%;"
      @playing="streamLoading = false"
      @loadeddata="streamLoading = false"
    ></video>
    <img
      v-if="gradcamOn && gradcamSrc"
      :src="gradcamSrc"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; object-fit: contain; opacity: 0.6; pointer-events: none;"
    />
    <img
      v-if="overlaySrc"
      :src="overlaySrc"
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
      <q-tooltip>{{ $t('gradcamTooltip') }}</q-tooltip>
    </q-btn>
    <q-inner-loading :showing="props.loading || streamLoading">
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
  overlaySrc: {
    type: String,
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
// 컴포넌트 마운트 동안 단 하나의 active stream 만 유지. setupWebRTC 가
// 여러 번 호출돼도 마지막 것만 살아남도록 generation 토큰으로 race 방어.
let _setupGeneration = 0;

// /offer 가 보내진 시점부터 첫 프레임이 디코드 (video element 의 loadeddata /
// playing 이벤트) 될 때까지 true. parent 가 주는 props.loading 과 OR 로 묶여
// q-inner-loading 에 표시된다.
const streamLoading = ref(false);

const setupWebRTC = () => {
  if (!props.topic) return;
  const myGen = ++_setupGeneration;
  streamLoading.value = true;
  connect(props.topic, {
    resize: props.resize.length ? props.resize : null,
    cropped_area: props.cropped_area.length ? props.cropped_area : null,
    rotate: props.rotate,
    msg_type: props.msgType,
  }, (event) => {
    // race: 새 setup 이 시작된 뒤 늦게 도착한 track 무시.
    if (myGen !== _setupGeneration) return;
    if (videoElement.value) {
      const newStream = new MediaStream();
      newStream.addTrack(event.track);
      videoElement.value.srcObject = newStream;
    }
  }).then((res) => {
    if (!res) {
      if (myGen === _setupGeneration) streamLoading.value = false;
      return;
    }
    if (myGen !== _setupGeneration) return;
    stream_id = res.stream_id;
  }).catch(() => {
    if (myGen === _setupGeneration) streamLoading.value = false;
  });
};

// loading watcher — 원래 동작 그대로. 마운트 시 loading=false 면 즉시 setup,
// 로딩 중이었다 false 로 바뀌면 그때 setup.
watch(() => props.loading, (newVal) => {
  if (!newVal) setupWebRTC();
}, { immediate: true });

// topic watcher — 같은 컴포넌트 인스턴스에서 topic 이 바뀌면 (예: WorkspacePage
// 에서 :key 를 제거해 컴포넌트를 reuse 할 때) 새 stream 으로 연결. connect()
// 내부에서 이전 PC 를 disconnect() 하므로 1 instance = 1 active PC.
watch(() => props.topic, (newTopic, oldTopic) => {
  if (!newTopic || newTopic === oldTopic) return;
  if (props.loading) return;
  setupWebRTC();
});


watch(() => props.resize, (newResize, oldResize) => {
  if (newResize[0] === oldResize[0] && newResize[1] === oldResize[1]) {
    return;
  }
  // [w, undefined] 같이 한 축이 비어있는 transient는 backend cv2.resize를 깨뜨리므로 무시
  if (!Array.isArray(newResize) || newResize.length !== 2 ||
      typeof newResize[0] !== 'number' || typeof newResize[1] !== 'number') {
    return;
  }
  addConfig(stream_id, {
    resize: newResize,
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
