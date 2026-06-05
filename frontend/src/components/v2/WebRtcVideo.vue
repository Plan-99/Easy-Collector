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
      v-if="overlaySrc"
      :src="overlaySrc"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; object-fit: contain; opacity: 0.6; pointer-events: none;"
    />
    <q-inner-loading :showing="props.loading || streamLoading">
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
let stream_id = '';
// 컴포넌트 마운트 동안 단 하나의 active stream 만 유지. setupWebRTC 가
// 여러 번 호출돼도 마지막 것만 살아남도록 generation 토큰으로 race 방어.
let _setupGeneration = 0;

// /offer 가 보내진 시점부터 첫 프레임이 디코드 (video element 의 loadeddata /
// playing 이벤트) 될 때까지 true. parent 가 주는 props.loading 과 OR 로 묶여
// q-inner-loading 에 표시된다.
const streamLoading = ref(false);
// onTrack 이 videoElement 마운트 전에 호출될 수 있다(풀에 track 이 이미 캐시된
// 2번째 view 는 _acquire 가 동기로 onTrack 을 부르는데, 이게 immediate loading
// watch 를 통해 setup() 단계 — onMounted 이전 — 에 실행되어 videoElement 가 아직
// null). 받은 stream 을 보관했다가 마운트 시 꽂아 무한 로딩을 막는다.
const pendingStream = ref(null);

const setupWebRTC = () => {
  if (!props.topic) return;
  const myGen = ++_setupGeneration;
  streamLoading.value = true;
  connect(props.topic, {
    resize: props.resize?.length ? props.resize : null,
    cropped_area: props.cropped_area?.length ? props.cropped_area : null,
    rotate: props.rotate,
    msg_type: props.msgType,
  }, (event) => {
    // race: 새 setup 이 시작된 뒤 늦게 도착한 track 무시.
    if (myGen !== _setupGeneration) return;
    const newStream = new MediaStream();
    newStream.addTrack(event.track);
    pendingStream.value = newStream;
    if (videoElement.value) {
      videoElement.value.srcObject = newStream;
    }
    // videoElement 가 아직 없으면 onMounted 에서 pendingStream 을 꽂는다.
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
  // [w, undefined] 같이 한 축이 비어있는 transient나 null/undefined 는 backend
  // cv2.resize 를 깨뜨리므로 무시. (타입 가드를 먼저 — null[0] 접근 방지)
  if (!Array.isArray(newResize) || newResize.length !== 2 ||
      typeof newResize[0] !== 'number' || typeof newResize[1] !== 'number') {
    return;
  }
  if (Array.isArray(oldResize) && newResize[0] === oldResize[0] && newResize[1] === oldResize[1]) {
    return;
  }
  addConfig(stream_id, {
    resize: newResize,
    cropped_area: props.cropped_area?.length ? props.cropped_area : null,
    rotate: props.rotate,
  });
});

watch(() => props.cropped_area, (newCroppedArea, oldCroppedArea) => {
  if (JSON.stringify(newCroppedArea) === JSON.stringify(oldCroppedArea)) {
    return;
  }
  addConfig(stream_id, {
    resize: props.resize?.length ? props.resize : null,
    cropped_area: newCroppedArea,
    rotate: props.rotate,
  });
});

watch (() => props.rotate, (newRotate, oldRotate) => {
  if (newRotate === oldRotate) {
    return;
  }
  addConfig(stream_id, {
    resize: props.resize?.length ? props.resize : null,
    cropped_area: props.cropped_area?.length ? props.cropped_area : null,
    rotate: newRotate,
  })
});

onMounted(() => {
  // setup() 단계에서 onTrack 이 먼저 와 보관해 둔 stream 이 있으면 지금 꽂는다.
  if (pendingStream.value && videoElement.value && !videoElement.value.srcObject) {
    videoElement.value.srcObject = pendingStream.value;
  }
});

onUnmounted(() => {
  disconnect();
});

</script>
