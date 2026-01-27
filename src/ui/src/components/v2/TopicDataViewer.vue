<template>
    <div class="topic-viewer-container bg-black text-green-5 q-pa-sm border-rounded relative-position column">
        
        <div class="row items-center q-mb-sm q-pb-xs" style="border-bottom: 1px solid #333;">
            <q-icon name="dataset" size="xs" class="q-mr-sm" />
            <div class="text-caption text-weight-bold ellipsis col">
                {{ topic }}
                <q-tooltip>{{ topic }}</q-tooltip>
            </div>
            <div class="text-caption text-grey-6" style="font-size: 10px;">
                {{ type }}
            </div>
        </div>

        <div class="col relative-position scroll code-area">
            <pre v-if="receivedData" class="json-text">{{ formattedData }}</pre>
            
            <div v-else class="absolute-center text-center text-grey-8">
                <q-spinner-dots size="30px" />
                <div class="text-caption q-mt-xs">Waiting for data...</div>
            </div>
        </div>

        <div class="status-dot" :class="{ 'blink': isReceiving }"></div>
    </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue';
import { socket } from 'src/boot/socket'; 

const props = defineProps({
    topic: { type: String, required: true },
    type: { type: String, default: 'Unknown Type' }
});

const receivedData = ref(null);
const isReceiving = ref(false);
let blinkTimeout = null;

// JSON 데이터 포맷팅
const formattedData = computed(() => {
    if (!receivedData.value) return '';
    try {
        return JSON.stringify(receivedData.value, null, 2);
    } catch (e) {
        return receivedData.value;
    }
});

// 데이터 수신 핸들러
function handleDataUpdate(data) {
    if (data.topic === props.topic) {
        receivedData.value = data.msg;
        isReceiving.value = true;
        if (blinkTimeout) clearTimeout(blinkTimeout);
        blinkTimeout = setTimeout(() => { isReceiving.value = false; }, 200);
    }
}

// ★ 핵심 수정: 구독 요청 함수 분리
function requestSubscription() {
    console.log(`[Viewer] Requesting subscription for: ${props.topic}`);
    socket.emit('subscribe_topic', { 
        topic: props.topic, 
        type: props.type 
    });
}

onMounted(() => {
    // 1. 데이터 받는 통로 열기
    socket.on('ros_data', handleDataUpdate);

    // 2. [중요] 이미 연결되어 있다면 바로 구독 요청
    if (socket.connected) {
        requestSubscription();
    }

    // 3. [핵심] 연결이 끊겼다가 다시 붙었을 때(재연결) 자동으로 구독 요청
    socket.on('connect', requestSubscription);
});

onUnmounted(() => {
    // 컴포넌트가 사라질 때만 구독 취소
    socket.emit('unsubscribe_topic', { topic: props.topic });
    
    // 리스너 정리 (메모리 누수 방지)
    socket.off('ros_data', handleDataUpdate);
    socket.off('connect', requestSubscription); // 재연결 리스너도 해제
});
</script>

<style scoped>
.topic-viewer-container {
    height: 100%;
    border: 1px solid #333;
    font-family: 'Consolas', 'Monaco', monospace;
}

.code-area {
    font-size: 11px;
    line-height: 1.4;
}

.json-text {
    margin: 0;
    white-space: pre-wrap;
    word-break: break-all;
}

.status-dot {
    position: absolute;
    top: 8px;
    right: 8px;
    width: 8px;
    height: 8px;
    background-color: #00ff00;
    border-radius: 50%;
    opacity: 0.2;
    transition: opacity 0.1s;
}
.status-dot.blink {
    opacity: 1;
    box-shadow: 0 0 5px #00ff00;
}
</style>