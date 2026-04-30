<template>
    <q-page-sticky position="bottom-right" :offset="[18, 18]">
        <div class="column items-end q-gutter-y-sm">
            <!-- Queued (FIFO order, oldest first) -->
            <q-btn
                v-for="(cp, idx) in queue.queued"
                :key="`q-${cp.id}`"
                round
                size="lg"
                color="grey-6"
                icon="schedule"
                @click="$emit('watch', cp)"
            >
                <q-badge color="orange" floating>{{ idx + 1 }}</q-badge>
                <q-tooltip>{{ cp.name }} — queued (#{{ idx + 1 }})</q-tooltip>
            </q-btn>

            <!-- Running (always at the bottom, prominent) -->
            <q-btn
                v-if="queue.running"
                push
                round
                size="xl"
                color="deep-orange"
                icon="model_training"
                @click="$emit('watch', queue.running)"
            >
                <q-tooltip>{{ queue.running.name }} — running</q-tooltip>
            </q-btn>
        </div>
    </q-page-sticky>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import { useSocket } from 'src/composables/useSocket';

const { socket } = useSocket();

defineEmits(['watch']);

// 큐 스냅샷 — backend에서 fetch. 'train_queue_changed' socketio 이벤트로 갱신.
// floating 버튼은 active(queued/running)만 보여주지만, recent도 함께 보관해서
// TrainingDialog가 종료 상태(finished/failed)로 자연스럽게 전환될 수 있도록 한다.
const queue = ref({ running: null, queued: [], recent: [] });
let pollTimer = null;

async function refresh() {
    try {
        const res = await api.get('/train/queue');
        queue.value = {
            running: res.data.running || null,
            queued: res.data.queued || [],
            recent: res.data.recent || [],
        };
    } catch {
        // 네트워크 일시 단절 — 다음 tick에 재시도
    }
}

const onQueueChanged = () => refresh();

onMounted(() => {
    refresh();
    socket.on('train_queue_changed', onQueueChanged);
    // 백엔드 단절 / 이벤트 누락 대비 안전망 폴링 (10초).
    pollTimer = setInterval(refresh, 10000);
});

onUnmounted(() => {
    socket.off('train_queue_changed', onQueueChanged);
    if (pollTimer) clearInterval(pollTimer);
});

defineExpose({ refresh, queue });
</script>
