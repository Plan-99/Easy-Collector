<template>
    <q-page-sticky position="bottom-right" :offset="[18, 18]">
        <div class="column items-end q-gutter-y-sm">
            <!-- 수동 새로고침 — 폴링(10초)/socketio 이벤트를 기다리지 않고
                 사용자가 즉시 큐 상태를 다시 가져온다. -->
            <q-btn
                round
                size="md"
                color="blue-grey-7"
                icon="refresh"
                :loading="refreshing"
                @click="manualRefresh"
            >
                <q-tooltip>Refresh queue</q-tooltip>
            </q-btn>

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
const refreshing = ref(false);

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

// 사용자가 직접 누르는 새로고침. refresh() 는 socketio 이벤트와 공유하므로
// loading 스피너는 manual 전용 래퍼에서만 띄운다.
async function manualRefresh() {
    if (refreshing.value) return;
    refreshing.value = true;
    try {
        await refresh();
    } finally {
        refreshing.value = false;
    }
}

const onQueueChanged = () => refresh();

onMounted(() => {
    // 마운트 시 1회 fetch. 이후 갱신은 socketio 'train_queue_changed' 이벤트와
    // 수동 새로고침 버튼으로만 — 주기 폴링 제거.
    refresh();
    socket.on('train_queue_changed', onQueueChanged);
});

onUnmounted(() => {
    socket.off('train_queue_changed', onQueueChanged);
});

defineExpose({ refresh, queue });
</script>
