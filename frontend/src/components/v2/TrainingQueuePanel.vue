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
                @click="refresh"
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

            <!-- Running (always at the bottom, prominent). GPU 여유 시 동시
                 학습이 가능하므로 여러 개가 동시에 보일 수 있다. -->
            <q-btn
                v-for="cp in queue.running_list"
                :key="`r-${cp.id}`"
                push
                round
                size="xl"
                color="deep-orange"
                icon="model_training"
                @click="$emit('watch', cp)"
            >
                <q-tooltip>{{ cp.name }} — running</q-tooltip>
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
const queue = ref({ running: null, running_list: [], queued: [], recent: [] });
const refreshing = ref(false);
// 진행 중인 refresh 개수 — 수동 클릭/socketio 이벤트/마운트/부모 호출이 겹쳐도
// 마지막 하나가 끝날 때까지 loading 을 유지하기 위한 카운터.
let inflight = 0;
let retryTimer = null;
let pollTimer = null;

// 모든 큐 새로고침의 단일 경로. 어떤 트리거(버튼/socketio/부모)든 이 함수를 거치므로
// refreshing(loading)이 모든 로딩을 포괄한다.
async function refresh() {
    inflight += 1;
    refreshing.value = true;
    try {
        const res = await api.get('/train/queue');
        const d = res.data;
        // 워크스페이스 진입 시 동시 요청 burst 로 dev 서버(Flask-SocketIO)가 응답을
        // 깨뜨리면 d 가 객체가 아니거나(파싱 실패 → 문자열) 예상 키가 없다. 이때
        // 빈 상태로 덮어써 주황 running 버튼이 사라지면 안 되므로, 기존 표시를
        // 유지하고 잠시 후 재시도한다.
        const malformed = !d || typeof d !== 'object'
            || (d.running_list === undefined && d.running === undefined && d.queued === undefined);
        if (malformed) {
            if (retryTimer) clearTimeout(retryTimer);
            retryTimer = setTimeout(refresh, 800);
            return;
        }
        queue.value = {
            running: d.running || null,
            running_list: d.running_list || (d.running ? [d.running] : []),
            queued: d.queued || [],
            recent: d.recent || [],
        };
    } catch {
        // 네트워크 일시 단절/오류 — 잠시 후 재시도
        if (retryTimer) clearTimeout(retryTimer);
        retryTimer = setTimeout(refresh, 800);
    } finally {
        inflight -= 1;
        if (inflight <= 0) {
            inflight = 0;
            refreshing.value = false;
        }
    }
}

const onQueueChanged = () => refresh();

onMounted(() => {
    refresh();
    socket.on('train_queue_changed', onQueueChanged);
    // socketio 이벤트 누락 / 응답 깨짐에 대비한 가벼운 안전망 주기 폴링.
    pollTimer = setInterval(refresh, 7000);
});

onUnmounted(() => {
    socket.off('train_queue_changed', onQueueChanged);
    if (pollTimer) clearInterval(pollTimer);
    if (retryTimer) clearTimeout(retryTimer);
});

defineExpose({ refresh, queue });
</script>
