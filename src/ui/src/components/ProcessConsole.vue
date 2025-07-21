<template>
    <q-scroll-area class="bg-dark q-pa-sm" style="height: 330px;">
        <div v-for="(log, index) in logs" :key="index" :class="log.type === 'error' ? 'text-red-5' : 'text-white'">
            {{ log.message }}
        </div>
    </q-scroll-area>
</template>

<script setup>
import { ref, onUnmounted, watch } from 'vue';
import { defineProps } from 'vue';
import { useSocket } from '../composables/useSocket';

const { socket } = useSocket();
const logs = ref([]);

const props = defineProps({
    process: {
        type: String,
        default: ''
    }
});

const logHandler = (data) => {
    logs.value.push({ type: data.type, message: data.log });
};

watch(() => props.process, (newProcess, oldProcess) => {
    logs.value = []; // 새로운 프로세스를 위해 로그를 비웁니다.
    if (oldProcess) {
        socket.off(`log_${oldProcess}`, logHandler);
    }
    if (newProcess) {
        socket.on(`log_${newProcess}`, logHandler);
    }
}, { immediate: true });

onUnmounted(() => {
    if (props.process) {
        socket.off(`log_${props.process}`, logHandler);
    }
});

</script>