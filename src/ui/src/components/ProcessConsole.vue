<template>
    <q-scroll-area class="bg-dark q-pa-sm" style="height: 330px;">
        <div v-for="(log, index) in logs" :key="index" :class="log.type === 'error' ? 'text-red-5' : 'text-white'">
            {{ log.message }}
        </div>
    </q-scroll-area>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue';
import { defineProps } from 'vue';
import { useSocket } from '../composables/useSocket';

const { socket } = useSocket();

onMounted(() => {
    socket.on(`log_${props.process}`, (data) => {
        logs.value.push({ type: data.type, message: data.log });
    });
});
onUnmounted(() => {
    socket.off(`log_${props.process}`);
});


const logs = ref([]);


const props = defineProps({
    process: {
        type: String,
        default: ''
    }
})
</script>