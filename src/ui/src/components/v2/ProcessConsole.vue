<template>
    <q-scroll-area class="bg-dark q-pa-sm border-rounded" style="height: 100%;">
        <div v-for="(log, index) in logs" :key="index" :class="log.type === 'error' ? 'text-red-5' : 'text-white'">
            {{ log.message }}
        </div>
    </q-scroll-area>
</template>

<script setup>
import { computed } from 'vue';
import { defineProps } from 'vue';
import { useProcessStore } from '../../stores/processStore';

const processStore = useProcessStore();

const props = defineProps({
    process: {
        type: String,
        default: ''
    }
});

// const logHandler = (data) => {
//     const type = data.log.startsWith('[ERROR]') ? 'error' : 'stdout';
//     logs.value.push({ type: type, message: data.log });
// };

// 특정 프로세스의 로그가 변경될 때마다 실행
const logs = computed(() => processStore.taskLogs[props.process] || []);

</script>