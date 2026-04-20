<template>
    <q-btn
        flat
        dense
        :color="hasLogs ? 'green-4' : 'grey-6'"
        icon="terminal"
        size="sm"
        @click="showDialog = true"
    >
        <q-tooltip>ROS2 Bridge Logs</q-tooltip>
        <q-badge v-if="newLogCount > 0" color="red" floating>{{ newLogCount > 9 ? '9+' : newLogCount }}</q-badge>
    </q-btn>
    <q-dialog v-model="showDialog" position="bottom" full-width @show="onShow">
        <q-card class="bg-secondary" style="height: 350px;">
            <q-card-section class="q-py-sm row items-center">
                <div class="text-subtitle2 text-white">ROS2 Bridge Logs</div>
                <q-space />
                <q-btn flat dense icon="close" color="white" @click="showDialog = false" />
            </q-card-section>
            <q-separator color="grey-8" />
            <q-card-section class="q-pa-none" style="height: calc(100% - 50px);">
                <process-console process="ros2_bridge" style="height: 100%;" />
            </q-card-section>
        </q-card>
    </q-dialog>
</template>

<script setup>
import { ref, computed, watch } from 'vue'
import { useProcessStore } from '../../stores/processStore'
import ProcessConsole from './ProcessConsole.vue'

const processStore = useProcessStore()
const showDialog = ref(false)
const lastSeenCount = ref(0)

const logs = computed(() => processStore.taskLogs['ros2_bridge'] || [])
const hasLogs = computed(() => logs.value.length > 0)
const newLogCount = computed(() => {
    if (showDialog.value) return 0
    return Math.max(0, logs.value.length - lastSeenCount.value)
})

function onShow() {
    lastSeenCount.value = logs.value.length
}

watch(showDialog, (visible) => {
    if (!visible) {
        lastSeenCount.value = logs.value.length
    }
})
</script>
