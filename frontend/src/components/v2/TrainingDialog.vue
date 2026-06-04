<template>
  <q-dialog full-width>
    <q-card class="bg-secondary border-rounded border-white" dark>
      <q-card-section class="row bg-dark text-white">
        <div class="text-h6 ellipsis">{{ checkpoint.name }}</div>
        <q-space></q-space>
        <q-btn dense color="white" round icon="close" text-color="dark" @click="$emit('update:modelValue', false)"/>
      </q-card-section>
      <q-separator color="white"></q-separator>
      <q-card-section>
        <checkpoint-info :checkpoint="props.checkpoint" />
      </q-card-section>
      <q-card-section>
        <div class="text-center">
            <q-btn color="negative" :label="$t('trainStopBtn')" @click="stopTraining" v-if="isTraining" />
            <q-btn :color="terminalColor" :label="terminalLabel" @click="$emit('update:modelValue', false)" v-else-if="isTerminal" />
            <q-btn color="primary" :label="$t('trainCancelBtn')" @click="cancelTraing" v-else />
        </div>
      </q-card-section>

      <q-card-section class="q-pt-none" v-if="props.isTraining || isFailed">
        <div v-if="isFailed" class="text-negative q-mb-sm">{{ $t('trainFailed') }}</div>
        <process-console v-if="isFailed" process="train_task" style="height: 240px;" />
      </q-card-section>

      <q-card-section class="q-pt-none" v-if="props.isTraining">
        <div class="q-gutter-y-md">
          <q-linear-progress
            instant-feedback
            :value="trainingProgress"
            size="20px"
            color="primary"
            track-color="black"
          >
            <div class="absolute-full flex flex-center">
              <q-badge color="white" text-color="dark" :label="`${Number(trainingProgress * 100).toFixed(2)}% (${formatTime(train_sec)}/${formatTime(train_sec / trainingProgress)})`" />
            </div>
          </q-linear-progress>

          <div class="row q-col-gutter-x-md">
            <div class="col-6">
              <process-console process="train_task" style="height: 400px;" />
            </div>
            <div class="col-6">
              <Line class="bg-dark" :data="data" :options="options" />
            </div>
          </div>
        </div>
      </q-card-section>
    </q-card>
  </q-dialog>
</template>

<script setup>
import { defineEmits, defineProps, computed, onMounted, ref, shallowRef, onUnmounted, watch } from 'vue';
// import { useTraining } from 'src/composables/useTraining';
import ProcessConsole from 'src/components/v2/ProcessConsole.vue';
import { Line } from 'vue-chartjs';
import { Notify } from 'quasar';
import { api } from 'src/boot/axios';
import { useSocket } from 'src/composables/useSocket';
import CheckpointInfo from './CheckpointInfo.vue';
import { useI18n } from 'vue-i18n';

const { t } = useI18n();

import {
  Chart as ChartJS,
  Title,
  Tooltip,
  Legend,
  BarElement,
  PointElement, // <-- 이 부분을 추가해야 합니다!
  LineElement,  // <-- 라인 차트의 '선'을 위해 이것도 필요합니다.
  CategoryScale, // <-- 여기에 꼭 포함!
  LinearScale
} from 'chart.js';

ChartJS.register(Title, Tooltip, Legend, BarElement, PointElement, LineElement, CategoryScale, LinearScale);

const emit = defineEmits(['update:modelValue', 'cpRemoved']);
const props = defineProps({
  checkpoint: {
    type: Object,
  },
  isTraining: {
    type: Boolean,
    default: false
  }
});

const { socket } = useSocket();

// 종료 상태(finished/failed/canceled)에서는 다이얼로그가 남아 있고 닫기 전용
// 버튼만 노출 — 결과를 확인한 뒤 사용자가 직접 닫는다.
const isTerminal = computed(() => {
    const s = props.checkpoint?.status;
    return s === 'finished' || s === 'failed' || s === 'canceled';
});

const isFailed = computed(() => props.checkpoint?.status === 'failed');
const isCanceled = computed(() => props.checkpoint?.status === 'canceled');

// 종료 사유에 맞춰 버튼 색/라벨을 분기 — 실패를 초록색 "완료"로 위장하지 않도록.
const terminalColor = computed(() => {
    if (isFailed.value) return 'negative';
    if (isCanceled.value) return 'grey-7';
    return 'positive';
});
const terminalLabel = computed(() => {
    if (isFailed.value) return t('trainFailedBtn');
    if (isCanceled.value) return t('trainCanceledBtn');
    return t('trainCompletedBtn');
});

const trainingProgress = ref(0);
const data = shallowRef({
    labels: [],
    datasets: [{
        label: t('trainLossTrain'),
        backgroundColor: 'rgba(192, 0, 192, 1)',
        borderColor: 'rgba(192, 0, 192, 1)',
        data: [],
    }, {
        label: t('trainLossValidate'),
        backgroundColor: 'rgba(192, 192, 0, 1)',
        borderColor: 'rgba(192, 192, 0, 1)',
        data: []
    }]
});

const options = {
  responsive: true,
  maintainAspectRatio: false
};

watch(() => props.isTraining, (newVal) => {
    if (!newVal) {
        trainingProgress.value = 0;
        data.value.labels = [];
        data.value.datasets[0].data = [];
        data.value.datasets[1].data = [];
    }
});


function addLoss(epoch, trainLoss, valLoss) {
    if (data.value.labels.length > 50) {
        data.value.labels.shift();
        data.value.datasets[0].data.shift();
        data.value.datasets[1].data.shift();
    }
    const newLabels = [...data.value.labels, epoch];
    const newTrainData = [...data.value.datasets[0].data, trainLoss];
    const newValData = [...data.value.datasets[1].data, valLoss];
    data.value = {
        labels: newLabels,
        datasets: [{
            ...data.value.datasets[0],
            data: newTrainData
        }, {
            ...data.value.datasets[1],
            data: newValData
        }]
    };
}

const train_sec = ref(0);

// Format seconds to HH:MM:SS
function formatTime(seconds) {
    const hrs = Math.floor(seconds / 3600);
    const mins = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    return `${hrs.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

const onLogTrainTask = (logData) => {
    if (logData.id === 'train_task') {
        if (logData.message.includes('[TRAIN_LOG]')) {
          const trainLogStr = logData.message.replace('[TRAIN_LOG] ', '');
          const trainLogJson = JSON.parse(trainLogStr);
          trainingProgress.value = trainLogJson.epoch / trainLogJson.total_epoch;
          addLoss(trainLogJson.epoch, trainLogJson.train_loss, trainLogJson.val_loss);
          train_sec.value = trainLogJson.train_time_sec;
      }
    }
};
// const { trainingProgress, data, options } = useTraining();

async function stopTraining() {
    if (!props.isTraining) return;
    // 새 큐 라우트: DELETE /api/train/queue/<id> — 실행 중이면 stop 신호.
    // 백엔드는 'stopping' 응답 후 runner가 빠져나오면 status=canceled로 마감.
    // 다이얼로그는 즉시 닫고 큐 패널이 변화 추적하도록 한다.
    try {
        await api.delete(`/train/queue/${props.checkpoint.id}`);
        emit('update:modelValue', false);
        emit('cpRemoved', props.checkpoint.id);
    } catch (error) {
        Notify.create({ color: 'negative', message: t('trainStopFailed', { error }) });
    }
}

function cancelTraing() {
    if (props.isTraining) return;
    // 새 큐 라우트: DELETE /api/train/queue/<id> — 큐 대기 중이면 즉시 canceled.
    api.delete(`/train/queue/${props.checkpoint.id}`).then(() => {
        emit('update:modelValue', false);
        emit('cpRemoved', props.checkpoint.id);
    }).catch((error) => {
        Notify.create({ color: 'negative', message: t('trainCancelFailed', { error }) });
    })
}

onMounted(() => {
    socket.on('task_log', onLogTrainTask);
});

onUnmounted(() => {
    socket.off('task_log', onLogTrainTask);
});

</script>
