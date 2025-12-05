<template>
  <q-dialog full-width>
    <q-card class="bg-secondary border-rounded border-white" dark>
      <q-card-section class="row bg-dark text-white">
        <div class="text-h6">{{ checkpoint.name }}</div>
        <q-space></q-space>
        <q-btn dense color="white" round icon="close" text-color="dark" @click="$emit('update:modelValue', false)"/>
      </q-card-section>
      <q-separator color="white"></q-separator>
      <q-card-section>
        <checkpoint-info :checkpoint="props.checkpoint" />
      </q-card-section>
      <q-card-section>
        <div class="text-center">
            <q-btn color="negative" label="Stop Training" @click="stopTraining" v-if="isTraining" />
            <q-btn color="primary" label="Cancel Training" @click="cancelTraing" v-else />
        </div>
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
              <q-badge color="white" text-color="dark" :label="`${Number(trainingProgress * 100).toFixed(2)}%`" />
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
import { defineEmits, defineProps, onMounted, ref, shallowRef, onUnmounted, watch } from 'vue';
// import { useTraining } from 'src/composables/useTraining';
import ProcessConsole from 'src/components/v2/ProcessConsole.vue';
import { Line } from 'vue-chartjs';
import { Notify } from 'quasar';
import { api } from 'src/boot/axios';
import { useSocket } from 'src/composables/useSocket';
import CheckpointInfo from './CheckpointInfo.vue';

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

const trainingProgress = ref(0);
const data = shallowRef({
    labels: [],
    datasets: [{
        label: 'Train Loss',
        backgroundColor: 'rgba(192, 0, 192, 1)',
        borderColor: 'rgba(192, 0, 192, 1)',
        data: [],
    }, {
        label: 'Validate Loss',
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

const onLogTrainTask = (logData) => {
    if (logData.log.includes('[TRAIN_LOG]')) {
        const trainLogStr = logData.log.replace('[TRAIN_LOG] ', '');
        const trainLogJson = JSON.parse(trainLogStr);
        trainingProgress.value = trainLogJson.epoch / trainLogJson.total_epoch;
        addLoss(trainLogJson.epoch, trainLogJson.train_loss, trainLogJson.val_loss);
    }
};
// const { trainingProgress, data, options } = useTraining();

async function stopTraining() {
    if (!props.isTraining) return;
    try {
        await api.post('/task:stop_training');
    } catch (error) {
        Notify.create({ color: 'negative', message: `${error}: Failed to stop training.` });
    }
    emit('update:modelValue', false);
}

function cancelTraing() {
    if (props.isTraining) return;
    api.post('/task:cancel_training', {
        checkpoint_id: props.checkpoint.id
    }).then(() => {
        emit('update:modelValue', false);
        emit('cpRemoved', props.checkpoint.id);
    }).catch((error) => {
        Notify.create({ color: 'negative', message: `${error}: Failed to cancel training.` });
    })
}

onMounted(() => {
    socket.on('log_train_task', onLogTrainTask);
});

onUnmounted(() => {
    socket.off('log_train_task', onLogTrainTask);
});

</script>
