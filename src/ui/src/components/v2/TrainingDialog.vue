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
        <div class="row q-col-gutter-lg" v-if="props.checkpoint">
          <div class="col-4">
            <div style="border: 1px solid rgba(0,0,0,0.12);">
              <div class="row q-pa-sm bg-grey-8 text-weight-bold">
                <div class="col-6">Policy Parameters</div>
                <div class="col-6">Value</div>
              </div>
              <q-scroll-area style="height: 200px;" class="bg-dark">
                <div v-for="(value, key) in { model: props.checkpoint.policy.type, ...props.checkpoint.policy.settings }" :key="key">
                  <q-separator color="white" />
                  <div class="row q-pa-sm">
                    <div class="col-6" style="word-wrap: break-word;">{{ key }}</div>
                    <div class="col-6" style="word-wrap: break-word;">{{ value }}</div>
                  </div>
                </div>
              </q-scroll-area>
            </div>
          </div>
          <div class="col-4">
            <div class="rounded-borders" style="border: 1px solid rgba(0,0,0,0.12);">
              <div class="row q-pa-sm bg-grey-8 text-weight-bold">
                <div class="col-6">Train Parameters</div>
                <div class="col-6">Value</div>
              </div>
              <q-scroll-area style="height: 200px;" class="bg-dark">
                <div v-for="(value, key) in { finetuned_from: props.checkpoint.load_model?.name, ...props.checkpoint.train_settings }" :key="key">
                  <q-separator color="white"/>
                  <div class="row q-pa-sm">
                    <div class="col-6" style="word-wrap: break-word;">{{ key }}</div>
                    <div class="col-6" style="word-wrap: break-word;">{{ value }}</div>
                  </div>
                </div>
              </q-scroll-area>
            </div>
          </div>
          <div class="col-4 full-height">
            <!-- <div class="q-pa-md bg-dark border-rounded border-white">
              <q-card
                  class="q-pa-md bg-secondary border-rounded border-white text-white col-2 flex flex-center" 
                  v-for="dataset in props.checkpoint.dataset_info"
                  :key="dataset.id"
                >
                    <q-card-section class="q-pa-none q-mt-sm text-center">
                        <q-img
                            src="images/folder-icon.png"
                            class="cursor-pointer"
                            fit="contain"
                            width="50px"
                        >
                            <div style="background: none;" class="absolute-full flex flex-center q-mt-xs text-primary">
                                {{ dataset.episode_num }}
                            </div>
                        </q-img>
                        <div class="text-bold q-mt-md">{{ dataset.name }}</div>
                    </q-card-section>
                </q-card>
            </div> -->
            <div class="rounded-borders" style="border: 1px solid rgba(0,0,0,0.12);">
              <div class="q-pa-sm bg-grey-8 text-weight-bold">
                Datasets
              </div>
              <q-scroll-area style="height: 200px;" class="bg-dark q-pa-md">
                <div class="row q-gutter-x-md">
                  <q-card
                    class="q-pa-md bg-secondary border-rounded border-white text-white col-3 flex flex-center" 
                    v-for="dataset in props.checkpoint.dataset_info"
                    :key="dataset.id"
                  >
                      <q-card-section class="q-pa-none q-mt-sm text-center">
                          <q-img
                              src="images/folder-icon.png"
                              class="cursor-pointer"
                              fit="contain"
                              width="50px"
                          >
                              <div style="background: none;" class="absolute-full flex flex-center q-mt-xs text-primary">
                                  {{ dataset.episode_num }}
                              </div>
                          </q-img>
                          <div class="text-bold q-mt-md">{{ dataset.name }}</div>
                      </q-card-section>
                  </q-card>
                </div>
              </q-scroll-area>
            </div>
            </div>  
        </div>
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
              <process-console process="train_task" />
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
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Line } from 'vue-chartjs';
import { Notify } from 'quasar';
import { api } from 'src/boot/axios';
import { useSocket } from 'src/composables/useSocket';

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
