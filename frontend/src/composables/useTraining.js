import { ref, onMounted, onUnmounted, shallowRef } from 'vue';
import { useSocket } from 'src/composables/useSocket';
import { api } from 'src/boot/axios';

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

// 차트 구성 요소를 등록합니다.
ChartJS.register(Title, Tooltip, Legend, BarElement, PointElement, LineElement, CategoryScale, LinearScale);

import { Notify } from 'quasar';

export function useTraining() {
    const trainingProgress = ref(0);
    const data = shallowRef({
        labels: [],
        datasets: [{
            label: 'Train Loss',
            backgroundColor: 'rgba(0, 0, 192, 1)',
            borderColor: 'rgba(0, 0, 192, 1)',
            data: [],
        }, {
            label: 'Validate Loss',
            backgroundColor: 'rgba(0, 192, 0, 1)',
            borderColor: 'rgba(0, 192, 0, 1)',
            data: []
        }]
    });

    const options = {
      responsive: true,
      maintainAspectRatio: false
    };

    const { socket } = useSocket();

    const unfinishedCheckpoints = ref([]);
    function getUnfinishedCheckpoint() {
        return api.get(`/checkpoints`, {
            params: {
                where: `status,!=,finished`,
            }
        }).then(response => {
            unfinishedCheckpoints.value = response.data.checkpoints
        })
    }


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

    const onStopProcess = (stopData) => {
        if (stopData.id === 'train_task') {
            const trainingCheckpoint = unfinishedCheckpoints.value.find(cp => cp.status === 'training');
            if (!trainingCheckpoint) return;
            api.get(`/checkpoint/${trainingCheckpoint.value.id}/:check_create_successed`).then(response => {
                if (response.data.check_create_successed) {
                    Notify.create({ color: 'positive', message: 'Training completed successfully.' });
                } else {
                    Notify.create({ color: 'negative', message: 'Training failed.' });
                }
            }).catch(error => {
                Notify.create({ color: 'negative', message: `Error stopping training: ${error}` });
            });
        }
    };

    function startTraining(checkpoint_id) {
        api.post('/task:start_training', { checkpoint_id }).catch(error => {
            Notify.create({ color: 'negative', message: `Error starting training: ${error}` });
        })
    }

    const watchingCheckpoint = ref(null);
    function watchCheckPoint(checkpoint) {
        watchingCheckpoint.value = checkpoint;
    }

    function onStartProcess(data) {
        if (data.id === 'train_task') {
            getUnfinishedCheckpoint().then(() => {
                watchingCheckpoint.value = unfinishedCheckpoints.value.at(-1)
            });
        }
    }


    onMounted(() => {
        socket.on('log_train_task', onLogTrainTask);
        socket.on('stop_process', onStopProcess);
        socket.on('start_process', onStartProcess);
    });

    onUnmounted(() => {
        socket.off('log_train_task', onLogTrainTask);
        socket.off('stop_process', onStopProcess);
        socket.off('start_process', onStartProcess);
    });

    return {
        trainingProgress,
        data,
        options,
        getUnfinishedCheckpoint,
        unfinishedCheckpoints,
        startTraining,
        watchingCheckpoint,
        watchCheckPoint,
    };
}
