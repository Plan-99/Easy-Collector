<template>
    <div class="row q-pt-md">
        <img :src="image" alt="" v-for="(image, name) in images" :key="name" class="col" style="width: 100%; height: 100%; object-fit: contain;">
    </div>
</template>
<script setup>
import { defineProps, onMounted, watch, ref, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import { LocalStorage } from 'quasar';
import { useSocket } from 'src/composables/useSocket.js';

const { socket } = useSocket();

const props = defineProps({
  path: {
    type: String,
    required: true,
  },
  config: {
    type: Object,
    default: () => ({})
  }
});

const images = ref([]);
const robot_states = ref({});

function startReadingHdf5(path) {
    api.post(`/dataset/${path}/:start_read_hdf5`, {
        sid: LocalStorage.getItem('socketId')
    }).then(() => {
        // Successfully started reading HDF5 file
    }).catch((error) => {
        console.error('Error starting to read HDF5 file:', error);
    });
}

function stopReadingHdf5(path) {
    api.post(`/dataset/${path}/:stop_read_hdf5`).then(() => {
        // Successfully stopped reading HDF5 file
    }).catch((error) => {
        console.error('Error stopping to read HDF5 file:', error);
    });
}

watch(() => props.path, (newVal, oldVal) => {
    if (newVal) {
        startReadingHdf5(newVal);
    }
    if (oldVal) {
        stopReadingHdf5(oldVal);
    }
});

onMounted(() => {
    if (props.path) {
        startReadingHdf5(props.path);
    }
    socket.on('show_episode_step', (data) => {
        // const hdf5_path = data.hdf5_path.split('/');
        // const file_name = hdf5_path.pop();
        // const dataset_id = Number(hdf5_path.pop());
        // if (!watchingDataset.value || watchingDataset.value.id !== dataset_id || !watchingFile.value || watchingFile.value.name !== file_name) {
        //     console.log(dataset_id)
        //     api.post(`/dataset/${dataset_id}/${file_name}/:stop_read_hdf5`).then(() => {
        //         // Successfully started reading HDF5 file
        //     }).catch((error) => {
        //         console.error('Error stoping to read HDF5 file:', error);
        //     });
        //     return;
        // }
        images.value = data.images
        robot_states.value = data.robot_states || {};
    });
});

onUnmounted(() => {
    socket.off('show_episode_step');
    stopReadingHdf5(props.path);
});
</script>