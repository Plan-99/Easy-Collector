<template>
    <q-page class="q-pa-md">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="dataset in datasets" :key="dataset">
                <q-card class="cursor-pointer" @click="showDatasetFiles(dataset)">
                    <q-card-section class="text-center">
                        <q-icon name="folder" size="100px" color="amber" />
                        <div class="text-h6">{{ dataset }}</div>
                    </q-card-section>
                </q-card>
            </div>
        </div>

        <q-dialog v-model="showFilesDialog">
            <q-card style="min-width: 350px">
                <q-card-section>
                    <div class="text-h6">{{ selectedDataset }}</div>
                </q-card-section>

                <q-list bordered separator>
                    <q-item v-for="file in files" :key="file">
                        <q-item-section>{{ file }}</q-item-section>
                    </q-item>
                </q-list>

                <q-card-actions align="right">
                    <q-btn flat label="Close" color="primary" v-close-popup />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { ref, onMounted } from 'vue';
import { api } from 'src/boot/axios';

const datasets = ref([]);
const files = ref([]);
const selectedDataset = ref(null);
const showFilesDialog = ref(false);

function listDatasets() {
    api.get('/datasets').then(response => {
        datasets.value = response.data.datasets;
    });
}

function showDatasetFiles(dataset) {
    selectedDataset.value = dataset;
    api.get(`/datasets/${dataset}`).then(response => {
        files.value = response.data.files;
        showFilesDialog.value = true;
    });
}

onMounted(() => {
    listDatasets();
});
</script>
