<template>
    <q-dialog maximized @show="startDialog">
        <q-card>
            <q-card-section class="row items-center q-pb-none">
                <div class="text-h6">Augment Dataset</div>
                <q-space />
                <q-btn icon="close" flat round dense v-close-popup />
            </q-card-section>

            <q-card-section>
                <div class="row q-col-gutter-lg">
                    <div class="col-md-5 col-sm-12">
                        <q-input
                            v-model="form.name"
                            type="text"
                            label="New Dataset Name"
                            outlined
                            dense
                            class="q-mb-md"
                            bg-color="grey-2"
                        />

                        <q-list bordered class="rounded-borders q-my-md">
                            <q-expansion-item
                                expand-separator
                                icon="brightness_6"
                                label="Lightness"
                                caption="Adjust image brightness"
                            >
                                <q-card>
                                    <q-card-section>
                                        <q-slider
                                            v-model="form.lightness"
                                            :min="-100"
                                            :max="100"
                                            label
                                            label-always
                                            @change="addConfig"
                                        />
                                    </q-card-section>
                                </q-card>
                            </q-expansion-item>

                            <q-expansion-item
                                expand-separator
                                icon="crop_square"
                                label="Disturbances"
                                caption="Add random disturbances"
                            >
                                <q-card>
                                    <q-card-section>
                                        <q-input
                                            v-model.number="form.rectangles.count"
                                            type="number"
                                            label="Number of Disturbances"
                                            outlined
                                            dense
                                            class="q-mb-md"
                                            bg-color="grey-2"
                                            @change="addConfig"
                                        />
                                        <q-checkbox
                                            v-model="form.rectangles.randomColor"
                                            label="Random Color"
                                            class="q-mb-md"
                                            @update:model-value="addConfig"
                                        />
                                        <div class="text-caption q-mb-sm">Rectangle Color</div>
                                        <q-color v-model="form.rectangles.color" :disable="form.rectangles.randomColor" 
                                            @change="addConfig"
                                        />
                                    </q-card-section>
                                </q-card>
                            </q-expansion-item>

                            <q-expansion-item
                                expand-separator
                                icon="grain"
                                label="Salt and Pepper Noise"
                                caption="Add salt and pepper noise"
                            >
                                <q-card>
                                    <q-card-section>
                                        <q-slider
                                            v-model="form.saltAndPepper.amount"
                                            :min="0"
                                            :max="1"
                                            :step="0.01"
                                            label
                                            label-always
                                            @change="addConfig"
                                        />
                                    </q-card-section>
                                </q-card>
                            </q-expansion-item>

                            <q-expansion-item
                                expand-separator
                                icon="blur_on"
                                label="Gaussian Noise"
                                caption="Add Gaussian noise"
                            >
                                <q-card>
                                    <q-card-section>
                                        <div class="text-caption">Mean</div>
                                        <q-slider
                                            v-model="form.gaussian.mean"
                                            :min="-255"
                                            :max="255"
                                            label
                                            label-always
                                            @change="addConfig"
                                        />
                                        <div class="text-caption">Sigma</div>
                                        <q-slider
                                            v-model="form.gaussian.sigma"
                                            :min="0"
                                            :max="50"
                                            label
                                            label-always
                                            @change="addConfig"
                                        />
                                    </q-card-section>
                                </q-card>
                            </q-expansion-item>
                        </q-list>
                    </div>
                    <div class="col-md-7 col-sm-12">
                        <q-card flat bordered>
                            <q-card-section>
                                <div class="text-subtitle1">Preview from {{ dataset ? dataset.name : '' }}</div>
                            </q-card-section>
                            <q-separator />
                            <q-card-section class="flex flex-center" style="min-height: 600px;">
                                <hdf5-viewer
                                    v-if="dataset && augmentationPreviewFile"
                                    :path="`${dataset.id}/${augmentationPreviewFile.name}`"
                                    class="full-width full-height"
                                />
                            </q-card-section>
                            <div class="row justify-start q-mt-md">
                                <q-btn label="Start Augmentation" color="primary" @click="augmentDataset" />
                                <q-btn label="Cancel" color="grey-6" v-close-popup class="q-mr-sm" />
                            </div>
                        </q-card>
                    </div>
                </div>
            </q-card-section>
        </q-card>
    </q-dialog>
</template>

<script setup>
import { ref, defineProps } from 'vue';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';
import Hdf5Viewer from 'src/components/Hdf5Viewer.vue';

const props = defineProps({
  taskId: {
    type: String,
    required: true,
  },
  dataset: {
    type: Object,
    default: null,
  },
});

const form = ref({
    name: '',
    task_id: props.taskId,
    lightness: 0,
    rectangles: {
        count: 0,
        color: '#000000',
        randomColor: false
    },
    saltAndPepper: {
        amount: 0
    },
    gaussian: {
        mean: 0,
        sigma: 0
    }
});

function augmentDataset() {
    if (!props.dataset) {
        Notify.create({
            color: 'negative',
            message: 'Please select a dataset to augment.'
        });
        return;
    }

    api.post(`/dataset/${props.dataset.id}/augment`, form.value).then(() => {
        Notify.create({
            color: 'positive',
            message: 'Dataset augmentation started.'
        });
    }).catch((error) => {
        console.error('Error starting dataset augmentation:', error);
        Notify.create({
            color: 'negative',
            message: 'Failed to start dataset augmentation.'
        });
    });
}

const augmentationPreviewFile = ref(null);

function startDialog() {
    form.value.name = props.dataset ? props.dataset.name + '_augmented' : '';
    if (props.dataset) {
        api.get(`/datasets/${props.dataset.id}/:get_one`).then((res) => {
            augmentationPreviewFile.value = res.data.file; // Assuming the first file is the one to preview
        }).catch((error) => {
            console.error('Error fetching dataset for preview:', error);
        });
    }
}

function addConfig() {
    api.post(`/dataset/${props.dataset.id}/${augmentationPreviewFile.value.name}/:read_hdf5_add_config`, form.value)
}

</script>