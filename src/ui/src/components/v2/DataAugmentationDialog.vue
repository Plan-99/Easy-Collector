<template>
    <q-card class="text-white full-height column">
        <q-card-section class="row items-center bg-dark text-white">
            <div class="text-h6">Augment Dataset</div>
            <q-space />
            <q-btn icon="close" flat round dense v-close-popup />
        </q-card-section>
        <q-separator></q-separator>
        <q-card-section class="bg-secondary q-px-xl q-py-lg col">
            <div class="row q-col-gutter-lg">
                <div class="col-md-4 col-sm-12">
                    <q-input
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        :label="$t('datasetName')"
                        v-model="form.name"
                    />

                    <q-list 
                        bordered
                        class="border-rounded q-mt-md"
                        separator
                        dark
                    >
                        <q-expansion-item
                            expand-separator
                            icon="brightness_6"
                            label="Lightness"
                            caption="Adjust image brightness"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section class="q-pa-md">
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

                        <q-item tag="label" v-ripple>
                            <q-item-section>
                                <q-item-label>HSV Augmentation</q-item-label>
                                <q-item-label caption>Randomly adjust hue, saturation, and value for each episode.</q-item-label>
                            </q-item-section>
                            <q-item-section avatar>
                                <q-checkbox v-model="form.hsv" @update:model-value="addConfig" />
                            </q-item-section>
                        </q-item>

                        <q-expansion-item
                            expand-separator
                            icon="crop_square"
                            label="Disturbances"
                            caption="Add random disturbances"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <q-input
                                        v-model.number="form.rectangles.count"
                                        type="number"
                                        label="Number of Disturbances"
                                        outlined
                                        dense
                                        dark
                                        bg-color="dark"
                                        @change="addConfig"
                                    />
                                    <q-checkbox
                                        v-model="form.rectangles.randomColor"
                                        label="Random Color"
                                        class="q-mb-md"
                                        @update:model-value="addConfig"
                                        color="primary"
                                        dark
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
                            <q-card class="bg-secondary" dark>
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
                            <q-card class="bg-secondary" dark>  
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

                        <q-expansion-item
                            expand-separator
                            icon="camera"
                            label="Prospective Transform"
                            caption="Change image perspective"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <div class="text-caption">Scale Factor</div>
                                    <q-slider
                                        v-model="form.prospective.scale_factor"
                                        :min="0"
                                        :max="50"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">Degrees</div>
                                    <q-slider
                                        v-model="form.prospective.degrees"
                                        :min="-180"
                                        :max="180"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">Shear</div>
                                    <q-slider
                                        v-model="form.prospective.shear"
                                        :min="-45"
                                        :max="45"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">Perspective</div>
                                    <q-slider
                                        v-model="form.prospective.perspective"
                                        :min="-100"
                                        :max="100"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>
                    </q-list>
                </div>
                <div class="col-md-8 col-sm-12">
                    <q-card flat bordered dark class="bg-dark border-rounded border-white">
                        <q-card-section>
                            <div class="text-subtitle1">Preview from {{ dataset ? dataset.name : '' }}</div>
                        </q-card-section>
                        <q-separator color="white" />
                        <q-card-section class="flex flex-center">
                            <hdf5-viewer
                                v-if="dataset && augmentationPreviewFile"
                                :path="`${dataset.id}/${augmentationPreviewFile.name}`"
                                class="full-width full-height q-gutter-x-sm"
                                style="width: 100%; height: 100%;"
                                image-class="border-rounded border-white"
                            />
                        </q-card-section>
                        <div class="row justify-center q-ma-md q-gutter-sm">
                            <q-btn label="Start Augmentation" outline color="primary" @click="augmentDataset" v-if="!processing && !progress" />
                            <q-linear-progress
                                :value="progress"
                                color="primary"
                                track-color="black"
                                size="30px"
                                instant-feedback
                                v-else
                            >
                                <div class="absolute-full flex flex-center">
                                    <q-badge color="white" text-color="dark" :label="`${Number(progress * 100).toFixed(0)}%`" />
                                </div>
                            </q-linear-progress>
                        </div>
                    </q-card>
                </div>
            </div>
        </q-card-section>
    </q-card>
</template>

<script setup>
import { ref, defineProps, onMounted, onUnmounted } from 'vue';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';
import Hdf5Viewer from 'src/components/v2/Hdf5Viewer.vue';
import { useSocket } from 'src/composables/useSocket';

const { socket } = useSocket()

const props = defineProps({
  taskId: {
    type: Number,
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
    },
    prospective: {
        scale_factor: 0,
        degrees: 0,
        shear: 0,
        perspective: 0,
    },
    hsv: false
});

function augmentDataset() {
    if (!props.dataset) {
        Notify.create({
            color: 'negative',
            message: 'Please select a dataset to augment.'
        });
        return;
    }

    processing.value = true

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
    form.value.name = props.dataset ? props.dataset.name + '_' + new Date().toISOString() : '';
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

const progress = ref(0)
const processing = ref(false)

onMounted(() => {
    startDialog()
    socket.on('augmentation_progress', (data) => {
        progress.value = data.progress
        if (progress.value === 1) {
            processing.value = false
            progress.value = 0
        }
    })
})

onUnmounted(() => {
    socket.off('augmentation_progress')
})

</script>