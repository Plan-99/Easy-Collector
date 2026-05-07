<template>
    <q-card class="text-white full-height column">
        <q-card-section class="row items-center bg-dark text-white">
            <div class="text-h6">{{ $t('augmentDatasetTitle') }}</div>
            <q-space />
            <q-btn icon="close" flat round dense v-close-popup />
        </q-card-section>
        <q-separator></q-separator>
        <q-card-section class="bg-secondary q-px-xl q-py-lg col">
            <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentIntro')" />
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
                            :label="$t('augLightness')"
                            :caption="$t('augLightnessCaption')"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section class="q-pa-md">
                                    <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentLightness')" />
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
                            icon="palette"
                            :label="$t('augHsv')"
                            :caption="$t('augHsvCaption')"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentHSV')" />
                                    <q-checkbox
                                        v-model="form.hsv.random"
                                        :label="$t('augHsvRandom')"
                                        @update:model-value="addConfig"
                                        class="q-mb-md"
                                    />
                                    <div class="text-caption">{{ $t('augHsvHue') }}</div>
                                    <q-slider
                                        v-model="form.hsv.h"
                                        :min="0"
                                        :max="1"
                                        :step="0.01"
                                        label
                                        label-always
                                        :disable="form.hsv.random"
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">{{ $t('augHsvSaturation') }}</div>
                                    <q-slider
                                        v-model="form.hsv.s"
                                        :min="0"
                                        :max="1"
                                        :step="0.01"
                                        label
                                        label-always
                                        :disable="form.hsv.random"
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">{{ $t('augHsvValue') }}</div>
                                    <q-slider
                                        v-model="form.hsv.v"
                                        :min="0"
                                        :max="1"
                                        :step="0.01"
                                        label
                                        label-always
                                        :disable="form.hsv.random"
                                        @change="addConfig"
                                    />
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>

                        <q-expansion-item
                            expand-separator
                            icon="crop_square"
                            :label="$t('augDisturbances')"
                            :caption="$t('augDisturbancesCaption')"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentDisturbances')" />
                                    <q-input
                                        v-model.number="form.rectangles.count"
                                        type="number"
                                        :label="$t('augDisturbancesCount')"
                                        outlined
                                        dense
                                        dark
                                        bg-color="dark"
                                        @change="addConfig"
                                    />
                                    <q-checkbox
                                        v-model="form.rectangles.randomColor"
                                        :label="$t('augDisturbancesRandomColor')"
                                        class="q-mb-md"
                                        @update:model-value="addConfig"
                                        color="primary"
                                        dark
                                    />
                                    <div class="text-caption q-mb-sm">{{ $t('augDisturbancesRectColor') }}</div>
                                    <q-color v-model="form.rectangles.color" :disable="form.rectangles.randomColor"
                                        @change="addConfig"
                                    />
                                </q-card-section>
                            </q-card>
                        </q-expansion-item>

                        <q-expansion-item
                            expand-separator
                            icon="grain"
                            :label="$t('augSaltPepper')"
                            :caption="$t('augSaltPepperCaption')"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentSaltPepper')" />
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
                            :label="$t('augGaussian')"
                            :caption="$t('augGaussianCaption')"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentGaussian')" />
                                    <div class="text-caption">{{ $t('augGaussianMean') }}</div>
                                    <q-slider
                                        v-model="form.gaussian.mean"
                                        :min="-255"
                                        :max="255"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">{{ $t('augGaussianSigma') }}</div>
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
                            :label="$t('augProspective')"
                            :caption="$t('augProspectiveCaption')"
                        >
                            <q-card class="bg-secondary" dark>
                                <q-card-section>
                                    <TutorialHint class="q-mb-md" :text="$t('tutorialAugmentPerspective')" />
                                    <div class="text-caption">{{ $t('augProspectiveScaleFactor') }}</div>
                                    <q-slider
                                        v-model="form.prospective.scale_factor"
                                        :min="0"
                                        :max="50"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">{{ $t('augProspectiveDegrees') }}</div>
                                    <q-slider
                                        v-model="form.prospective.degrees"
                                        :min="-180"
                                        :max="180"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">{{ $t('augProspectiveShear') }}</div>
                                    <q-slider
                                        v-model="form.prospective.shear"
                                        :min="-45"
                                        :max="45"
                                        label
                                        label-always
                                        @change="addConfig"
                                    />
                                    <div class="text-caption">{{ $t('augProspectivePerspective') }}</div>
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
                            <div class="text-subtitle1">{{ $t('augPreviewTitle', { name: dataset ? dataset.name : '' }) }}</div>
                        </q-card-section>
                        <q-separator color="white" />
                        <q-card-section class="flex flex-center">
                            <episode-viewer
                                v-if="dataset && augmentationPreviewFile"
                                :path="`${dataset.id}/${augmentationPreviewFile.name}`"
                                class="full-width full-height q-gutter-x-sm"
                                style="width: 100%; height: 100%;"
                                image-class="border-rounded border-white"
                            />
                        </q-card-section>
                        <div class="row justify-center q-ma-md q-gutter-sm">
                            <q-btn :label="$t('augStartButton')" outline color="primary" @click="augmentDataset" v-if="!processing && !progress" />
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
import EpisodeViewer from 'src/components/v2/EpisodeViewer.vue';
import { useSocket } from 'src/composables/useSocket';
import { useI18n } from 'vue-i18n';
import TutorialHint from 'src/components/v2/TutorialHint.vue';

const { t } = useI18n();
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
    hsv: {
        h: 0,
        s: 0,
        v: 0,
        random: false
    }
});

function augmentDataset() {
    if (!props.dataset) {
        Notify.create({
            color: 'negative',
            message: t('augErrorNoDataset')
        });
        return;
    }

    processing.value = true

    api.post(`/dataset/${props.dataset.id}/augment`, form.value).then(() => {
        Notify.create({
            color: 'positive',
            message: t('augStarted')
        });
    }).catch((error) => {
        console.error('Error starting dataset augmentation:', error);
        Notify.create({
            color: 'negative',
            message: t('augStartFailed')
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
    api.post(`/dataset/${props.dataset.id}/${augmentationPreviewFile.value.name}/:read_dataset_add_config`, form.value)
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
