<template>
    <q-dialog v-model="show" persistent>
        <q-card dark class="bg-dark border-rounded border-white" style="min-width: 640px; max-width: 820px;">
            <q-card-section class="row items-center q-pb-sm">
                <div>
                    <div class="text-h6 text-white">{{ $t('pipelineGuideTitle') }}</div>
                    <div class="text-caption text-grey-5">{{ $t('pipelineGuideSubtitle') }}</div>
                </div>
                <q-space />
                <q-btn icon="close" flat round dense color="white" v-close-popup />
            </q-card-section>
            <q-separator color="white" />

            <q-card-section style="max-height: 70vh; overflow-y: auto;">
                <q-stepper
                    v-model="step"
                    vertical
                    color="primary"
                    dark
                    flat
                    animated
                    header-nav
                >
                    <q-step
                        v-for="(s, idx) in steps"
                        :key="s.name"
                        :name="s.name"
                        :title="$t(s.key + 'Title')"
                        :icon="s.icon"
                        :done="step > s.name"
                    >
                        <div class="q-mb-md">
                            <div class="text-caption text-grey-5 q-mb-xs">📍 {{ $t('pipelineWhereLabel') }}</div>
                            <div class="text-body2 text-white">{{ $t(s.key + 'Where') }}</div>
                        </div>
                        <div class="q-mb-md">
                            <div class="text-caption text-grey-5 q-mb-xs">🔧 {{ $t('pipelineWhatLabel') }}</div>
                            <div class="text-body2 text-white">{{ $t(s.key + 'What') }}</div>
                        </div>
                        <div class="q-mb-md">
                            <div class="text-caption text-primary q-mb-xs">💡 {{ $t('pipelineWhyLabel') }}</div>
                            <div class="text-body2 text-white">{{ $t(s.key + 'Why') }}</div>
                        </div>
                        <q-stepper-navigation>
                            <q-btn
                                v-if="idx > 0"
                                flat
                                color="white"
                                @click="step = s.name - 1"
                                :label="$t('pipelinePrev')"
                                class="q-mr-sm"
                            />
                            <q-btn
                                v-if="idx < steps.length - 1"
                                color="primary"
                                @click="step = s.name + 1"
                                :label="$t('pipelineNext')"
                            />
                            <q-btn
                                v-else
                                color="primary"
                                v-close-popup
                                :label="$t('pipelineClose')"
                            />
                        </q-stepper-navigation>
                    </q-step>
                </q-stepper>
            </q-card-section>
        </q-card>
    </q-dialog>
</template>

<script setup>
import { ref, computed } from 'vue'

const props = defineProps({
    modelValue: { type: Boolean, default: false },
})
const emit = defineEmits(['update:modelValue'])

const show = computed({
    get: () => props.modelValue,
    set: (v) => emit('update:modelValue', v),
})

const step = ref(1)

const steps = [
    { name: 1, key: 'pipelineStep1', icon: 'videocam' },
    { name: 2, key: 'pipelineStep2', icon: 'precision_manufacturing' },
    { name: 3, key: 'pipelineStep3', icon: 'extension' },
    { name: 4, key: 'pipelineStep4', icon: 'workspaces' },
    { name: 5, key: 'pipelineStep5', icon: 'folder_open' },
    { name: 6, key: 'pipelineStep6', icon: 'school' },
    { name: 7, key: 'pipelineStep7', icon: 'psychology' },
    { name: 8, key: 'pipelineStep8', icon: 'account_tree' },
]
</script>
