<template>
    <div class="row q-col-gutter-lg" v-if="props.checkpoint">
        <div class="col-4">
            <div style="border: 1px solid rgba(0,0,0,0.12);">
                <div class="row q-pa-sm bg-grey-8 text-weight-bold">
                    <div class="col-6">{{ $t('checkpointPolicyParameters') }}</div>
                    <div class="col-6">{{ $t('checkpointValueColumn') }}</div>
                </div>
                <q-scroll-area :style="`height: ${height}px;`" class="bg-dark">
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
                    <div class="col-6">{{ $t('checkpointTrainParameters') }}</div>
                    <div class="col-6">{{ $t('checkpointValueColumn') }}</div>
                </div>
                <q-scroll-area :style="`height: ${height}px;`" class="bg-dark">
                    <div v-for="(value, key) in { finetuned_from: props.checkpoint.load_model_id ? `#${props.checkpoint.load_model_id}` : '-', ...props.checkpoint.train_settings }" :key="key">
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
            <div class="rounded-borders" style="border: 1px solid rgba(0,0,0,0.12);">
                <div class="q-pa-sm bg-grey-8 text-weight-bold">
                    {{ $t('checkpointDatasets') }}
                </div>
                <q-scroll-area :style="`height: ${height}px;`" class="bg-dark q-pa-md">
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
</template>

<script setup>
import { defineProps } from 'vue';

const props = defineProps({
    checkpoint: {
        type: Object,
        required: true
    },
    height: {
        type: Number,
        default: 200
    }
});
</script>