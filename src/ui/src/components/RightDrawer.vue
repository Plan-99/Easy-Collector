<template>
    <div class="absolute-top-right full-height" style="z-index: 1000;">
        <div class="row full-height">
            <q-icon
                size="md"
                :name="model ? 'close' : 'settings'"
                style="top: 60px; border-radius: 15px 0 0 15px; background-color: rgba(0, 0, 0, 0.7);"
                class="cursor-pointer q-pa-sm q-py-xl text-white"
                @click="model = !model"
            ></q-icon>
            <div class="text-white" v-if="model" style=" min-width: 400px; max-width: 800px;; background-color: rgba(0, 0, 0, 0.7);">
                <slot></slot>
            </div>
        </div>
    </div>
</template>

<script setup>
import { defineModel } from 'vue';

const model = defineModel({
    type: Boolean,
    default: false,
});

</script>