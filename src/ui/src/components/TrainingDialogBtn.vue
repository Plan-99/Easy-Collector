<template>
    <q-page-sticky position="bottom-right" :offset="[18, 18]">
        <q-btn
            push
            round
            size="40px"
            color="deep-orange"
            icon="model_training"
            @click="$emit('open')"
        />
    </q-page-sticky>
</template>

<script setup>
import { defineEmits } from 'vue';

defineEmits(['open']);
</script>
