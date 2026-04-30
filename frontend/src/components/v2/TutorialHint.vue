<template>
    <div v-if="show" class="tutorial-hint" :class="positionClass">
        <div v-if="step != null" class="tutorial-hint__step">{{ step }}</div>
        <q-icon v-else name="lightbulb" class="tutorial-hint__icon" size="sm" />
        <div class="tutorial-hint__body">
            <div v-if="title" class="tutorial-hint__title">{{ title }}</div>
            <div class="tutorial-hint__text">
                <slot>{{ text }}</slot>
            </div>
        </div>
    </div>
</template>

<script setup>
import { computed } from 'vue'
import { useTutorialStore } from 'src/stores/tutorialStore.js'

const props = defineProps({
    text: { type: String, default: '' },
    title: { type: String, default: '' },
    step: { type: [Number, String], default: null },
    placement: {
        type: String,
        default: 'block',
        validator: (v) => ['block', 'inline'].includes(v),
    },
})

const tutorial = useTutorialStore()
const show = computed(() => tutorial.running)
const positionClass = computed(() => `tutorial-hint--${props.placement}`)
</script>

<style scoped>
.tutorial-hint {
    display: flex;
    align-items: flex-start;
    gap: 12px;
    padding: 10px 14px;
    border-radius: 10px;
    background: rgba(0, 200, 180, 0.08);
    border: 1px solid var(--q-primary);
    color: #fff;
    font-size: 13px;
    line-height: 1.45;
    box-shadow: 0 0 0 1px rgba(0, 200, 180, 0.15);
}

.tutorial-hint--block {
    width: 100%;
    margin: 8px 0;
}

.tutorial-hint--inline {
    display: inline-flex;
    margin: 4px 0;
}

.tutorial-hint__step {
    flex: 0 0 auto;
    width: 24px;
    height: 24px;
    border-radius: 50%;
    background: var(--q-primary);
    color: #000;
    font-weight: 700;
    font-size: 12px;
    display: flex;
    align-items: center;
    justify-content: center;
}

.tutorial-hint__icon {
    color: var(--q-primary);
    margin-top: 2px;
}

.tutorial-hint__body {
    flex: 1 1 auto;
    min-width: 0;
}

.tutorial-hint__title {
    font-weight: 600;
    color: var(--q-primary);
    margin-bottom: 2px;
    font-size: 13px;
}

.tutorial-hint__text {
    color: #e0e0e0;
}
</style>
