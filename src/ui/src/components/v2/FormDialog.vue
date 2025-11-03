<template>
    <q-dialog v-model="isOpen" persistent transition-show="slide-up" transition-hide="slide-down">
        <q-card style="min-width: 600px;" class="border-rounded border-white">
            <q-card-section class="bg-dark text-white row">
                <div class="text-h6">{{ props.title }}</div>
                <q-space></q-space>
                <q-btn dense color="white" round icon="close" text-color="dark" @click="isOpen = false"/>
            </q-card-section>

            <q-separator />

            <q-card-section class="bg-secondary q-px-xl q-py-lg">
                <div v-for="(data, index) in form.filter((d) => d.show === undefined || d.show(form))" :key="index"
                >
                    <div class="text-white row">
                        <div class="q-mr-md">{{ data.label }}</div>
                        <q-icon name="error" size="xs" v-if="data.validated === false" color="red"></q-icon>
                    </div>
                    <q-input
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="data.value"
                        type="text"
                        :autofocus="index === 0"
                        class="q-mb-md q-mt-xs"
                        v-if="data.type === 'text'"
                    />
                    <q-input
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model.number="data.value"
                        type="number"
                        :autofocus="index === 0"
                        class="q-mb-md q-mt-xs"
                        v-else-if="data.type === 'number'"
                    ></q-input>
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="data.value"
                        map-options
                        emit-value
                        :options="data.options || []"
                        class="q-mb-md q-mt-xs"
                        v-else-if="data.type === 'select'"
                    />
                    <slot :name="data.key" v-else-if="data.type === 'custom'"></slot>
                </div>
                <div class="row flex-center q-mt-xl">
                    <q-btn :label="cancelButtonLabel" color="white" text-color="dark" class="q-mr-md" @click="isOpen = false" />
                    <q-btn :label="okButtonLabel" color="positive" unelevated @click="submitForm" />
                </div>
            </q-card-section>
        </q-card>
    </q-dialog>
</template>
<script setup>
import { ref, watch } from 'vue';
const props = defineProps({
    modelValue: {
        type: Boolean,
        required: true
    },
    title: {
        type: String,
        default: ''
    },
    form: {
        type: Array,
        default: () => []
    },
    okButtonLabel: {
        type: String,
        default: 'OK'
    },
    cancelButtonLabel: {
        type: String,
        default: 'Cancel'
    }
});
const emit = defineEmits(['update:modelValue', 'submit']);
const isOpen = ref(props.modelValue);
watch(() => props.modelValue, (newVal) => {
    isOpen.value = newVal;
});
watch(isOpen, (newVal) => {
    emit('update:modelValue', newVal);
});

function submitForm() {
    const formData = {};
    props.form.forEach((field) => {
        if (!(field.show === undefined || field.show(props.form))) {
            return;
        }
        if (field.value === '' || field.value === null || field.value === undefined) {
            // Notify.create({
            //     type: 'negative',
            //     message: `Please fill in the ${field.label} field.`
            // });
            field.validated = false;
            return;
        }
        formData[field.key] = field.value;
    });
    if (props.form.some((field) => field.validated === false)) {
        return;
    }
    emit('submit', {
        ...formData
    });
    isOpen.value = false;
}
// function closeDialog() {
//     isOpen.value = false;
// }
</script>