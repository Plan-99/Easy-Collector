<template>
    <q-dialog v-model="isOpen" persistent transition-show="slide-up" transition-hide="slide-down">
        <q-card :style="`min-width: ${minWidth};`" class="border-rounded border-white">
            <q-card-section class="bg-dark text-white row">
                <div class="text-h6">{{ props.title }}</div>
                <q-space></q-space>
                <q-btn dense color="white" round icon="close" text-color="dark" @click="isOpen = false"/>
            </q-card-section>

            <q-separator />

            <q-card-section class="bg-secondary q-px-xl q-py-lg">
                <slot></slot>
                <div v-for="(data, index) in form.filter((d) => d.show === undefined || d.show(form))" :key="index"
                >
                    <div class="text-white row">
                        <div class="q-mr-md col">{{ data.label }}</div>
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
                    <q-select
                        dense
                        outlined
                        dark
                        bg-color="dark"
                        v-model="data.value"
                        map-options
                        emit-value
                        multiple
                        :max-values="data.maxValues || 'no-limit'"
                        :options="data.options || []"
                        class="q-mb-md q-mt-xs"
                        v-else-if="data.type === 'multiselect'"
                    />
                    <q-scroll-area v-else-if="data.type === 'multiselect_list'" style="height: 150px" class="q-mb-md q-mt-xs">
                        <q-list bordered separator dark v-if="data.options.length > 0">
                            <q-item clickable v-ripple  v-for="option in data.options" :key="option.value"
                                @click="{
                                    if (data.value.includes(option.value)) {
                                        data.value = data.value.filter(v => v !== option.value);
                                    } else {
                                        data.value = [...data.value, option.value];
                                    }
                                }"
                                :class="{'bg-grey-8': data.value.includes(option.value)}"
                            >
                                <q-item-section>{{ option.label }}</q-item-section>
                                <q-item-section side>
                                    <q-icon v-if="data.value.includes(option.value)" name="check" color="positive" />
                                </q-item-section>
                            </q-item>
                        </q-list>
                        <div v-else class="q-pa-md bg-dark text-white text-center border-white">
                            {{ $t('formDialogNoOptions', { label: data.label }) }}
                        </div>
                    </q-scroll-area>
                    <q-checkbox
                        v-else-if="data.type === 'checkbox'"
                        v-model="data.value"
                        :label="data.label"
                        dense
                        class="q-mb-md"
                    />
                    <slot :name="data.key" v-else-if="data.type === 'custom'"></slot>
                </div>
                <div class="row flex-center q-mt-xl">
                    <q-btn :label="cancelLabel" color="white" text-color="dark" class="q-mr-md" @click="isOpen = false" />
                    <q-btn :label="okLabel" color="positive" unelevated @click="submitForm" />
                </div>
            </q-card-section>
        </q-card>
    </q-dialog>
</template>
<script setup>
import { ref, watch, computed } from 'vue';
import { useI18n } from 'vue-i18n';

const { t } = useI18n();

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
        default: ''
    },
    cancelButtonLabel: {
        type: String,
        default: ''
    },
    minWidth: {
        type: String,
        default: '600px'
    }
});

const okLabel = computed(() => props.okButtonLabel || t('ok'));
const cancelLabel = computed(() => props.cancelButtonLabel || t('cancel'));
const emit = defineEmits(['update:modelValue', 'submit']);
const isOpen = ref(props.modelValue);
watch(() => props.modelValue, (newVal) => {
    isOpen.value = newVal;
});
watch(isOpen, (newVal) => {
    emit('update:modelValue', newVal);
});

function submitForm() {
    // 1. Reset validation status
    props.form.forEach(field => {
        field.validated = true;
    });

    let isFormValid = true;
    const formData = {};

    // 2. Validate and collect data
    props.form.forEach((field) => {
        // Skip fields that shouldn't be shown
        if (field.show !== undefined && !field.show(props.form)) {
            return;
        }

        const value = field.value;
        const isInvalid = (value === '' || value === null || value === undefined);

        if (isInvalid && field.key !== 'id' && !field.optional) {
            field.validated = false;
            isFormValid = false;
        } else {
            formData[field.key] = value;
        }
    });

    // 3. Check validity and submit
    if (!isFormValid) {
        return; // Exit if validation failed
    }
    
    emit('submit', { ...formData });
    isOpen.value = false;
}
// function closeDialog() {
//     isOpen.value = false;
// }
</script>