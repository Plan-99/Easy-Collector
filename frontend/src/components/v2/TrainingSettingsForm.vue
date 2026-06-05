<template>
  <!-- TrainPage step 3 의 학습 하이퍼파라미터 폼을 그대로 컴포넌트화.
       modelValue 는 { key: { label, value, type, options, showIf, ... } } 형태의 trainingForm. -->
  <div class="row q-col-gutter-md">
    <div
      v-for="(config, key) in modelValue"
      :key="key"
      class="col-12 col-sm-6"
      v-show="!config.showIf || (modelValue[config.showIf] && modelValue[config.showIf].value)"
    >
      <q-select
        v-if="config.type === 'select'"
        dense outlined dark bg-color="dark"
        v-model="config.value"
        :options="config.options"
        :label="config.label"
        emit-value
        map-options
      >
        <template v-slot:label>
          <span>{{ config.label }}</span>
          <HyperparamHelp :policy-type="policyType" :param-key="key" />
        </template>
      </q-select>

      <q-input
        v-else-if="config.type === 'number'"
        dense outlined dark bg-color="dark"
        v-model.number="config.value"
        :label="config.label"
        type="number"
        step="any"
      >
        <template v-slot:label>
          <span>{{ config.label }}</span>
          <HyperparamHelp :policy-type="policyType" :param-key="key" />
        </template>
      </q-input>

      <div v-else-if="config.type === 'boolean'" class="row items-center no-wrap">
        <q-btn-toggle
          dense
          v-model="config.value"
          :options="[{ label: config.label, value: true }, { label: t('no'), value: false }]"
          spread
          class="col bg-dark"
        />
        <HyperparamHelp :policy-type="policyType" :param-key="key" class="q-ml-xs" />
      </div>

      <q-select
        v-else-if="config.type === 'wrist_sensor_select'"
        dense outlined dark bg-color="dark"
        v-model="config.value"
        :options="wristSensorOptions"
        :label="config.label"
        emit-value
        map-options
        multiple
        use-chips
      >
        <template v-slot:label>
          <span>{{ config.label }}</span>
          <HyperparamHelp :policy-type="policyType" :param-key="key" />
        </template>
      </q-select>

      <q-input
        v-else
        dense outlined dark bg-color="dark"
        v-model="config.value"
        :label="config.label"
        type="text"
      >
        <template v-slot:label>
          <span>{{ config.label }}</span>
          <HyperparamHelp :policy-type="policyType" :param-key="key" />
        </template>
      </q-input>
    </div>
  </div>
</template>

<script setup>
import { useI18n } from 'vue-i18n'
import HyperparamHelp from 'src/components/v2/HyperparamHelp.vue'

const { t } = useI18n()

defineProps({
  // trainingForm 객체 (부모의 reactive 객체를 그대로 받아 config.value 를 직접 수정).
  modelValue: { type: Object, required: true },
  policyType: { type: String, default: '' },
  wristSensorOptions: { type: Array, default: () => [] },
})
</script>
