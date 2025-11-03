<template>
  <div class="absolute-bottom bg-dark q-pb-lg q-pr-lg">
    <div class="border-rounded bg-secondary q-pa-lg">
      <div class="row">
        <q-tabs
          v-model="model[props.tabValue]"
          dense
          align="left"
          :breakpoint="0"
          class="text-white"
          active-color="primary"
        >
          <q-tab 
            v-for="tab in tabs"
            :key="tab[props.tabValue]"
            :name="tab[props.tabValue]"
            :label="tab[props.tabLabel]"
          />
        </q-tabs>
        <q-space></q-space>
        <q-btn
          flat
          class="q-pa-sm text-white"
          icon="close"
          style="z-index: 1000"
          @click="model = null; $emit('close')"
        ></q-btn>
      </div>

      <q-separator class="q-mb-lg" color="white"/>
      <div
        v-for="tab in tabs"
        :key="tab[props.tabValue]"
        v-show="tab[props.tabValue] === model[props.tabValue]"
        style="height: 100%;"
      >
        <slot :name="tab[props.tabValue]"></slot>
      </div>
    </div>
  </div>
</template>

<script setup>
import { defineModel, defineProps } from 'vue';

// Props for the BottomTerminal component
const props = defineProps({
  // This prop is not used in the current implementation but can be used for future enhancements
  tabLabel: {
    type: String,
    default: null,
  },
  tabValue: {
    type: String,
    default: null,
  },
});
// 1. 선택된 탭을 위한 v-model (기존과 동일)
// 부모 컴포넌트에서 v-model="selectedTab"으로 사용됩니다.
const model = defineModel({ required: true, default: null });

// 2. 탭 목록 배열을 위한 v-model:tabs
// 부모 컴포넌트에서 v-model:tabs="tabsArray"로 사용됩니다.
const tabs = defineModel('tabs', {
  type: Array,
  required: true,
});
</script>