<template>
  <div class="absolute-bottom">
    <div class="row">
      <div
        v-for="tab in tabs"
        :key="tab[tabValue]"
        @click="model = tab"
        class="q-pa-sm col-1 text-center cursor-pointer"
        :class="{ 'text-white': model === tab, 'text-grey-8': model !== tab, 'bg-grey-6': model === tab, 'bg-grey-4': model !== tab }"
      >
        {{ tab[tabLabel] }}
      </div>
    </div>
    <q-separator />
    <div class="bg-grey-4" style="min-height: 350px;">
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