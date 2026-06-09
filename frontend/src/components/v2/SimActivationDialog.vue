<template>
  <q-dialog
    :model-value="modelValue"
    @update:model-value="$emit('update:modelValue', $event)"
    @show="onShow"
  >
    <q-card dark class="bg-dark text-white" style="min-width: 560px; max-width: 92vw;">
      <q-card-section class="row items-center q-pb-sm">
        <q-icon name="view_in_ar" color="primary" size="sm" class="q-mr-sm" />
        <div class="text-h6">{{ $t('simActivationTitle') }}</div>
        <q-space />
        <q-btn flat dense round icon="refresh" :loading="demo.loading" @click="refresh">
          <q-tooltip>{{ $t('simActivationRefresh') }}</q-tooltip>
        </q-btn>
        <q-btn flat dense round icon="close" v-close-popup />
      </q-card-section>
      <q-separator dark />

      <q-card-section style="max-height: 70vh; overflow: auto;">
        <div class="text-caption text-grey-5 q-mb-md">{{ $t('simActivationSubtitle') }}</div>

        <div v-if="demo.loading && !demo.demos.length" class="row flex-center q-pa-lg">
          <q-spinner color="primary" size="md" />
        </div>

        <div v-else-if="!demo.demos.length" class="text-grey q-pa-md text-center">
          {{ $t('simActivationEmpty') }}
        </div>

        <q-card
          v-for="d in demo.demos"
          :key="d.id"
          dark
          flat
          bordered
          class="q-mb-sm cursor-pointer"
          :class="selectedId === d.id ? 'bg-grey-9 selected-demo' : 'bg-grey-10'"
          @click="selectedId = d.id"
        >
          <q-card-section class="row items-center no-wrap q-py-sm">
            <q-radio
              :model-value="selectedId"
              :val="d.id"
              color="primary"
              dense
              class="q-mr-sm"
              @update:model-value="selectedId = d.id"
            />
            <div class="col">
              <div class="row items-center">
                <div class="text-body1">{{ demoName(d) }}</div>
                <q-badge
                  v-if="demo.runningDemoId === d.id"
                  color="green"
                  class="q-ml-sm"
                >
                  {{ demo.hasTopics ? $t('simActivationRunning') : $t('simActivationStarting') }}
                </q-badge>
              </div>
              <div class="text-caption text-grey-5">{{ demoDesc(d) }}</div>
              <div v-if="d.cameras && d.cameras.length" class="q-mt-xs">
                <q-badge
                  v-for="cam in d.cameras"
                  :key="cam"
                  color="grey-8"
                  class="q-mr-xs"
                >{{ cam }}</q-badge>
              </div>
            </div>
          </q-card-section>
        </q-card>

        <div v-if="demo.lastError" class="text-negative text-caption q-mt-sm">
          {{ demo.lastError }}
        </div>
      </q-card-section>

      <q-separator dark />
      <q-card-actions align="right" class="q-pa-md">
        <q-btn
          v-if="demo.runningDemoId"
          flat
          color="red"
          icon="stop"
          :label="$t('simActivationStop')"
          :loading="demo.busy"
          @click="stopDemo"
        />
        <q-space />
        <q-btn
          color="primary"
          unelevated
          icon="play_arrow"
          :label="$t('simActivationRun')"
          :disable="!selectedId"
          :loading="demo.busy"
          @click="runDemo"
        />
      </q-card-actions>
    </q-card>
  </q-dialog>
</template>

<script setup>
import { ref } from 'vue'
import { Notify } from 'quasar'
import { useI18n } from 'vue-i18n'
import { useDemoStore } from 'src/stores/demoStore.js'

defineProps({ modelValue: Boolean })
defineEmits(['update:modelValue'])

const { t, locale } = useI18n()
const demo = useDemoStore()
const selectedId = ref(null)

const isKo = () => locale.value && locale.value.startsWith('ko')
function demoName (d) {
  return isKo() ? (d.name || d.name_en) : (d.name_en || d.name)
}
function demoDesc (d) {
  return isKo() ? (d.description || d.description_en) : (d.description_en || d.description)
}

async function onShow () {
  await demo.fetchList()
  await demo.refreshStatus()
  // Preselect the running demo, else the first one.
  selectedId.value = demo.runningDemoId || (demo.demos[0] && demo.demos[0].id) || null
}

async function refresh () {
  await demo.fetchList()
  await demo.refreshStatus()
}

async function runDemo () {
  if (!selectedId.value) return
  try {
    await demo.start(selectedId.value)
    Notify.create({
      type: 'positive',
      message: t('simActivationStartedNotify', { name: selectedId.value }),
      timeout: 4000,
    })
  } catch (e) {
    Notify.create({
      type: 'negative',
      message: t('simActivationFailed', { error: e?.response?.data?.message || e.message || e }),
      timeout: 5000,
    })
  }
}

async function stopDemo () {
  try {
    await demo.stop()
    Notify.create({ type: 'info', message: t('simActivationStoppedNotify'), timeout: 2500 })
  } catch (e) {
    Notify.create({
      type: 'negative',
      message: t('simActivationFailed', { error: e?.response?.data?.message || e.message || e }),
      timeout: 5000,
    })
  }
}
</script>

<style scoped>
.selected-demo {
  border-color: var(--q-primary);
}
</style>
