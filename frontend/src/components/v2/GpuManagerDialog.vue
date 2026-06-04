<template>
  <q-dialog
    :model-value="modelValue"
    @update:model-value="$emit('update:modelValue', $event)"
    @show="onShow"
    @hide="onHide"
  >
    <q-card dark class="bg-dark text-white" style="min-width: 560px; max-width: 92vw;">
      <q-card-section class="row items-center q-pb-sm">
        <q-icon name="memory" color="primary" size="sm" class="q-mr-sm" />
        <div class="text-h6">{{ $t('gpuManagerTitle') }}</div>
        <q-space />
        <q-btn flat dense round icon="refresh" :loading="loading" @click="refresh">
          <q-tooltip>{{ $t('gpuRefresh') }}</q-tooltip>
        </q-btn>
        <q-btn flat dense round icon="close" v-close-popup />
      </q-card-section>
      <q-separator dark />

      <q-card-section style="max-height: 72vh; overflow: auto;">
        <div v-if="status && !status.available" class="text-orange q-mb-md">
          {{ $t('gpuUnavailable') }}
        </div>

        <!-- GPU status -->
        <div v-for="g in gpus" :key="g.index" class="q-mb-md">
          <div class="row items-center q-mb-xs">
            <div class="text-subtitle2 text-primary">GPU {{ g.index }} · {{ g.name }}</div>
            <q-space />
            <q-badge color="grey-8" class="q-mr-xs">{{ $t('gpuUtil') }} {{ g.util_pct }}%</q-badge>
            <q-badge :color="g.temp_c >= 80 ? 'negative' : 'grey-8'">{{ g.temp_c }}°C</q-badge>
          </div>
          <q-linear-progress
            :value="g.mem_total_mb ? g.mem_used_mb / g.mem_total_mb : 0"
            size="20px"
            :color="memColor(g)"
            track-color="grey-9"
            rounded
          >
            <div class="absolute-full flex flex-center">
              <span class="text-caption text-white">{{ g.mem_used_mb }} / {{ g.mem_total_mb }} MiB</span>
            </div>
          </q-linear-progress>
        </div>

        <!-- Large models -->
        <div class="text-subtitle2 text-primary q-mt-md q-mb-xs">{{ $t('gpuModels') }}</div>
        <q-card
          v-for="m in models"
          :key="m.id"
          dark
          flat
          bordered
          class="bg-grey-10 q-mb-sm"
        >
          <q-card-section class="row items-center q-py-sm">
            <div>
              <div class="text-body2">{{ m.name }}</div>
              <div class="text-caption">
                <span v-if="!m.installed" class="text-grey">{{ $t('gpuModelNotInstalled') }}</span>
                <span v-else-if="m.loaded" class="text-green">● {{ $t('gpuModelLoaded') }}</span>
                <span v-else class="text-grey-5">○ {{ $t('gpuModelNotLoaded') }}</span>
              </div>
            </div>
            <q-space />
            <q-btn
              v-if="m.installed && !m.loaded"
              color="primary"
              dense
              unelevated
              icon="play_arrow"
              :label="$t('gpuLoad')"
              :loading="!!busy[m.id]"
              @click="loadModel(m.id)"
            />
            <q-btn
              v-else-if="m.installed && m.loaded"
              color="red"
              dense
              outline
              icon="eject"
              :label="$t('gpuUnload')"
              :loading="!!busy[m.id]"
              @click="unloadModel(m.id)"
            />
          </q-card-section>
        </q-card>
        <div class="text-caption text-grey q-mt-xs">{{ $t('gpuModelAutoloadNote') }}</div>

        <!-- GPU processes -->
        <div class="text-subtitle2 text-primary q-mt-md q-mb-xs">{{ $t('gpuProcesses') }}</div>
        <q-markup-table v-if="processes.length" dark flat dense class="bg-grey-10">
          <thead>
            <tr>
              <th class="text-left">PID</th>
              <th class="text-left">{{ $t('gpuProcessName') }}</th>
              <th class="text-right">{{ $t('gpuProcessMem') }}</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="p in processes" :key="p.pid">
              <td>{{ p.pid }}</td>
              <td>{{ p.name }}</td>
              <td class="text-right">{{ p.mem_mb }} MiB</td>
            </tr>
          </tbody>
        </q-markup-table>
        <div v-else class="text-caption text-grey-5">{{ $t('gpuNoProcesses') }}</div>
      </q-card-section>
    </q-card>
  </q-dialog>
</template>

<script setup>
import { ref, reactive, computed } from 'vue'
import { api } from 'src/boot/axios'
import { Notify } from 'quasar'
import { useI18n } from 'vue-i18n'

defineProps({ modelValue: Boolean })
defineEmits(['update:modelValue'])

const { t } = useI18n()
const status = ref(null)
const loading = ref(false)
const busy = reactive({})
let timer = null

const gpus = computed(() => status.value?.gpus || [])
const models = computed(() => status.value?.models || [])
const processes = computed(() => status.value?.processes || [])

function memColor (g) {
  const r = g.mem_total_mb ? g.mem_used_mb / g.mem_total_mb : 0
  return r >= 0.9 ? 'negative' : r >= 0.7 ? 'orange' : 'primary'
}

async function refresh () {
  loading.value = true
  try {
    const { data } = await api.get('/gpu/status')
    status.value = data
  } catch (e) {
    // keep last snapshot; surface only the first failure quietly
  } finally {
    loading.value = false
  }
}

function onShow () {
  refresh()
  timer = setInterval(refresh, 3000)
}
function onHide () {
  if (timer) { clearInterval(timer); timer = null }
}

async function loadModel (id) {
  busy[id] = true
  try {
    await api.post(`/gpu/models/${id}:load`)
    Notify.create({ color: 'positive', message: t('gpuLoadedToast') })
    await refresh()
  } catch (e) {
    Notify.create({ color: 'negative', message: e?.response?.data?.message || e.message })
  } finally {
    busy[id] = false
  }
}

async function unloadModel (id) {
  busy[id] = true
  try {
    await api.post(`/gpu/models/${id}:unload`)
    Notify.create({ color: 'info', message: t('gpuUnloadedToast') })
    await refresh()
  } catch (e) {
    Notify.create({ color: 'negative', message: e?.response?.data?.message || e.message })
  } finally {
    busy[id] = false
  }
}
</script>
