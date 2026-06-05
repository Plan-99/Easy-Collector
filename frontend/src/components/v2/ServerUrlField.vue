<template>
  <!-- TrainPage step3 의 학습 서버 URL 필드(연결 확인 버튼 + GPU 뱃지)를 컴포넌트화. -->
  <div>
    <q-input
      dense outlined dark bg-color="dark"
      :model-value="modelValue"
      @update:model-value="onInput"
      :label="t('trainServerUrl')"
      :placeholder="t('trainServerUrlPlaceholder')"
    >
      <template v-slot:append>
        <q-btn
          flat dense
          :icon="statusIcon"
          :color="statusColor"
          :loading="serverStatus === 'checking'"
          @click="checkServerHealth"
        />
      </template>
    </q-input>
    <q-badge v-if="serverGpuAvailable" color="positive" :label="t('trainServerGpuAvailable')" class="q-mt-xs" />
  </div>
</template>

<script setup>
import { ref, computed } from 'vue'
import { useI18n } from 'vue-i18n'
import { api } from 'src/boot/axios'
import { Notify } from 'quasar'

const { t } = useI18n()
const props = defineProps({ modelValue: { type: String, default: '' } })
const emit = defineEmits(['update:modelValue'])

const serverStatus = ref('unknown') // unknown | checking | connected | error
const serverGpuAvailable = ref(false)

const statusIcon = computed(() =>
  serverStatus.value === 'connected' ? 'check_circle'
    : serverStatus.value === 'checking' ? 'hourglass_empty' : 'wifi_off',
)
const statusColor = computed(() =>
  serverStatus.value === 'connected' ? 'positive'
    : serverStatus.value === 'checking' ? 'warning' : 'negative',
)

function onInput (v) {
  emit('update:modelValue', v)
  serverStatus.value = 'unknown'
  serverGpuAvailable.value = false
}

function checkServerHealth () {
  if (!props.modelValue) {
    Notify.create({ color: 'negative', message: t('trainEnterServerUrl') })
    return
  }
  serverStatus.value = 'checking'
  api.post('/remote-train/health', { server_url: props.modelValue })
    .then((res) => {
      serverStatus.value = 'connected'
      serverGpuAvailable.value = res.data.server?.gpu_available || false
      Notify.create({ color: 'positive', message: t('trainServerConnected') })
    })
    .catch(() => {
      serverStatus.value = 'error'
      serverGpuAvailable.value = false
      Notify.create({ color: 'negative', message: t('trainServerCannotConnect') })
    })
}
</script>
