<template>
  <span
    v-if="help"
    class="inline-block q-ml-xs"
    style="vertical-align: middle; pointer-events: auto;"
    @click.stop.prevent
    @mousedown.stop
  >
    <q-btn
      flat
      dense
      round
      size="xs"
      icon="help_outline"
      color="amber"
      @click.stop.prevent="open = true"
      @mousedown.stop
    >
      <q-tooltip class="bg-dark text-white text-caption">{{ tooltipText }}</q-tooltip>
    </q-btn>

    <q-dialog v-model="open">
      <q-card class="bg-secondary border-rounded border-white" style="min-width: 520px; max-width: 720px;" dark>
        <q-card-section class="row items-center bg-dark text-white">
          <q-icon name="help_outline" color="amber" class="q-mr-sm" />
          <div class="text-h6">{{ localized.title }}</div>
          <q-space />
          <q-btn v-if="policyType" :label="policyType" flat dense no-caps size="sm" color="primary" class="q-mr-sm" disable />
          <q-btn dense color="white" round icon="close" text-color="dark" @click="open = false" />
        </q-card-section>

        <q-separator color="white" />

        <q-card-section class="text-white">
          <div v-if="localized.note" class="q-mb-md q-pa-sm border-rounded" style="background: rgba(255, 152, 0, 0.15); border-left: 3px solid #ffb74d;">
            <div class="text-caption text-amber q-mb-xs">
              <q-icon name="warning" size="xs" class="q-mr-xs" />{{ $t('hyperparamNoteHeading') }}
            </div>
            <div style="white-space: pre-wrap; line-height: 1.5;">{{ localized.note }}</div>
          </div>

          <div class="text-overline text-amber q-mb-sm">
            <q-icon name="lightbulb" size="xs" class="q-mr-xs" />{{ $t('hyperparamEasyHeading') }}
          </div>
          <p
            v-for="(para, idx) in easyParagraphs"
            :key="'e' + idx"
            class="q-mb-md"
            style="white-space: pre-wrap; line-height: 1.65;"
          >{{ para }}</p>

          <q-separator dark class="q-my-md" />

          <div class="text-overline text-primary q-mb-sm">
            <q-icon name="science" size="xs" class="q-mr-xs" />{{ $t('hyperparamExpertHeading') }}
          </div>
          <p
            v-for="(para, idx) in expertParagraphs"
            :key="'x' + idx"
            class="q-mb-md text-grey-3"
            style="white-space: pre-wrap; line-height: 1.65;"
          >{{ para }}</p>
        </q-card-section>

        <q-separator color="white" />

        <q-card-actions align="right" class="bg-dark">
          <q-btn flat :label="$t('close')" color="white" @click="open = false" />
        </q-card-actions>
      </q-card>
    </q-dialog>
  </span>
</template>

<script setup>
import { ref, computed } from 'vue'
import { useI18n } from 'vue-i18n'
import { getHyperparamHelp } from 'src/configs/hyperparamHelp'

const props = defineProps({
  policyType: { type: String, default: null },
  paramKey: { type: String, required: true },
})

const { locale, t } = useI18n()
const open = ref(false)

const help = computed(() => getHyperparamHelp(props.policyType, props.paramKey))

const localized = computed(() => {
  if (!help.value) return { title: '', easy: '', expert: '', note: '' }
  const lang = locale.value && locale.value.startsWith('ko') ? 'ko' : 'en'
  return help.value[lang] || help.value.en || help.value.ko
})

const easyParagraphs = computed(() => (localized.value.easy || '').split(/\n\s*\n/))
const expertParagraphs = computed(() => (localized.value.expert || '').split(/\n\s*\n/))

const tooltipText = computed(() => t('hyperparamHelpTooltip'))
</script>
