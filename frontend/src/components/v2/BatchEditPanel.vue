<template>
  <!-- Root is `col-auto` (flex: 0 0 auto) — the parent right-pane wrapper in
       DatasetPage is a flex column with overflow:auto. Without `col-auto`,
       this panel is shrunk by the flex algorithm whenever its intrinsic
       content height exceeds the viewport, which in turn squashes the
       crop-stage divs (overflow:hidden) and chops the bottom off the
       camera preview images. With `col-auto` the panel keeps its natural
       size and the wrapper scrolls instead. (Narrow screens worked because
       col-sm-12 stacks the crop columns, lowering total intrinsic height
       below the viewport — no shrink pressure.)
       Also no `column` flex here: children stack as normal blocks so per-
       card heights are intrinsic, not flex-shrunk. -->
  <div v-if="dataset" class="col-auto text-white">
    <!-- Header — mirrors EpisodePanel style for visual continuity. -->
    <div class="row items-center q-mb-md q-pa-xs">
      <q-icon name="dataset" size="22px" color="primary" class="q-mr-sm" />
      <div class="column col-shrink ellipsis-2-lines">
        <div class="text-subtitle1 text-weight-medium ellipsis">{{ dataset.name }}</div>
        <div class="text-caption text-grey-5">
          {{ episodeCount }} {{ $t('hdf5Episodes') }}
        </div>
      </div>
      <q-space />
      <q-btn flat round dense icon="close" color="grey-5" @click="$emit('close')">
        <q-tooltip>{{ $t('close') }}</q-tooltip>
      </q-btn>
    </div>

    <!-- Output dataset name -->
    <q-input
      v-model="form.outputName"
      outlined dense dark hide-bottom-space
      color="primary" bg-color="dark"
      input-class="text-white"
      label-color="grey-5"
      :label="$t('datasetName')"
      class="q-mb-md"
    />

    <!-- Video preview — reuse EpisodeViewer; live preview reflects current
         augmentation snapshot via :read_dataset_add_config below. -->
    <q-card v-if="firstEpisodeName" flat bordered class="bg-secondary q-mb-md">
      <q-card-section>
        <episode-viewer
          :key="previewPath"
          :path="previewPath"
          :total-frames="previewTotalFrames"
          :task-id="taskId"
        />
      </q-card-section>
    </q-card>

    <!-- ── Augmentation ──────────────────────────────────────────────── -->
    <q-card flat bordered class="bg-secondary q-mb-md">
      <q-card-section class="row items-center q-gutter-sm q-py-sm">
        <q-icon name="auto_fix_high" size="16px"
          :color="form.augmentation.enabled ? 'primary' : 'grey-6'" />
        <span class="text-caption text-weight-medium">{{ $t('augmentDatasetTitle') }}</span>
        <q-toggle
          v-model="form.augmentation.enabled"
          color="primary" dense size="sm"
        />
        <q-space />
        <q-badge v-if="form.augmentation.enabled" color="primary" text-color="white">
          ×{{ form.augmentation.repeat }}
        </q-badge>
      </q-card-section>
      <q-separator dark />

      <q-card-section :class="{ disabled: !form.augmentation.enabled }">
        <!-- Repeat + sensor selector -->
        <div class="row items-center q-gutter-md q-mb-md">
          <q-input
            v-model.number="form.augmentation.repeat"
            type="number" :min="1" :max="20"
            dense outlined dark hide-bottom-space bg-color="dark"
            input-class="text-white text-center"
            label-color="grey-5"
            style="width: 92px;"
            :label="$t('augRepeat')"
            :disable="!form.augmentation.enabled"
          />
          <q-select
            v-model="editingSensor"
            :options="sensorOptions"
            outlined dense dark
            color="primary" bg-color="dark"
            label-color="grey-5"
            map-options emit-value
            :label="$t('augEditingSensor')"
            input-class="text-white"
            style="min-width: 140px;"
            :disable="!form.augmentation.enabled || !sensors.length"
          />
          <q-btn
            flat dense no-caps size="sm" color="primary"
            icon="content_copy"
            :label="$t('augCopyToAll')"
            :disable="!form.augmentation.enabled || sensors.length < 2"
            @click="copyAugmentationToAll"
          >
            <q-tooltip>{{ $t('augCopyToAllTip') }}</q-tooltip>
          </q-btn>
          <q-space />
          <span class="text-caption text-grey-6">{{ $t('augRangeHint') }}</span>
        </div>

        <template v-if="currentAug">
          <!-- Lightness -->
          <div class="text-caption text-grey-5 row items-center">
            <span>{{ $t('augLightness') }}</span>
            <q-space />
            <span class="text-grey-6">[{{ currentAug.lightness[0] }}, {{ currentAug.lightness[1] }}]</span>
          </div>
          <q-range
            v-model="lightnessRange"
            :min="-100" :max="100" :step="1"
            label label-always color="primary"
            :disable="!form.augmentation.enabled"
          />

          <!-- HSV -->
          <div class="row items-center q-gutter-md q-mt-md q-mb-xs">
            <div class="text-caption text-grey-5">HSV</div>
            <q-checkbox
              v-model="currentAug.hsv.random"
              :label="$t('augHsvRandom')"
              size="xs" dense dark color="primary"
              :disable="!form.augmentation.enabled"
            />
          </div>
          <div class="row q-col-gutter-lg">
            <div class="col">
              <div class="text-caption text-grey-6">{{ $t('augHsvHue') }}</div>
              <q-range v-model="hsvHRange"
                :min="0" :max="1" :step="0.01"
                label label-always color="primary"
                :disable="!form.augmentation.enabled || currentAug.hsv.random" />
            </div>
            <div class="col">
              <div class="text-caption text-grey-6">{{ $t('augHsvSaturation') }}</div>
              <q-range v-model="hsvSRange"
                :min="0" :max="1" :step="0.01"
                label label-always color="primary"
                :disable="!form.augmentation.enabled || currentAug.hsv.random" />
            </div>
            <div class="col">
              <div class="text-caption text-grey-6">{{ $t('augHsvValue') }}</div>
              <q-range v-model="hsvVRange"
                :min="0" :max="1" :step="0.01"
                label label-always color="primary"
                :disable="!form.augmentation.enabled || currentAug.hsv.random" />
            </div>
          </div>

          <!-- Disturbances + Salt&Pepper -->
          <div class="row q-col-gutter-md q-mt-md">
            <div class="col-md-6 col-sm-12">
              <div class="text-caption text-grey-5">{{ $t('augDisturbances') }}</div>
              <q-range v-model="rectCountRange"
                :min="0" :max="20" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
              <div class="row q-gutter-sm items-center q-mt-xs">
                <q-checkbox
                  v-model="currentAug.rectangles.randomColor"
                  :label="$t('augDisturbancesRandomColor')"
                  size="xs" dense dark color="primary"
                  :disable="!form.augmentation.enabled"
                />
                <q-space />
                <!-- Preset palette — quick swatches.  Custom picker behind icon. -->
                <div class="row items-center q-gutter-xs">
                  <div
                    v-for="c in colorPalette"
                    :key="c"
                    :style="{
                      width: '18px',
                      height: '18px',
                      background: c,
                      borderRadius: '3px',
                      cursor: 'pointer',
                      border: c.toLowerCase() === (currentAug.rectangles.color || '').toLowerCase()
                        ? '2px solid #1976d2'
                        : '1px solid rgba(255,255,255,0.2)',
                    }"
                    @click="setRectColor(c)"
                  ></div>
                  <q-btn flat dense size="sm" icon="colorize" round
                    :disable="!form.augmentation.enabled || currentAug.rectangles.randomColor">
                    <q-popup-proxy cover>
                      <q-color v-model="currentAug.rectangles.color" dark />
                    </q-popup-proxy>
                    <q-tooltip>{{ $t('augColorCustom') }}</q-tooltip>
                  </q-btn>
                </div>
              </div>
            </div>
            <div class="col-md-6 col-sm-12">
              <div class="text-caption text-grey-5">{{ $t('augSaltPepper') }}</div>
              <q-range v-model="saltRange"
                :min="0" :max="1" :step="0.01"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
          </div>

          <!-- Gaussian noise (mean / sigma) -->
          <div class="row q-col-gutter-md q-mt-md">
            <div class="col">
              <div class="text-caption text-grey-5">{{ $t('augGaussianMean') }}</div>
              <q-range v-model="gaussianMeanRange"
                :min="-255" :max="255" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
            <div class="col">
              <div class="text-caption text-grey-5">{{ $t('augGaussianSigma') }}</div>
              <q-range v-model="gaussianSigmaRange"
                :min="0" :max="50" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
          </div>

          <!-- Perspective -->
          <div class="text-caption text-grey-5 q-mt-md">{{ $t('augProspective') }}</div>
          <div class="row q-col-gutter-md">
            <div class="col-md-3 col-sm-6 col-xs-12">
              <div class="text-caption text-grey-6">{{ $t('augProspectiveScaleFactor') }}</div>
              <q-range v-model="perspScaleRange"
                :min="0" :max="50" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
            <div class="col-md-3 col-sm-6 col-xs-12">
              <div class="text-caption text-grey-6">{{ $t('augProspectiveDegrees') }}</div>
              <q-range v-model="perspDegRange"
                :min="-180" :max="180" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
            <div class="col-md-3 col-sm-6 col-xs-12">
              <div class="text-caption text-grey-6">{{ $t('augProspectiveShear') }}</div>
              <q-range v-model="perspShearRange"
                :min="-45" :max="45" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
            <div class="col-md-3 col-sm-6 col-xs-12">
              <div class="text-caption text-grey-6">{{ $t('augProspectivePerspective') }}</div>
              <q-range v-model="perspPerspRange"
                :min="-100" :max="100" :step="1"
                label label-always color="primary"
                :disable="!form.augmentation.enabled" />
            </div>
          </div>
        </template>
      </q-card-section>
    </q-card>

    <!-- ── Downsample ────────────────────────────────────────────────── -->
    <q-card flat bordered class="bg-secondary q-mb-md">
      <q-card-section class="row items-center q-gutter-sm q-py-sm">
        <q-icon name="compress" size="16px"
          :color="form.downsample.enabled ? 'primary' : 'grey-6'" />
        <span class="text-caption text-weight-medium">{{ $t('datasetDownsampleTitle') }}</span>
        <q-toggle
          v-model="form.downsample.enabled"
          color="primary" dense size="sm"
        />
        <q-space />
        <q-badge v-if="form.downsample.enabled" color="primary" text-color="white">
          {{ form.downsample.keep }} / {{ form.downsample.every }}
        </q-badge>
      </q-card-section>
      <q-separator dark />
      <q-card-section :class="{ disabled: !form.downsample.enabled }">
        <div class="row q-gutter-md items-center">
          <q-input
            v-model.number="form.downsample.keep"
            type="number" :min="1"
            :label="$t('datasetDownsampleKeep')"
            dense outlined dark hide-bottom-space bg-color="dark"
            input-class="text-white text-center"
            label-color="grey-5"
            style="width: 110px;"
            :disable="!form.downsample.enabled"
          />
          <div class="text-caption text-grey-5">/</div>
          <q-input
            v-model.number="form.downsample.every"
            type="number" :min="2"
            :label="$t('datasetDownsampleEvery')"
            dense outlined dark hide-bottom-space bg-color="dark"
            input-class="text-white text-center"
            label-color="grey-5"
            style="width: 110px;"
            :disable="!form.downsample.enabled"
          />
          <q-space />
          <div class="text-caption text-grey-6 ellipsis">
            {{ $t('datasetDownsampleHint') }}
          </div>
        </div>
      </q-card-section>
    </q-card>

    <!-- ── Crop ──────────────────────────────────────────────────────── -->
    <q-card flat bordered class="bg-secondary q-mb-md">
      <q-card-section class="row items-center q-gutter-sm q-py-sm">
        <q-icon name="crop" size="16px"
          :color="form.crop.enabled ? 'primary' : 'grey-6'" />
        <span class="text-caption text-weight-medium">{{ $t('augCrop') }}</span>
        <q-toggle
          v-model="form.crop.enabled"
          color="primary" dense size="sm"
        />
        <q-space />
        <q-badge v-if="form.crop.enabled && cropCount" color="primary" text-color="white">
          {{ cropCount }} {{ cropCount > 1 ? 'sensors' : 'sensor' }}
        </q-badge>
      </q-card-section>
      <q-separator dark />
      <q-card-section :class="{ disabled: !form.crop.enabled }">
        <div v-if="loadingPreview" class="text-center text-grey-5 q-pa-md">
          <q-spinner color="primary" /> Loading preview…
        </div>
        <div v-else-if="!sensors.length" class="text-grey-5">
          {{ $t('augNoSensors') }}
        </div>

        <div v-else class="row q-col-gutter-md">
          <!-- per-sensor wrapper: block layout (no `column` flex class) — using
               flex column here makes the inner `width:100% height:auto` image
               get its height computed BEFORE the width is known, which causes
               the bottom of the preview to clip. Normal block flow stacks
               header / crop-stage / WxH / inputs without that hazard. -->
          <div
            v-for="sensor in sensors"
            :key="`crop-${sensor.name}`"
            class="col-md-6 col-sm-12"
          >
            <div class="row items-center q-gutter-sm q-mb-xs">
              <q-icon name="videocam" size="14px" color="grey-5" />
              <div class="text-caption text-grey-4">{{ sensor.name }}</div>
              <div class="text-caption text-grey-6" v-if="sensor.width">
                {{ sensor.width }}×{{ sensor.height }}
              </div>
              <q-space />
              <q-btn
                flat dense size="xs" color="grey-5" icon="restart_alt"
                :label="$t('reset')"
                @click="resetCrop(sensor.name)"
                :disable="!form.crop.enabled"
              />
            </div>
            <!-- Drag container: fixed height + flex centering. The image keeps
                 its source aspect ratio (height: 100% of stage, width: auto)
                 so it can't change height with viewport width — that means no
                 flex-shrink cascade from "too-tall content" elsewhere on the
                 panel. The unused column width on either side shows the
                 container's bg-black background. -->
            <div
              :ref="(el) => (containerRefs[sensor.name] = el)"
              class="bg-black"
              :style="{
                position: 'relative',
                overflow: 'hidden',
                height: '240px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                cursor: form.crop.enabled ? 'crosshair' : 'not-allowed',
                userSelect: 'none',
              }"
              @mousedown.prevent="onCropStart($event, sensor.name)"
              @mousemove="onCropMove($event, sensor.name)"
              @mouseup="onCropEnd(sensor.name)"
              @mouseleave="onCropEnd(sensor.name)"
            >
              <img
                :ref="(el) => (imgRefs[sensor.name] = el)"
                :src="sensor.imgUrl"
                :alt="sensor.name"
                :style="{ display: 'block', height: '100%', width: 'auto', pointerEvents: 'none' }"
              />
              <div
                v-if="cropBoxes[sensor.name] && cropBoxes[sensor.name].dragStyle.value"
                :style="{
                  ...cropBoxes[sensor.name].dragStyle.value,
                  pointerEvents: 'none',
                  border: '2px dashed #1976d2',
                  background: 'rgba(25,118,210,0.15)',
                }"
              />
              <div
                v-if="cropBoxes[sensor.name] && cropBoxes[sensor.name].persistedStyle.value"
                :style="{
                  ...cropBoxes[sensor.name].persistedStyle.value,
                  pointerEvents: 'none',
                  border: '2px solid #1976d2',
                  background: 'rgba(25,118,210,0.25)',
                }"
              />
            </div>
            <!-- Cropped area summary + 4 inputs (x1/y1/x2/y2) mirroring
                 Workspace's setting tab UX: WxH on the right, single-row
                 inputs growing to fill width. -->
            <div class="row justify-between items-center q-mt-xs q-px-xs">
              <div class="text-caption text-grey-5">{{ $t('workspaceCroppedAreaLabel') }}</div>
              <div class="text-caption text-grey-4">{{ cropDimensions(sensor.name) }}</div>
            </div>
            <div class="row q-gutter-x-sm q-mt-xs">
              <q-input
                v-for="(label, index) in cropLabels"
                :key="index"
                :model-value="getCropCoord(sensor.name, index)"
                @update:model-value="(v) => setCropCoord(sensor.name, index, v, sensor)"
                type="number" :min="0"
                :max="index % 2 === 0 ? (sensor.width || 0) : (sensor.height || 0)"
                dense outlined dark hide-bottom-space
                color="primary" bg-color="dark"
                input-class="text-white text-center"
                label-color="grey-5"
                :placeholder="label"
                class="col"
                :disable="!form.crop.enabled"
              />
            </div>
          </div>
        </div>
      </q-card-section>
    </q-card>

    <!-- ── Rotate ────────────────────────────────────────────────────── -->
    <q-card flat bordered class="bg-secondary q-mb-md">
      <q-card-section class="row items-center q-gutter-sm q-py-sm">
        <q-icon name="rotate_right" size="16px"
          :color="form.rotate.enabled ? 'primary' : 'grey-6'" />
        <span class="text-caption text-weight-medium">{{ $t('augRotate') }}</span>
        <q-toggle
          v-model="form.rotate.enabled"
          color="primary" dense size="sm"
        />
        <q-space />
        <q-badge v-if="form.rotate.enabled && rotateSummary" color="primary" text-color="white">
          {{ rotateSummary }}
        </q-badge>
      </q-card-section>
      <q-separator dark />
      <q-card-section :class="{ disabled: !form.rotate.enabled }">
        <div v-if="!sensors.length" class="text-grey-5">
          {{ $t('augNoSensors') }}
        </div>
        <div
          v-for="(sensor, idx) in sensors"
          :key="`rot-${sensor.name}`"
          class="row items-center q-gutter-md"
          :class="{ 'q-mt-sm': idx > 0 }"
        >
          <q-icon name="videocam" size="14px" color="grey-5" />
          <div class="text-caption text-grey-4" style="min-width: 110px;">{{ sensor.name }}</div>
          <q-btn-toggle
            v-model="form.rotate.angles[sensor.name]"
            :options="rotateOptions"
            toggle-color="primary"
            color="grey-9"
            text-color="grey-3"
            unelevated dense no-caps
            :disable="!form.rotate.enabled"
          />
        </div>
      </q-card-section>
    </q-card>

    <!-- Apply bar — wrapped in a q-card so the panel terminates with the
         same visual shape as the other sections (mirrors EpisodePanel's
         pattern of ending in a styled section). Without this wrapper the
         transparent row left an "orphan" strip between the last section
         card and the wrapper's bottom padding. -->
    <q-card flat bordered class="bg-secondary q-mb-md">
      <q-card-section class="row items-center q-py-sm">
        <div v-if="processing" class="col q-mr-md">
          <q-linear-progress
            :value="progress"
            color="primary" track-color="grey-9"
            size="22px" instant-feedback
          >
            <div class="absolute-full flex flex-center">
              <q-badge color="white" text-color="dark"
                :label="`${Math.round(progress * 100)}%`" />
            </div>
          </q-linear-progress>
        </div>
        <q-space />
        <q-btn
          :label="$t('apply')"
          color="primary" unelevated no-caps
          :disable="!anyEnabled || !form.outputName.trim() || processing"
          @click="apply"
        />
      </q-card-section>
    </q-card>
  </div>
</template>

<script setup>
import { ref, computed, reactive, shallowReactive, watch, onMounted, onUnmounted, defineProps, defineEmits } from 'vue'
import { api } from 'src/boot/axios'
import { Notify } from 'quasar'
import { useI18n } from 'vue-i18n'
import { useSocket } from 'src/composables/useSocket'
import { useCropBox } from 'src/utils/cropBox'
import EpisodeViewer from 'src/components/v2/EpisodeViewer.vue'

const { t } = useI18n()
const { socket } = useSocket()

const props = defineProps({
  taskId: { type: [Number, String], default: null },
  dataset: { type: Object, default: null },
})
const emit = defineEmits(['close', 'completed'])

// ─── Defaults ─────────────────────────────────────────────────────────
// Per-sensor augmentation defaults — every numeric field is a [min, max]
// range. (Single value = same min/max, no variance.)
const _defaultSensorAug = () => ({
  lightness: [0, 0],
  hsv: { h: [0, 0], s: [0, 0], v: [0, 0], random: false },
  rectangles: { count: [0, 0], color: '#000000', randomColor: false },
  saltAndPepper: { amount: [0, 0] },
  gaussian: { mean: [0, 0], sigma: [0, 0] },
  prospective: {
    scale_factor: [0, 0], degrees: [0, 0], shear: [0, 0], perspective: [0, 0],
  },
})

const _defaultForm = () => ({
  outputName: '',
  augmentation: {
    enabled: false,
    repeat: 1,
    sensors: {},   // populated when sensors load
  },
  downsample: { enabled: false, keep: 1, every: 2 },
  crop:   { enabled: false, regions: {} },
  rotate: { enabled: false, angles:  {} },
})
const form = reactive(_defaultForm())

const rotateOptions = [
  { label: '0°',   value: 0   },
  { label: '90°',  value: 90  },
  { label: '180°', value: 180 },
  { label: '270°', value: 270 },
]

// Small fixed palette for the disturbance color picker. Non-continuous on
// purpose — covers the common "blocker" / "annotation" use cases plus a
// neutral grey. Custom HEX still reachable via the eyedropper popup.
const colorPalette = [
  '#000000', '#ffffff', '#808080',
  '#e53935', '#fb8c00', '#fdd835',
  '#43a047', '#1e88e5', '#8e24aa',
]

// ─── EpisodeViewer preview (top) ──────────────────────────────────────
const previewTotalFrames = ref(0)

const episodeCount = computed(() => {
  const eps = props.dataset?.episodes || props.dataset?.files || []
  return Array.isArray(eps) ? eps.length : 0
})

const firstEpisodeName = computed(() => {
  const eps = props.dataset?.episodes || props.dataset?.files || []
  if (!eps.length) return null
  const first = eps[0]
  return typeof first === 'string' ? first : first.name
})

const previewPath = computed(() => {
  if (!props.dataset || !firstEpisodeName.value) return ''
  return `${props.dataset.id}/${firstEpisodeName.value}`
})

async function loadPreviewFrameCount() {
  if (!props.dataset || !firstEpisodeName.value) {
    previewTotalFrames.value = 0
    return
  }
  try {
    const res = await api.get(
      `/dataset/${props.dataset.id}/${firstEpisodeName.value}/:get_data`,
    )
    previewTotalFrames.value = res.data?.episode?.num_frames || 0
  } catch (e) {
    console.error('[BatchEditPanel] preview frame count failed:', e)
    previewTotalFrames.value = 0
  }
}

// ─── Crop preview frames + sensor list ────────────────────────────────
const sensors = ref([])
const containerRefs = reactive({})
const imgRefs = reactive({})
// shallowReactive so the computed refs returned by useCropBox are not
// auto-unwrapped at property access — keeps `.value` semantics intact.
const cropBoxes = shallowReactive({})
const loadingPreview = ref(false)

// Currently edited sensor in the Augmentation section. Defaults to the
// first sensor once they load.
const editingSensor = ref(null)

const sensorOptions = computed(() =>
  sensors.value.map((s) => ({ label: s.name, value: s.name })),
)
const currentAug = computed(() => {
  const name = editingSensor.value
  return name ? form.augmentation.sensors[name] : null
})

async function loadCropPreview() {
  if (!props.dataset || !firstEpisodeName.value) return
  loadingPreview.value = true
  try {
    const res = await api.get(
      `/dataset/${props.dataset.id}/${firstEpisodeName.value}/:frame`,
      { params: { index: 0 } },
    )
    const images = res.data?.images || {}
    const out = []
    for (const [name, url] of Object.entries(images)) {
      out.push({ name, imgUrl: url, width: 0, height: 0 })
    }
    sensors.value = out
    // Reset per-sensor maps
    for (const k of Object.keys(cropBoxes)) delete cropBoxes[k]
    for (const k of Object.keys(containerRefs)) delete containerRefs[k]
    for (const k of Object.keys(imgRefs)) delete imgRefs[k]
    form.crop.regions = {}
    form.rotate.angles = {}
    form.augmentation.sensors = {}
    for (const s of sensors.value) {
      form.rotate.angles[s.name] = 0
      form.augmentation.sensors[s.name] = _defaultSensorAug()
      cropBoxes[s.name] = useCropBox(() => {
        const sensor = sensors.value.find((x) => x.name === s.name) || {}
        return {
          resolution: [sensor.width || 0, sensor.height || 0],
          containerEl: containerRefs[s.name],
          innerEl: () => imgRefs[s.name],
        }
      })
    }
    editingSensor.value = sensors.value[0]?.name || null
  } catch (e) {
    console.error('[BatchEditPanel] crop preview load failed:', e)
    Notify.create({ color: 'negative', message: t('augPreviewFailed') })
  } finally {
    loadingPreview.value = false
  }
}

// After preview frames load into the DOM, snapshot each <img>'s natural
// resolution so cropBox can convert drag coords → source pixels.
watch(sensors, async () => {
  await new Promise((r) => requestAnimationFrame(r))
  await new Promise((r) => requestAnimationFrame(r))
  for (const s of sensors.value) {
    const el = imgRefs[s.name]
    if (!el) continue
    const captureSize = () => {
      const sensor = sensors.value.find((x) => x.name === s.name)
      if (sensor && el.naturalWidth && el.naturalHeight) {
        sensor.width = el.naturalWidth
        sensor.height = el.naturalHeight
      }
    }
    if (el.complete && el.naturalWidth) captureSize()
    else el.addEventListener('load', captureSize, { once: true })
  }
})

// ─── Per-field range proxies (computed) ───────────────────────────────
// q-range needs a {min, max} value. We expose proxies that translate
// [a, b] arrays in the form ↔ {min: a, max: b} for q-range.
function _arrToRange(arr, fallback = [0, 0]) {
  const a = Array.isArray(arr) && arr.length === 2 ? arr : fallback
  return { min: a[0], max: a[1] }
}
function _setArr(field, key, val) {
  if (!currentAug.value) return
  const obj = key ? currentAug.value[field] : currentAug.value
  if (!obj) return
  const target = key ? obj[key] : obj[field]
  // ensure we mutate the existing array (keeps reactivity simple)
  if (Array.isArray(target)) {
    target.splice(0, 2, val.min, val.max)
  }
}

const lightnessRange = computed({
  get: () => _arrToRange(currentAug.value?.lightness),
  set: (v) => currentAug.value && currentAug.value.lightness.splice(0, 2, v.min, v.max),
})
const hsvHRange = computed({
  get: () => _arrToRange(currentAug.value?.hsv.h),
  set: (v) => _setArr('hsv', 'h', v),
})
const hsvSRange = computed({
  get: () => _arrToRange(currentAug.value?.hsv.s),
  set: (v) => _setArr('hsv', 's', v),
})
const hsvVRange = computed({
  get: () => _arrToRange(currentAug.value?.hsv.v),
  set: (v) => _setArr('hsv', 'v', v),
})
const rectCountRange = computed({
  get: () => _arrToRange(currentAug.value?.rectangles.count),
  set: (v) => _setArr('rectangles', 'count', v),
})
const saltRange = computed({
  get: () => _arrToRange(currentAug.value?.saltAndPepper.amount),
  set: (v) => _setArr('saltAndPepper', 'amount', v),
})
const gaussianMeanRange = computed({
  get: () => _arrToRange(currentAug.value?.gaussian.mean),
  set: (v) => _setArr('gaussian', 'mean', v),
})
const gaussianSigmaRange = computed({
  get: () => _arrToRange(currentAug.value?.gaussian.sigma),
  set: (v) => _setArr('gaussian', 'sigma', v),
})
const perspScaleRange = computed({
  get: () => _arrToRange(currentAug.value?.prospective.scale_factor),
  set: (v) => _setArr('prospective', 'scale_factor', v),
})
const perspDegRange = computed({
  get: () => _arrToRange(currentAug.value?.prospective.degrees),
  set: (v) => _setArr('prospective', 'degrees', v),
})
const perspShearRange = computed({
  get: () => _arrToRange(currentAug.value?.prospective.shear),
  set: (v) => _setArr('prospective', 'shear', v),
})
const perspPerspRange = computed({
  get: () => _arrToRange(currentAug.value?.prospective.perspective),
  set: (v) => _setArr('prospective', 'perspective', v),
})

function setRectColor(c) {
  if (!currentAug.value || !form.augmentation.enabled) return
  if (currentAug.value.rectangles.randomColor) return
  currentAug.value.rectangles.color = c
}

function copyAugmentationToAll() {
  if (!currentAug.value) return
  const src = JSON.parse(JSON.stringify(currentAug.value))
  for (const s of sensors.value) {
    if (s.name !== editingSensor.value) {
      form.augmentation.sensors[s.name] = JSON.parse(JSON.stringify(src))
    }
  }
  Notify.create({ color: 'positive', message: t('augCopiedToAll') })
}

// ─── Crop drag handlers + numeric input ──────────────────────────────
function onCropStart(e, sensorName) {
  if (!form.crop.enabled) return
  cropBoxes[sensorName]?.onMouseDown(e)
}
function onCropMove(e, sensorName) {
  if (!form.crop.enabled) return
  cropBoxes[sensorName]?.onMouseMove(e)
}
function onCropEnd(sensorName) {
  if (!form.crop.enabled) return
  cropBoxes[sensorName]?.onMouseUp()
  const b = cropBoxes[sensorName]?.box?.value
  if (b) form.crop.regions[sensorName] = [...b]
}
function resetCrop(sensorName) {
  cropBoxes[sensorName]?.reset()
  delete form.crop.regions[sensorName]
}
const cropLabels = ['x1', 'y1', 'x2', 'y2']

function getCropCoord(sensorName, idx) {
  return form.crop.regions[sensorName]?.[idx] ?? 0
}
function setCropCoord(sensorName, idx, value, sensor) {
  const v = Math.max(0, Math.round(Number(value) || 0))
  const w = sensor?.width || Infinity
  const h = sensor?.height || Infinity
  const clamped = idx % 2 === 0 ? Math.min(v, w) : Math.min(v, h)

  const existing = form.crop.regions[sensorName] || [0, 0, sensor?.width || 0, sensor?.height || 0]
  const next = [...existing]
  next[idx] = clamped
  form.crop.regions[sensorName] = next
  // Sync the drag-box visual to the typed coordinates.
  cropBoxes[sensorName]?.setBox(next)
}
function cropDimensions(sensorName) {
  const r = form.crop.regions[sensorName]
  if (!r) return t('workspaceCroppedAreaNotSet')
  return `${r[2] - r[0]} × ${r[3] - r[1]}`
}

// ─── Live preview (read_dataset_add_config) ──────────────────────────
// EpisodeViewer auto-starts a streaming process via read_dataset on the
// backend. That process applies the current global ``config`` to each
// emitted frame. We push form changes to it (debounced) so dragging a
// q-range updates the preview in ~real-time.
let liveConfigTimer = null
function pushLiveConfig() {
  if (!props.dataset || !firstEpisodeName.value) return
  // Send the full transform-shape so the preview matches the final output:
  //   - sensors map for per-sensor augmentation (range-aware on server)
  //   - crop regions per sensor (resizes preview frames in-place)
  //   - rotate angles per sensor
  // Empty / disabled sections send empty so the server clears them.
  const payload = {
    sensors: form.augmentation.enabled
      ? JSON.parse(JSON.stringify(form.augmentation.sensors))
      : {},
    crop: form.crop.enabled
      ? { regions: JSON.parse(JSON.stringify(form.crop.regions)) }
      : { regions: {} },
    rotate: form.rotate.enabled
      ? { angles: JSON.parse(JSON.stringify(form.rotate.angles)) }
      : { angles: {} },
  }
  api.post(
    `/dataset/${props.dataset.id}/${firstEpisodeName.value}/:read_dataset_add_config`,
    payload,
  ).catch((e) => {
    // Quiet — the streaming process may not be alive yet, no need to spam.
    console.debug('[BatchEditPanel] live config push failed:', e?.message || e)
  })
}
function scheduleLiveConfig() {
  if (liveConfigTimer) clearTimeout(liveConfigTimer)
  liveConfigTimer = setTimeout(pushLiveConfig, 150)
}
// Watch all three transform groups deeply — every form mutation re-pushes.
watch(
  () => [form.augmentation, form.crop, form.rotate],
  () => { scheduleLiveConfig() },
  { deep: true },
)

// ─── Apply / submit ───────────────────────────────────────────────────
const processing = ref(false)
const progress = ref(0)

const anyEnabled = computed(() =>
  form.augmentation.enabled || form.downsample.enabled
    || form.crop.enabled || form.rotate.enabled,
)

const cropCount = computed(() => Object.keys(form.crop.regions).length)
const rotateSummary = computed(() => {
  const angles = Object.values(form.rotate.angles).filter((a) => a)
  if (!angles.length) return ''
  return angles.map((a) => `${a}°`).join(' / ')
})

function _buildOperations() {
  const ops = {}
  ops.repeat = form.augmentation.enabled ? Math.max(1, form.augmentation.repeat || 1) : 1
  if (form.augmentation.enabled) {
    ops.augmentation = {
      sensors: JSON.parse(JSON.stringify(form.augmentation.sensors)),
    }
  }
  if (form.downsample.enabled) {
    ops.downsample = { keep: form.downsample.keep, every: form.downsample.every }
  }
  if (form.crop.enabled && Object.keys(form.crop.regions).length) {
    ops.crop = { regions: { ...form.crop.regions } }
  }
  if (form.rotate.enabled) {
    const angles = {}
    for (const [k, v] of Object.entries(form.rotate.angles)) {
      if (v) angles[k] = v
    }
    if (Object.keys(angles).length) ops.rotate = { angles }
  }
  return ops
}

async function apply() {
  if (!props.dataset || processing.value) return
  if (!anyEnabled.value) {
    Notify.create({ color: 'warning', message: t('augErrorNoOp') })
    return
  }
  const outputName = form.outputName.trim()
  if (!outputName) {
    Notify.create({ color: 'warning', message: t('augErrorNoName') })
    return
  }
  processing.value = true
  progress.value = 0
  try {
    await api.post(`/dataset/${props.dataset.id}/transform`, {
      name: outputName,
      task_id: props.taskId,
      operations: _buildOperations(),
    })
    Notify.create({ color: 'positive', message: t('transformStarted') })
  } catch (e) {
    console.error('[BatchEditPanel] transform start failed:', e)
    Notify.create({ color: 'negative', message: t('transformFailed') })
    processing.value = false
  }
}

// ─── Socket events ────────────────────────────────────────────────────
function onProgress(data) {
  if (typeof data?.progress !== 'number') return
  progress.value = Math.max(0, Math.min(1, data.progress))
}
function onComplete(data) {
  processing.value = false
  progress.value = 0
  emit('completed', data)
}

onMounted(() => {
  socket.on('transform_progress', onProgress)
  socket.on('transform_complete', onComplete)
  socket.on('augmentation_progress', onProgress)
})
onUnmounted(() => {
  socket.off('transform_progress', onProgress)
  socket.off('transform_complete', onComplete)
  socket.off('augmentation_progress', onProgress)
  if (liveConfigTimer) clearTimeout(liveConfigTimer)
})

// ─── Re-init on dataset change ────────────────────────────────────────
watch(
  () => props.dataset?.id,
  (newId) => {
    Object.assign(form, _defaultForm())
    form.outputName = props.dataset ? `${props.dataset.name}_transformed` : ''
    sensors.value = []
    previewTotalFrames.value = 0
    editingSensor.value = null
    if (newId) {
      loadPreviewFrameCount()
      loadCropPreview()
    }
  },
  { immediate: true },
)
</script>
