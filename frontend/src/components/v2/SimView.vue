<template>
  <div class="sim-view column" style="height: 100%;">
    <!-- Toolbar -->
    <div class="row items-center q-pa-xs q-gutter-xs" style="background: #1a1a2e;">
      <q-btn
        v-if="!simRunning"
        dense flat size="sm" icon="play_arrow" color="green"
        @click="startSim" :loading="starting"
      >
        <q-tooltip>시뮬레이션 시작</q-tooltip>
      </q-btn>
      <q-btn v-else dense flat size="sm" icon="stop" color="red" @click="stopSim">
        <q-tooltip>정지</q-tooltip>
      </q-btn>
      <q-separator vertical />
      <q-btn dense flat icon="rotate_left" size="sm" color="white" @click="orbitYaw(-15)" />
      <q-btn dense flat icon="rotate_right" size="sm" color="white" @click="orbitYaw(15)" />
      <q-btn dense flat icon="arrow_upward" size="sm" color="white" @click="orbitPitch(-10)" />
      <q-btn dense flat icon="arrow_downward" size="sm" color="white" @click="orbitPitch(10)" />
      <q-btn dense flat icon="zoom_in" size="sm" color="white" @click="zoom(-0.2)" />
      <q-btn dense flat icon="zoom_out" size="sm" color="white" @click="zoom(0.2)" />
    </div>

    <!-- Video -->
    <div class="col relative-position" style="background: #000;">
      <video
        ref="videoEl"
        autoplay playsinline muted
        style="width: 100%; height: 100%; object-fit: contain;"
      />
      <div v-if="!simRunning && !starting" class="absolute-center text-grey-6 text-caption">
        시뮬레이션을 시작하세요
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, watch, onMounted, onUnmounted } from 'vue'
import { useSimWebRTC } from 'src/composables/useSimWebRTC'
import axios from 'axios'

const props = defineProps({
  robotId: { type: [Number, String], default: null },
  autoStart: { type: Boolean, default: false },
})

const API = 'http://localhost:5000/api'
const { connect, disconnect } = useSimWebRTC()

const videoEl = ref(null)
const simRunning = ref(false)
const starting = ref(false)

let camYaw = 45
let camPitch = -30
let camDistance = 1.0

async function startSim() {
  if (!props.robotId) return
  starting.value = true
  try {
    await axios.post(`${API}/sim/start`, {
      robot_id: props.robotId,
      cam_config: { yaw: camYaw, pitch: camPitch, distance: camDistance },
    })
    simRunning.value = true

    // WebRTC 연결
    await connect((event) => {
      if (videoEl.value) {
        const stream = new MediaStream()
        stream.addTrack(event.track)
        videoEl.value.srcObject = stream
      }
    }, 30)
  } catch (e) {
    console.error('Sim start error:', e)
  } finally {
    starting.value = false
  }
}

async function stopSim() {
  disconnect()
  await axios.post(`${API}/sim/stop`).catch(() => {})
  simRunning.value = false
  if (videoEl.value) videoEl.value.srcObject = null
}

function orbitYaw(delta) {
  camYaw += delta
  axios.post(`${API}/sim/camera`, { yaw: camYaw }).catch(() => {})
}

function orbitPitch(delta) {
  camPitch = Math.max(-89, Math.min(-5, camPitch + delta))
  axios.post(`${API}/sim/camera`, { pitch: camPitch }).catch(() => {})
}

function zoom(delta) {
  camDistance = Math.max(0.2, camDistance + delta)
  axios.post(`${API}/sim/camera`, { distance: camDistance }).catch(() => {})
}

watch(() => props.robotId, (newId, oldId) => {
  if (newId && newId !== oldId && props.autoStart) {
    if (simRunning.value) {
      stopSim().then(() => startSim())
    } else {
      startSim()
    }
  }
})

onMounted(() => {
  if (props.autoStart && props.robotId) {
    startSim()
  }
})

onUnmounted(() => {
  if (simRunning.value) {
    disconnect()
    axios.post(`${API}/sim/stop`).catch(() => {})
  }
})
</script>
