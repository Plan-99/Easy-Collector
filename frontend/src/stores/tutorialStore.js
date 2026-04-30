import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { api } from 'boot/axios'

/**
 * Tutorial mode store.
 *
 * Owns the global "tutorial mode is on" state and proxies start/stop calls
 * to the backend (`/api/tutorial:start`, `/api/tutorial:stop`). Tutorial mode
 * boots a bundled MuJoCo world that publishes the topics consumed by the
 * `tutorial_arm` (custom robot) and `tutorial_camera` (custom sensor) DB rows.
 */
export const useTutorialStore = defineStore('tutorial', () => {
  const running = ref(false)         // sim launch is alive
  const hasTopics = ref(false)       // sim's ROS topics are visible
  const robotId = ref(null)          // seeded tutorial_arm DB id
  const sensorId = ref(null)         // seeded tutorial_camera DB id
  const busy = ref(false)            // start/stop in flight
  const lastError = ref(null)

  let pollHandle = null

  const isActive = computed(() => running.value)

  async function refresh() {
    try {
      const { data } = await api.get('/tutorial/status')
      running.value = !!data.running
      hasTopics.value = !!data.has_topics
      robotId.value = data.robot_id ?? null
      sensorId.value = data.sensor_id ?? null
    } catch (e) {
      // Backend may be transiently unavailable; don't clobber state.
      console.warn('[tutorial] status refresh failed', e)
    }
  }

  function _startPolling() {
    if (pollHandle) return
    pollHandle = setInterval(refresh, 3000)
  }

  function _stopPolling() {
    if (pollHandle) {
      clearInterval(pollHandle)
      pollHandle = null
    }
  }

  async function start() {
    if (busy.value) return
    busy.value = true
    lastError.value = null
    try {
      const { data } = await api.post('/tutorial:start')
      running.value = true
      robotId.value = data.robot_id ?? null
      sensorId.value = data.sensor_id ?? null
      _startPolling()
      return data
    } catch (e) {
      lastError.value = e?.response?.data?.message || e.message || String(e)
      throw e
    } finally {
      busy.value = false
    }
  }

  async function stop() {
    if (busy.value) return
    busy.value = true
    lastError.value = null
    try {
      await api.post('/tutorial:stop')
      running.value = false
      hasTopics.value = false
      _stopPolling()
    } catch (e) {
      lastError.value = e?.response?.data?.message || e.message || String(e)
      throw e
    } finally {
      busy.value = false
    }
  }

  async function toggle() {
    if (running.value) {
      await stop()
    } else {
      await start()
    }
  }

  // Initial sync on first import; cheap and safe.
  refresh().then(() => {
    if (running.value) _startPolling()
  })

  return {
    running,
    hasTopics,
    robotId,
    sensorId,
    busy,
    lastError,
    isActive,
    refresh,
    start,
    stop,
    toggle,
  }
})
