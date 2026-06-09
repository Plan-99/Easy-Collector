import { defineStore } from 'pinia'
import { ref } from 'vue'
import { api } from 'boot/axios'

/**
 * Demo simulation store.
 *
 * Backs the top-bar "Sim Activation" button. Lists the demos under the repo
 * `demo/` folder (served by `/api/demo/list`) and launches/stops the selected
 * one via the gRPC bridge (`/api/demo:start` / `/api/demo:stop`) — the same
 * mechanism tutorial mode uses.
 */
export const useDemoStore = defineStore('demo', () => {
  const demos = ref([])            // [{id, name, name_en, description, ...}]
  const runningDemoId = ref(null)  // id of the currently-running demo sim
  const hasTopics = ref(false)     // sim's ROS topics are visible
  const loading = ref(false)       // list fetch in flight
  const busy = ref(false)          // start/stop in flight
  const lastError = ref(null)

  let pollHandle = null

  async function fetchList() {
    loading.value = true
    try {
      const { data } = await api.get('/demo/list')
      demos.value = data.demos || []
    } catch (e) {
      lastError.value = e?.response?.data?.message || e.message || String(e)
    } finally {
      loading.value = false
    }
  }

  async function refreshStatus() {
    try {
      const { data } = await api.get('/demo/status')
      runningDemoId.value = data.running ? data.demo_id : null
      hasTopics.value = !!data.has_topics
    } catch (e) {
      console.warn('[demo] status refresh failed', e)
    }
  }

  function _startPolling() {
    if (pollHandle) return
    pollHandle = setInterval(refreshStatus, 3000)
  }

  function _stopPolling() {
    if (pollHandle) {
      clearInterval(pollHandle)
      pollHandle = null
    }
  }

  async function start(demoId) {
    if (busy.value) return
    busy.value = true
    lastError.value = null
    try {
      const { data } = await api.post('/demo:start', { demo_id: demoId })
      runningDemoId.value = data.demo_id
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
      await api.post('/demo:stop', {})
      runningDemoId.value = null
      hasTopics.value = false
      _stopPolling()
    } catch (e) {
      lastError.value = e?.response?.data?.message || e.message || String(e)
      throw e
    } finally {
      busy.value = false
    }
  }

  return {
    demos,
    runningDemoId,
    hasTopics,
    loading,
    busy,
    lastError,
    fetchList,
    refreshStatus,
    start,
    stop,
  }
})
