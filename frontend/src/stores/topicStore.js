import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { api } from 'boot/axios'
import { useSocket } from 'src/composables/useSocket'

/**
 * 활성 ROS2 토픽 push 모델 store.
 *
 * 백엔드(TopicWatcher)가 bridge gRPC ListTopics를 1Hz로 폴링하고 변경 시
 * 'topics_changed' 이벤트를 발행한다. 이 store는 그 이벤트를 listen하여
 * `topics` Map을 갱신하므로, 컴포넌트는 `isPublished(topicName)`만 watch하면
 * 토픽 가시성 변화에 자동으로 반응할 수 있다.
 *
 * 새로고침 / 폴링 / startRobot 후 별도 검증 — 더 이상 필요 없음.
 */
export const useTopicStore = defineStore('topic', () => {
  // Map<string, string> — name → type
  const topics = ref(new Map())
  const initialized = ref(false)

  const topicNames = computed(() => Array.from(topics.value.keys()))

  function isPublished(topicName) {
    if (!topicName) return false
    return topics.value.has(topicName)
  }

  function _replaceAll(list) {
    const next = new Map()
    for (const t of list || []) {
      if (t && t.name) next.set(t.name, t.type || '')
    }
    topics.value = next
  }

  function initialize() {
    if (initialized.value) return Promise.resolve()
    const { socket } = useSocket()

    // Push 이벤트
    socket.on('topics_changed', (data) => {
      _replaceAll(data?.topics)
    })

    // 백엔드와 connect 즉시 'topics_changed'가 한 번 오지만, 그 이전에 마운트된
    // 컴포넌트가 있을 수 있으니 한 번은 REST로 즉시 채워둔다.
    initialized.value = true
    return api.get('/topics').then((res) => {
      _replaceAll(res?.data?.topics)
    }).catch(() => {
      // 일시 실패해도 곧 push로 채워질 것이므로 무시
    })
  }

  return {
    topics,
    topicNames,
    isPublished,
    initialize,
  }
})
