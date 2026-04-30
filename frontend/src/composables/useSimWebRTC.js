// composables/useSimWebRTC.js
// 시뮬레이션 전용 WebRTC — 센서 로직과 완전히 분리
import { ref, onUnmounted } from 'vue'

const streaming_server = 'http://localhost:5002'

export function useSimWebRTC() {
  const pc = ref(null)
  const streamId = ref(null)
  const connectionState = ref('new')

  const connect = async (onTrack, fps = 30) => {
    disconnect()

    const peerConnection = new RTCPeerConnection({
      iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
    })
    pc.value = peerConnection

    peerConnection.addTransceiver('video', { direction: 'recvonly' })
    peerConnection.ontrack = (event) => onTrack(event)
    peerConnection.onconnectionstatechange = () => {
      connectionState.value = peerConnection.connectionState
    }

    const offer = await peerConnection.createOffer({ offerToReceiveVideo: true })
    await peerConnection.setLocalDescription(offer)

    const response = await fetch(`${streaming_server}/sim/offer`, {
      method: 'POST',
      body: JSON.stringify({ sdp: offer.sdp, type: offer.type, fps }),
      headers: { 'Content-Type': 'application/json' },
    })

    const answer = await response.json()
    if (answer.error) throw new Error(answer.error)

    await peerConnection.setRemoteDescription(new RTCSessionDescription(answer))
    streamId.value = answer.stream_id

    return { pc: peerConnection, stream_id: answer.stream_id }
  }

  const disconnect = () => {
    if (pc.value) {
      pc.value.close()
      pc.value = null
    }
    streamId.value = null
    connectionState.value = 'closed'
  }

  const updateCamera = async (params) => {
    await fetch(`${streaming_server}/sim/camera`, {
      method: 'POST',
      body: JSON.stringify(params),
      headers: { 'Content-Type': 'application/json' },
    })
  }

  onUnmounted(() => disconnect())

  return { connectionState, connect, disconnect, updateCamera }
}
