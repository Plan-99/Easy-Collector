// composables/useWebRTC.js
//
// WebRTC 공유 풀 + per-instance 핸들 (useWebRTC).
//
// 핵심 아이디어:
//   - 같은 (topic, resize, crop, rotate, msg_type) 조합은 모듈 레벨 풀에
//     하나의 RTCPeerConnection 만 유지 (refcount). 여러 컴포넌트가 동시에
//     요청하면 동일한 MediaStreamTrack 을 공유해서 받아 각자의 <video>.srcObject
//     로 꽂는다. 한 토픽을 N 곳에서 보면 PC 1개 + ICE handshake 1번이면 충분.
//   - config 가 다르면 다른 stream — backend 가 crop/resize/rotate 를 인코딩 전에
//     적용하므로 다른 view 는 어차피 다른 영상이다.
//   - addConfig 는 *공유된* PC 의 backend 설정을 바꾼다 → 같은 키를 공유하는
//     모든 viewer 가 동시에 새 view 로 전환됨 (의도된 동작; 같은 키 = 같은 view).
//
// useWebRTC() 는 컴포넌트에 1개 핸들을 돌려준다. connect/disconnect 는 풀의
// acquire/release 와 1:1. onUnmounted 자동 release.

import { ref, onUnmounted } from 'vue';

const STREAMING_SERVER = 'http://localhost:5002'

// 풀 entry: { pc, track, stream, streamId, refCount, pending, key, params }
const pool = new Map()

function _stableKey (topic, config) {
    return JSON.stringify({
        topic,
        resize: config?.resize || null,
        cropped_area: config?.cropped_area || null,
        rotate: config?.rotate ?? 0,
        msg_type: config?.msg_type || null,
    })
}

async function _negotiate (entry, topic, config) {
    const pc = entry.pc
    pc.addTransceiver('video', { direction: 'recvonly' })
    pc.ontrack = (event) => {
        entry.track = event.track
        entry.stream = event.streams?.[0] || null
        // 누적된 onTrack 콜백 모두 호출
        const callbacks = entry.pending.splice(0)
        for (const cb of callbacks) {
            try { cb(event) } catch (e) { console.error('webrtc onTrack cb:', e) }
        }
    }
    const offer = await pc.createOffer({ offerToReceiveVideo: true })
    if (pc.signalingState === 'closed') return null
    await pc.setLocalDescription(offer)
    if (pc.signalingState === 'closed') return null

    const resp = await fetch(`${STREAMING_SERVER}/offer`, {
        method: 'POST',
        body: JSON.stringify({
            sdp: offer.sdp, type: offer.type, topic, config,
            msg_type: config?.msg_type,
        }),
        headers: { 'Content-Type': 'application/json' },
    })
    const answer = await resp.json()
    if (pc.signalingState === 'closed') return null
    await pc.setRemoteDescription(new RTCSessionDescription(answer))
    entry.streamId = answer.stream_id
    return answer.stream_id
}

async function _acquire (topic, config, onTrack) {
    const key = _stableKey(topic, config)
    let entry = pool.get(key)
    if (entry) {
        entry.refCount += 1
        // 이미 track 받은 상태면 즉시 콜백 (다른 viewer 도 같은 track 으로 시작).
        if (entry.track) {
            try {
                onTrack({ track: entry.track, streams: entry.stream ? [entry.stream] : [] })
            } catch (e) { console.error('webrtc onTrack cb (cached):', e) }
        } else {
            entry.pending.push(onTrack)
        }
        return { pc: entry.pc, stream_id: entry.streamId, key }
    }

    // 같은 호스트(localhost) 연결이라 외부 STUN 불필요. 외부 STUN 을 두면 백엔드
    // aiortc 가 IPv6 로 resolve 된 stun.l.google.com 에 ~5s 타임아웃 후에야 host
    // candidate 로 폴백해 카메라가 5초 늦게 뜬다. host candidate 만 쓰면 즉시 연결.
    const pc = new RTCPeerConnection({ iceServers: [] })
    entry = {
        pc, track: null, stream: null, streamId: null,
        refCount: 1, pending: [onTrack], key, params: { topic, config },
    }
    pool.set(key, entry)
    try {
        await _negotiate(entry, topic, config)
    } catch (e) {
        console.error('webrtc negotiate failed:', e)
        try { pc.close() } catch { /* ignore */ }
        pool.delete(key)
        throw e
    }
    return { pc: entry.pc, stream_id: entry.streamId, key }
}

function _release (key) {
    const entry = pool.get(key)
    if (!entry) return
    entry.refCount -= 1
    if (entry.refCount <= 0) {
        try { entry.pc.close() } catch { /* ignore */ }
        pool.delete(key)
    }
}

async function _addConfigShared (streamId, config) {
    if (!streamId) return
    try {
        await fetch(`${STREAMING_SERVER}/add_config`, {
            method: 'POST',
            body: JSON.stringify({ stream_id: streamId, config }),
            headers: { 'Content-Type': 'application/json' },
        })
    } catch (e) {
        console.error('webrtc add_config:', e)
    }
}

export function useWebRTC () {
    const streamId = ref(null)
    const connectionState = ref('new')
    const error = ref(null)
    let activeKey = null

    const connect = async (topic, config, onTrack) => {
        // 이전 키가 있고 새 키와 다르면 release 후 acquire — 같으면 그대로 (no-op).
        const nextKey = _stableKey(topic, config)
        if (activeKey === nextKey) {
            const entry = pool.get(activeKey)
            if (entry) {
                streamId.value = entry.streamId
                if (entry.track) onTrack({ track: entry.track, streams: entry.stream ? [entry.stream] : [] })
                return { pc: entry.pc, stream_id: entry.streamId }
            }
        }
        if (activeKey) {
            _release(activeKey)
            activeKey = null
        }
        try {
            const res = await _acquire(topic, config, onTrack)
            activeKey = res.key
            streamId.value = res.stream_id
            connectionState.value = res.pc?.connectionState || 'new'
            return { pc: res.pc, stream_id: res.stream_id }
        } catch (e) {
            error.value = e
            return null
        }
    }

    const disconnect = () => {
        if (activeKey) {
            _release(activeKey)
            activeKey = null
        }
        streamId.value = null
        connectionState.value = 'closed'
    }

    const addConfig = (sid, config) => {
        return _addConfigShared(sid || streamId.value, config)
    }

    onUnmounted(() => disconnect())

    return { streamId, connectionState, error, connect, disconnect, addConfig }
}
