// composables/useWebRTC.js
import { ref, onUnmounted } from 'vue';

const streaming_server = 'http://localhost:5002'

export function useWebRTC() {
    const pc = ref(null);
    const streamId = ref(null);
    const streams = ref([]);
    const connectionState = ref('new');
    const error = ref(null);

    const connect = async (topic, config, onTrack) => {
        // 이전 연결이 있으면 먼저 닫기
        disconnect();

        const peerConnection = new RTCPeerConnection({
            iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
        });

        pc.value = peerConnection;

        peerConnection.addTransceiver("video", { direction: "recvonly" });

        peerConnection.ontrack = (event) => {
            console.log('Track received:', event.streams[0].id);
            onTrack(event);
        };

        peerConnection.onconnectionstatechange = () => {
            connectionState.value = peerConnection.connectionState;
        };

        const offer = await peerConnection.createOffer({ offerToReceiveVideo: true });
        await peerConnection.setLocalDescription(offer);

        const response = await fetch(`${streaming_server}/offer`, {
            method: "POST",
            body: JSON.stringify({ sdp: offer.sdp, type: offer.type, topic, config, msg_type: config.msg_type }),
            headers: { "Content-Type": "application/json" },
        });

        const answer = await response.json();

        await peerConnection.setRemoteDescription(new RTCSessionDescription(answer));

        streamId.value = answer.stream_id;

        return {
            pc: peerConnection,
            stream_id: answer.stream_id,
        };
    };

    const disconnect = () => {
        if (pc.value) {
            pc.value.close();
            pc.value = null;
        }
        streamId.value = null;
        streams.value = [];
        connectionState.value = 'closed';
    };

    const addConfig = async (sid, config) => {
        const id = sid || streamId.value;
        if (!id) return;
        await fetch(`${streaming_server}/add_config`, {
            method: "POST",
            body: JSON.stringify({ stream_id: id, config }),
            headers: { "Content-Type": "application/json" },
        });
    };

    onUnmounted(() => {
        disconnect();
    });

    return { streams, connectionState, error, connect, disconnect, addConfig };
}
