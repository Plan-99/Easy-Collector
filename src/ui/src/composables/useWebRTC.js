// composables/useWebRTC.js
import { ref, onUnmounted } from 'vue';
// import { api } from 'src/boot/axios';

const streaming_server = 'http://192.168.50.46:5002'

export function useWebRTC() {
    const pc = ref(null);
    const streams = ref([]);
    const connectionState = ref('new');
    const error = ref(null);

    const connect = async (topic, config, onTrack) => {
        const pc = new RTCPeerConnection({
            iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
        });

        pc.addTransceiver("video", { direction: "recvonly" });

        pc.ontrack = (event) => {
            console.log('Track received:', event.streams[0].id);
            onTrack(event);
        };

        const offer = await pc.createOffer({ offerToReceiveVideo: true });
        await pc.setLocalDescription(offer);

        const response = await fetch(`${streaming_server}/offer`, {
            method: "POST",
            body: JSON.stringify({ sdp: offer.sdp, type: offer.type, topic, config }),
            headers: { "Content-Type": "application/json" },
        });

        const answer = await response.json();

        await pc.setRemoteDescription(new RTCSessionDescription(answer));

        return {
            pc,
            stream_id: answer.stream_id,
        };
    };

    const disconnect = () => {
        if (pc.value) {
            pc.value.close();
            pc.value = null;
        }
        streams.value = [];
        connectionState.value = 'closed';

        // socket.off('answer')
    };

    const addConfig = async (streamId, config) => {
        await fetch(`${streaming_server}/add_config`, {
            method: "POST",
            body: JSON.stringify({ stream_id: streamId, config }),
            headers: { "Content-Type": "application/json" },
        });
    };


    onUnmounted(() => {
        disconnect();
    });

    return { streams, connectionState, error, connect, disconnect, addConfig };
}