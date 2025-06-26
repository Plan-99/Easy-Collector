// composables/useWebRTC.js
import { ref, onUnmounted } from 'vue';
// import { api } from 'src/boot/axios';

const streaming_server = 'http://127.0.0.1:5001'

export function useWebRTC() {
    const pc = ref(null);
    const streams = ref([]);
    const connectionState = ref('new');
    const error = ref(null);

    const connect = async (sensor, onTrack) => {
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
            body: JSON.stringify({ sdp: offer.sdp, type: offer.type, sensor }),
            headers: { "Content-Type": "application/json" },
        });

        const answer = await response.json();
        await pc.setRemoteDescription(new RTCSessionDescription(answer));

        // console.log('aaaaa')

        // socket.emit('offer', {
        //     sdp: offer.sdp, type: offer.type, sensor: sensor
        // })

        // socket.on('answer', async (data) => {
        //     await pc.setRemoteDescription(new RTCSessionDescription(data));
        // });


        return pc;
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

    onUnmounted(() => {
        disconnect();
    });

    return { streams, connectionState, error, connect, disconnect };
}