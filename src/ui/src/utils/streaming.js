export async function startStreaming(httpserver, onTrack) {
    const pc = new RTCPeerConnection({
        iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
    });

    pc.addTransceiver("video", { direction: "recvonly" });
    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = (event) => {
        onTrack(event);
    };

    const offer = await pc.createOffer({ offerToReceiveVideo: true });
    await pc.setLocalDescription(offer);

    const response = await fetch(`${httpserver}/offer`, {
        method: "POST",
        body: JSON.stringify({ sdp: offer.sdp, type: offer.type }),
        headers: { "Content-Type": "application/json" },
    });

    const answer = await response.json();
    await pc.setRemoteDescription(new RTCSessionDescription(answer));

    return pc;
}
  