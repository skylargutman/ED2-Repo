function initStream() {
    const video = document.getElementById('cameraStream');
    document.getElementById('streamError').style.display = 'none';
    document.getElementById('streamStatus').textContent = 'CONNECTING...';
    document.getElementById('streamStatus').className = 'badge bg-warning';

    const pc = new RTCPeerConnection();
    pc.addTransceiver('video', { direction: 'recvonly' });

    pc.addEventListener('track', (evt) => {
        if (evt.receiver) {
            try { evt.receiver.jitterBufferTarget = 0; } catch(e) {}
        }
        video.srcObject = evt.streams[0];
        video.play();
        document.getElementById('streamStatus').textContent = 'LIVE';
        document.getElementById('streamStatus').className = 'badge bg-success';
    });

    pc.addEventListener('connectionstatechange', () => {
        if (['failed', 'disconnected'].includes(pc.connectionState)) {
            document.getElementById('streamStatus').textContent = 'OFFLINE';
            document.getElementById('streamStatus').className = 'badge bg-danger';
            document.getElementById('streamError').style.display = 'block';
        }
    });

    pc.createOffer()
        .then(offer => pc.setLocalDescription(offer))
        .then(() => fetch('/stream/whep/', {
            method: 'POST',
            headers: { 'Content-Type': 'application/sdp' },
            body: pc.localDescription.sdp
        }))
        .then(r => r.text())
        .then(sdp => pc.setRemoteDescription({ type: 'answer', sdp }))
        .catch(err => {
            console.error('WebRTC error:', err);
            document.getElementById('streamStatus').textContent = 'OFFLINE';
            document.getElementById('streamStatus').className = 'badge bg-danger';
            document.getElementById('streamError').style.display = 'block';
        });

    window.peerConnection = pc;
}

document.addEventListener('DOMContentLoaded', initStream);

