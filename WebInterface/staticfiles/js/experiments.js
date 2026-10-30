let experimentRunning = false;
let iAmInControl = false;
let heartbeatInterval = null;
let pollInterval = null;
let inactivityTimer = null;
const INACTIVITY_TIMEOUT = 2 * 60 * 1000; //2 minutes

// ─────────────────────────────────────────────
//Control elements to disable when user not in control
const CONTROL_ELEMENTS = [
    'startBtn',
    'stopBtn',
    'updateParamsBtn',
    'restoreDefaultsBtn',
];

//Also disable every editable param-input (not just read only)
function getEditableParamInputs() {
    return document.querySelectorAll('.param-input:not([readonly]):not([disabled])');
}

async function triggerEstop() {
    try {
        const res = await fetch('/control/estop/', {
            method: 'POST',
            headers: { 'X-CSRFToken': getCsrfToken() }
        });
        const data = await res.json();

        if (data.status === 'estop_ok') {
            // Give instructor control first so sendCommand works
            iAmInControl = true;
            // Stop the experiment on the Pi
            fetch(EXPERIMENT_URLS.command, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'X-CSRFToken': getCsrfToken()
                },
                body: JSON.stringify({ command: 'sto', parameters: {} })
            });
            // Update full UI state
            setControlMode(true);
            startHeartbeat();
            // Clear any running state
            experimentRunning = false;
            updateUIState();
            document.getElementById('paramStatus').textContent = 'Stopped';
            document.getElementById('paramStatus').classList.remove('bg-warning');
            document.getElementById('paramStatus').classList.add('bg-light', 'text-dark');
        }
    } catch (err) {
        console.error('E-Stop error:', err);
    }
}


//Control lock — acquire / release / heartbeat
//Heartbeat to see if user is active, acquire lock when first in queue, release lock on inactivity or tab close
function getCsrfToken() {
    return document.querySelector('[name=csrfmiddlewaretoken]').value;
}

async function acquireLock() {
    try {
        const res = await fetch('/control/acquire/', {
            method: 'POST',
            headers: { 'X-CSRFToken': getCsrfToken() }
        });
        const data = await res.json();

        if (data.status === 'acquired' || data.status === 'already_yours') {
            setControlMode(true);
            startHeartbeat();
        } else if (data.status === 'viewer') {
            //Permanently view only (demo user)
            setViewerMode()  
        } else {
            //Lock held by someone else
            setControlMode(false);
            startPolling();
        }
    } catch (err) {
        console.error('acquireLock error:', err);
        setControlMode(false);
        startPolling();
    }
}

/*SET VIEW ONLY MODE FOR VIEWER ACCOUNTS (CANNOT ACQUIRE CONTROL)*/
function setViewerMode() {
    iAmInControl = false;

    CONTROL_ELEMENTS.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = true;
    });

    getEditableParamInputs().forEach(el => {
        el.disabled = true;
    });

    const banner = document.getElementById('controlStatusBanner');
    const text   = document.getElementById('controlStatusText');
    banner.style.display = 'flex';
    banner.className = 'alert alert-secondary';
    text.textContent = 'View only account - controls disabled';
}

async function releaseLock() {
    try {
        await fetch('/control/release/', {
            method: 'POST',
            keepalive: true,
            headers: { 'X-CSRFToken': getCsrfToken() }
        });
    } catch (err) {
        console.error('releaseLock error:', err);
    }
}

function startHeartbeat() {
    //clear any existing poll
    if (pollInterval) {
        clearInterval(pollInterval);
        pollInterval = null;
    }

    heartbeatInterval = setInterval(async () => {
        try {
            const res = await fetch('/control/heartbeat/', {
                method: 'POST',
                headers: { 'X-CSRFToken': getCsrfToken() }
            });
            const data = await res.json();

            if (data.status === 'lost') {
                //Server no longer recognises our lock (admin clear)
                clearInterval(heartbeatInterval);
                heartbeatInterval = null;
                setControlMode(false);
                startPolling();
            }
        } catch (err) {
            console.error('Heartbeat error:', err);
        }
    }, 15000); //every 15 seconds
}

function startPolling() {
    // Clear any existing heartbeat - no user control
    if (heartbeatInterval) {
        clearInterval(heartbeatInterval);
        heartbeatInterval = null;
    }

    pollInterval = setInterval(async () => {
        try {
            const res = await fetch('/control/status/');
            const data = await res.json();

            if (!data.locked) {
                //Lock is free, stop polling and try to grab it
                clearInterval(pollInterval);
                pollInterval = null;
                acquireLock();
            }
        } catch (err) {
            console.error('Poll error:', err);
        }
    }, 5000); //every 5 seconds
}

//  Inactivity timer resets on any control interaction, releases lock if time runs out
function resetInactivityTimer() {
    if (!iAmInControl) return;

    if (inactivityTimer) {
        clearTimeout(inactivityTimer);
    }

    inactivityTimer = setTimeout(() => {
        //Timer up, release lock and switch to view only
        releaseLock();
        setControlMode(false);
        startPolling();

        // Hide regular banner, show inactivity banner
        document.getElementById('controlStatusBanner').style.display = 'none';
        document.getElementById('inactivityBanner').style.display = 'flex';
    }, INACTIVITY_TIMEOUT);
}

async function requestControl() {
    document.getElementById('inactivityBanner').style.display = 'none';
    await acquireLock();
}

//  Control lock — UI mode switching
function setControlMode(inControl) {
    iAmInControl = inControl;

    CONTROL_ELEMENTS.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = !inControl;
    });

    getEditableParamInputs().forEach(el => {
        el.disabled = !inControl;
    });

    const banner = document.getElementById('controlStatusBanner');
    const text = document.getElementById('controlStatusText');
    const inactivityBanner = document.getElementById('inactivityBanner');

    if (inControl) {
        banner.style.display = 'none';
        inactivityBanner.style.display = 'none';
        resetInactivityTimer(); //start countdown when control gained
    } else {
        //Clear inactivity timer when control is lost
        if (inactivityTimer) {
            clearTimeout(inactivityTimer);
            inactivityTimer = null;
        }
        banner.style.display = 'flex';
        banner.className = 'alert alert-danger';
        text.innerHTML = '<b>View only</b><br>Another user is in control';
    }

    updateUIState();
}

// ─────────────────────────────────────────────
//  Experiment control functions
// ─────────────────────────────────────────────
function startExperiment() {
    if (!iAmInControl) return;
    resetInactivityTimer();
    experimentRunning = true;
    updateUIState();

    // Collect current parameter values from the form
    const params = {};
    document.querySelectorAll('#paramForm .param-input:not([disabled]):not([readonly])').forEach(input => {
        params[input.name] = input.value;
    });

    sendCommand('sta', params);

    document.getElementById('paramStatus').textContent = 'Running';
    document.getElementById('paramStatus').classList.remove('bg-light', 'text-dark');
    document.getElementById('paramStatus').classList.add('bg-warning', 'text-dark');
}

function stopExperiment() {
    if (!iAmInControl) return;
    resetInactivityTimer();
    experimentRunning = false;
    updateUIState();
    sendCommand('sto');
    
    document.getElementById('paramStatus').textContent = 'Stopped';
    document.getElementById('paramStatus').classList.remove('bg-warning');
    document.getElementById('paramStatus').classList.add('bg-light', 'text-dark');
}

function recenterExperiment() {
    if (!iAmInControl) return;
    resetInactivityTimer();
    if (!experimentRunning) {
        sendCommand('rec');
    }
}


//  UI state — combines lock state + running state
function updateUIState() {
    const parametersBody = document.getElementById('parametersBody');

    if (experimentRunning) {
        parametersBody.classList.add('disabled-overlay');
    } else {
        parametersBody.classList.remove('disabled-overlay');
    }
}

//  Send command to Django view
function sendCommand(command, parameters = {}) {
    if (!iAmInControl) return;

    fetch(EXPERIMENT_URLS.command, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': getCsrfToken()
        },
        body: JSON.stringify({ command: command, parameters: parameters })
    })
    .then(response => response.json())
    .then(data => {
        console.log('Command sent:', data);
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

// ─────────────────────────────────────────────
//  Parameter functions
// ─────────────────────────────────────────────
function updateParameters() {
    if (!iAmInControl) return;
    resetInactivityTimer();

    if (experimentRunning) {
        alert('Cannot update parameters while experiment is running. Please stop the experiment first.');
        return;
    }
    
    const formData = new FormData(document.getElementById('paramForm'));
    const params = {};
    
    formData.forEach((value, key) => {
        if (key !== 'action' && key !== 'csrfmiddlewaretoken') {
            params[key] = value;
        }
    });
    
    fetch(EXPERIMENT_URLS.updateParams, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': getCsrfToken()
        },
        body: JSON.stringify({ parameters: params })
    })
    .then(response => response.json())
    .then(data => {
        console.log('Updated parameters:', data);
    })
    .catch(error => {
        console.error('Error:', error);
        alert('Error updating parameters');
    });
}

function restoreDefaults() {
    if (!iAmInControl) return;
    resetInactivityTimer();

    if (experimentRunning) {
        alert('Cannot restore defaults while experiment is running.');
        return;
    }

    document.querySelectorAll('.param-input').forEach(input => {
        const defaultValue = input.getAttribute('data-default');
        if (defaultValue) {
            input.value = defaultValue;
        }
    });

    updateParameters();
}

// ─────────────────────────────────────────────
//  Camera / HLS stream --> replaced with webrtc
// ─────────────────────────────────────────────
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
			setTimeout(() => {
				if(window.peerConnection) { window.peerConnection.close(); }
				initStream();
			}, 2000);
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

function reloadStream() {
	if (window.peerConnection) { window.peerConnection.close(); }
	initStream();
}

// ─────────────────────────────────────────────
//  Live charts — dynamic, key-driven
// ─────────────────────────────────────────────
const MAX_POINTS = 200; // rolling window of data points per chart

// Map of key → Chart.js instance, created on first data arrival
const activeCharts = {};

// Colors assigned in order to each new key that appears
const CHART_COLORS = [
    '#0d6efd', // blue
    '#198754', // green
    '#dc3545', // red
    '#fd7e14', // orange
    '#6f42c1', // purple
    '#20c997', // teal
];
let colorIndex = 0;

function createChart(key) {
    // Remove placeholder text on first chart creation
    const placeholder = document.getElementById('chartsPlaceholder');
    if (placeholder) placeholder.remove();

    // Create a col div containing a canvas
    const container = document.getElementById('chartsContainer');
    const col = document.createElement('div');
    col.className = 'col-md-6 mb-3';
    col.id = `chart-col-${key}`;

    const wrapper = document.createElement('div');
    wrapper.className = 'chart-wrapper';

    const canvas = document.createElement('canvas');
    canvas.id = `chart-${key}`;
    wrapper.appendChild(canvas);
    col.appendChild(wrapper);
    container.appendChild(col);

    // Pick next color
    const color = CHART_COLORS[colorIndex % CHART_COLORS.length];
    colorIndex++;

    // Format the key as a readable label (underscores -> spaces, title case)
    const label = key.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());

    const chart = new Chart(canvas, {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                label: label,
                data: [],
                borderColor: color,
                borderWidth: 1.5,
                pointRadius: 0,
                tension: 0.3,
                fill: false,
            }]
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    display: true,
                    title: { display: true, text: 'Time (s)' },
                    ticks: { maxTicksLimit: 8 }
                },
                y: {
                    display: true,
                    title: { display: true, text: label }
                }
            },
            plugins: {
                legend: { display: true }
            }
        }
    });

    activeCharts[key] = chart;
}

function pushToChart(chart, t, value) {
    chart.data.labels.push(t);
    chart.data.datasets[0].data.push(value);

    // Rolling window — drop oldest point when limit reached
    if (chart.data.labels.length > MAX_POINTS) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
    }

    chart.update('none'); // skip animation for speed
}

function clearCharts() {
    Object.values(activeCharts).forEach(chart => {
        chart.data.labels = [];
        chart.data.datasets[0].data = [];
        chart.update('none');
    });
}

// ─────────────────────────────────────────────
//  Message buffer — throttles chart redraws to
//  10x per second regardless of incoming rate
// ─────────────────────────────────────────────
const messageBuffer = [];
const CHART_UPDATE_INTERVAL = 100; // ms — redraw charts 10x per second

setInterval(() => {
    if (messageBuffer.length === 0) return;

    const data = messageBuffer[messageBuffer.length - 1];
    messageBuffer.length = 0;

    try {
        const t = data.time !== undefined ? parseFloat(data.time.toFixed(2)) : new Date().toLocaleTimeString();
        
        Object.entries(data).forEach(([key, value]) => {
            if (key === 'time') return;
            if (typeof value !== 'number') return;
            if (!activeCharts[key]) {
                createChart(key);
            }
            pushToChart(activeCharts[key], t, value);
        });
    } catch(err) {
        console.error('Chart update error:', err);
    }
}, CHART_UPDATE_INTERVAL);

// ─────────────────────────────────────────────
//  Rig status badge
//
//  Fed by MQTT topic pendulum/status, which dac_daemon.py publishes with
//  retain=True and a Last Will of "offline". Because the messages are
//  retained, a freshly-opened dashboard learns the rig's current state
//  immediately rather than waiting for the next state change.
// ─────────────────────────────────────────────
const RIG_STATUS_STYLES = {
    online:  { text: 'RIG: ONLINE',  cls: 'badge bg-success',    is_error: false   },
    running: { text: 'RIG: RUNNING', cls: 'badge bg-primary',    is_error: false   },
    homing:  { text: 'RIG: HOMING',  cls: 'badge bg-warning',    is_error: false   },
    stopped: { text: 'RIG: STOPPED', cls: 'badge bg-secondary',  is_error: false   },
    offline: { text: 'RIG: OFFLINE', cls: 'badge bg-danger',     is_error: true    },
    error:   { text: 'RIG: ERROR',   cls: 'badge bg-danger',     is_error: true    },
    unknown:  { text: 'RIG: UNKNOWN', cls: 'badge bg-secondary', is_error: true    },
};

function updateRigStatus(status, detail) {
    const el = document.getElementById('rigStatus');
    if (!el) return;

    const style = RIG_STATUS_STYLES[status] || RIG_STATUS_STYLES["unknown"];

    const statusBanner = document.getElementById('rigStatusBanner');
    const statusBannerText = document.getElementById('rigStatusText');
    if (style.is_error && iAmInControl) {
        statusBanner.style.display = 'flex';
        statusBanner.className = 'alert alert-danger';
        statusBannerText.innerHTML = "<b>Rig Error</b><br>Control functionality may be limited"
    } else {
        statusBanner.style.display = 'none';
    }

    // For "running", the detail is the model name — worth showing, since it
    // tells you whether someone else's experiment is already on the hardware.
    el.textContent = detail ? `${style.text} (${detail})` : style.text;
    el.className = style.cls;
    el.title = detail ? `${status}: ${detail}` : status;
}

// ─────────────────────────────────────────────
//  WebSocket — receives sensor data from server
// ─────────────────────────────────────────────
let sensorSocket = null;

function initWebSocket() {
    // Match the page's scheme: wss:// on HTTPS, ws:// on plain HTTP. A
    // hardcoded wss:// fails outright when the site is served over HTTP,
    // which is how it runs before TLS is enabled.
    const wsScheme = (window.location.protocol === 'https:') ? 'wss://' : 'ws://';
    const wsUrl = wsScheme + window.location.host + '/ws/sensor/';
    sensorSocket = new WebSocket(wsUrl);

    sensorSocket.onopen = function() {
        document.getElementById('wsStatus').textContent = 'WS: CONNECTED';
        document.getElementById('wsStatus').className = 'badge bg-success';
    };

    sensorSocket.onclose = function() {
        document.getElementById('wsStatus').textContent = 'WS: DISCONNECTED';
        document.getElementById('wsStatus').className = 'badge bg-danger';
        // With the socket down we no longer know the rig's state; showing the
        // last-seen value would be misleading.
        updateRigStatus('unknown', '');
        // Reconnect after 3 seconds
        setTimeout(initWebSocket, 3000);
    };

    sensorSocket.onerror = function() {
        document.getElementById('wsStatus').textContent = 'WS: ERROR';
        document.getElementById('wsStatus').className = 'badge bg-danger';
    };

    sensorSocket.onmessage = function(e) {
        const outer = JSON.parse(e.data);

        // Rig state (from MQTT pendulum/status) shares this socket with the
        // sensor stream. Route it away before the charting path, or its
        // fields end up in the chart buffer.
        if (outer.type === 'rig_status') {
            updateRigStatus(outer.status, outer.detail);
            return;
        }

        let data = outer.data || outer;

        // If still wrapped as raw string, parse it
        if (data.raw !== undefined) {
            try {
                // Fix unquoted keys and parse
                const fixed = data.raw.replace(/(\{|,)\s*(\w+)\s*:/g, '$1"$2":');
                data = JSON.parse(fixed);
            } catch(err) {
                return; // genuinely unparseable, skip
            }
        }

        messageBuffer.push(data);
    };
}

// ─────────────────────────────────────────────
//  Release lock when tab/browser closes
// ─────────────────────────────────────────────
window.addEventListener('beforeunload', () => {
    if (iAmInControl) {
        releaseLock();
    }
});

// ─────────────────────────────────────────────
//  Init on page load
// ─────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', function() {
    initStream();
    acquireLock();       // Try to take control; falls back to view-only automatically
    updateUIState();     // Set initial button states
    initWebSocket();     // Connect to live data WebSocket
});

