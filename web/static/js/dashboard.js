// Connessione WebSocket
const socket = io();

// Latency probe: WS round-trip / 2 + latenza ROS (web node -> simulatore)
let wsLatencyMs = 0;
setInterval(() => socket.emit('latency_ping', { t: performance.now() }), 1000);
socket.on('latency_pong', (data) => { wsLatencyMs = (performance.now() - data.t) / 2; });

// Riferimenti UI
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const valRoll = document.getElementById('val-roll');
const valPitch = document.getElementById('val-pitch');
const valYaw = document.getElementById('val-yaw');
const valPosz = document.getElementById('val-posz');
const footL = document.getElementById('foot-l');
const footR = document.getElementById('foot-r');
const flightBadge = document.getElementById('flight-badge');
const fallAlarm = document.getElementById('fall-alarm');
const telemetryRate = document.getElementById('telemetry-rate');
const recordBanner = document.getElementById('record-banner');
const recordText = document.getElementById('record-text');
const btnRecordStart = document.getElementById('btn-record-start');
const btnRecordStop = document.getElementById('btn-record-stop');
const btnDownloadJson = document.getElementById('btn-download-json');
const btnDownloadCsv = document.getElementById('btn-download-csv');

// Configurazione grafici Plotly
const maxDataPoints = 100;
const chartUpdateIntervalMs = 200;
let timeData = [];
let imuData = { x: [], y: [], z: [] };
let jointData = {
    left_hip: [],
    left_knee: [],
    left_ankle: [],
    right_hip: [],
    right_knee: [],
    right_ankle: []
};
let latencyData = [];
let lastChartUpdateMs = 0;

let currentScenario = 'flat';
let currentBagName = 'dashboard_session';
let currentTelemetryHz = 0;
let sessionMetrics = createEmptySessionMetrics();

const chartColors = {
    paper: '#ffffff',
    plot: '#ffffff',
    grid: '#e4e7ec',
    axis: '#667085',
};

// Grafico IMU
const imuPlotDiv = document.getElementById('plot-imu');
const imuTraces = [
    { x: timeData, y: imuData.x, name: 'Acc X', mode: 'lines', line: { color: '#ef4444', width: 2 } },
    { x: timeData, y: imuData.y, name: 'Acc Y', mode: 'lines', line: { color: '#06b6d4', width: 2 } },
    { x: timeData, y: imuData.z, name: 'Acc Z', mode: 'lines', line: { color: '#8b5cf6', width: 2 } }
];
const imuLayout = {
    paper_bgcolor: chartColors.paper,
    plot_bgcolor: chartColors.plot,
    margin: { t: 20, b: 30, l: 40, r: 10 },
    xaxis: { color: chartColors.axis, showgrid: false, zeroline: false },
    yaxis: { color: chartColors.axis, gridcolor: chartColors.grid, zeroline: false },
    legend: { font: { color: chartColors.axis }, orientation: 'h', y: -0.2 }
};
Plotly.newPlot(imuPlotDiv, imuTraces, imuLayout, { displayModeBar: false });

// Grafico giunti
const jointsPlotDiv = document.getElementById('plot-joints');
const jointTraces = [
    { x: timeData, y: jointData.left_knee, name: 'Ginocchio SX', mode: 'lines', line: { color: '#7c3aed', width: 1.5 } },
    { x: timeData, y: jointData.right_knee, name: 'Ginocchio DX', mode: 'lines', line: { color: '#db2777', width: 1.5 } },
    { x: timeData, y: jointData.left_ankle, name: 'Caviglia SX', mode: 'lines', line: { color: '#2563eb', width: 1.5 } },
    { x: timeData, y: jointData.right_ankle, name: 'Caviglia DX', mode: 'lines', line: { color: '#059669', width: 1.5 } }
];
const jointsLayout = {
    paper_bgcolor: chartColors.paper,
    plot_bgcolor: chartColors.plot,
    margin: { t: 20, b: 30, l: 40, r: 10 },
    xaxis: { color: chartColors.axis, showgrid: false, zeroline: false },
    yaxis: { color: chartColors.axis, gridcolor: chartColors.grid, zeroline: false },
    legend: { font: { color: chartColors.axis }, orientation: 'h', y: -0.2 }
};
Plotly.newPlot(jointsPlotDiv, jointTraces, jointsLayout, { displayModeBar: false });

// Grafico latenza comando
const latencyPlotDiv = document.getElementById('plot-latency');
const latencyTraces = [
    { x: timeData, y: latencyData, name: 'Latenza cmd', mode: 'lines', line: { color: '#1f5f99', width: 2 } },
    { x: timeData, y: [], name: 'Target 50 ms', mode: 'lines', line: { color: '#b42318', width: 1, dash: 'dash' } }
];
const latencyLayout = {
    paper_bgcolor: chartColors.paper,
    plot_bgcolor: chartColors.plot,
    margin: { t: 20, b: 30, l: 46, r: 10 },
    xaxis: { color: chartColors.axis, showgrid: false, zeroline: false },
    yaxis: { color: chartColors.axis, gridcolor: chartColors.grid, zeroline: false, title: 'ms' },
    legend: { font: { color: chartColors.axis }, orientation: 'h', y: -0.2 }
};
Plotly.newPlot(latencyPlotDiv, latencyTraces, latencyLayout, { displayModeBar: false });

// Contatore frequenza telemetria
let msgCount = 0;
let lastRateCalcTime = Date.now();

// Stato connessione
socket.on('connect', () => {
    statusDot.className = 'status-indicator online';
    statusText.innerText = 'Online';
});

socket.on('disconnect', () => {
    statusDot.className = 'status-indicator offline';
    statusText.innerText = 'Disconnesso';
    telemetryRate.innerText = '0 Hz';
});

// Stream telemetria
socket.on('telemetry', (state) => {
    msgCount++;
    const now = Date.now();
    
    // Calcola la frequenza telemetria ogni secondo.
    if (now - lastRateCalcTime >= 1000) {
        currentTelemetryHz = msgCount;
        telemetryRate.innerText = `${currentTelemetryHz} Hz`;
        observeTelemetryRate(currentTelemetryHz);
        msgCount = 0;
        lastRateCalcTime = now;
        checkRecordingStatus(); // Keep record status synced
    }

    // Converte l'orientamento IMU in angoli di Eulero.
    const euler = quaternionToEuler(state.imu.orientation);
    valRoll.innerText = `${euler.roll.toFixed(1)}°`;
    valPitch.innerText = `${euler.pitch.toFixed(1)}°`;
    valYaw.innerText = `${euler.yaw.toFixed(1)}°`;
    valPosz.innerText = `${state.odom.position.z.toFixed(2)} m`;

    const totalLatency = wsLatencyMs + (state.latency_ms || 0);
    document.getElementById('val-latency').innerText = `${totalLatency.toFixed(1)} ms`;
    observeSessionTelemetry(state, euler, totalLatency);

    // Overlay di caduta pilotato dal backend.
    if (state.fall_detected) {
        fallAlarm.classList.add('triggered');
    } else {
        fallAlarm.classList.remove('triggered');
    }

    // Mappa roll/pitch nell'indicatore di assetto.
    const bubbleDot = document.getElementById('tilt-bubble');
    let xOffset = Math.max(-45, Math.min(45, euler.roll)) / 45 * 50;
    let yOffset = Math.max(-45, Math.min(45, -euler.pitch)) / 45 * 50;
    bubbleDot.style.transform = `translate(calc(-50% + ${xOffset}px), calc(-50% + ${yOffset}px))`;

    // Indicatori contatto piedi.
    const leftContact = state.contacts.left;
    const rightContact = state.contacts.right;

    if (leftContact) {
        footL.classList.add('active');
    } else {
        footL.classList.remove('active');
    }

    if (rightContact) {
        footR.classList.add('active');
    } else {
        footR.classList.remove('active');
    }

    // Flight phase: nessun contatto piede rilevato.
    if (!leftContact && !rightContact) {
        flightBadge.classList.remove('hidden');
    } else {
        flightBadge.classList.add('hidden');
    }

    // Aggiorna le serie dei grafici.
    const timestampStr = new Date().toLocaleTimeString();
    timeData.push(timestampStr);
    imuData.x.push(state.imu.linear_acceleration.x);
    imuData.y.push(state.imu.linear_acceleration.y);
    imuData.z.push(state.imu.linear_acceleration.z);
    latencyData.push(totalLatency);

    // Estrae ginocchio e caviglia pitch: indici 3/4 a sinistra, 9/10 a destra.
    const pos = state.joints.position;
    if (pos && pos.length >= 12) {
        jointData.left_knee.push(pos[3]);
        jointData.right_knee.push(pos[9]);
        jointData.left_ankle.push(pos[4]);
        jointData.right_ankle.push(pos[10]);
    } else {
        jointData.left_knee.push(0);
        jointData.right_knee.push(0);
        jointData.left_ankle.push(0);
        jointData.right_ankle.push(0);
    }

    // Finestra mobile sui campioni piu recenti.
    if (timeData.length > maxDataPoints) {
        timeData.shift();
        imuData.x.shift();
        imuData.y.shift();
        imuData.z.shift();
        jointData.left_knee.shift();
        jointData.right_knee.shift();
        jointData.left_ankle.shift();
        jointData.right_ankle.shift();
        latencyData.shift();
    }

    if (now - lastChartUpdateMs >= chartUpdateIntervalMs) {
        lastChartUpdateMs = now;
        updateCharts();
    }
});

function updateCharts() {
    Plotly.update(imuPlotDiv, { x: [timeData, timeData, timeData], y: [imuData.x, imuData.y, imuData.z] });
    Plotly.update(jointsPlotDiv, { x: [timeData, timeData, timeData, timeData], y: [jointData.left_knee, jointData.right_knee, jointData.left_ankle, jointData.right_ankle] });
    Plotly.update(latencyPlotDiv, { x: [timeData, timeData], y: [latencyData, latencyData.map(() => 50)] });
}

// Conversione quaternione -> Eulero in gradi.
function quaternionToEuler(q) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    
    // Roll: rotazione sull'asse X.
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch: rotazione sull'asse Y.
    const sinp = 2 * (w * y - z * x);
    let pitch;
    if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * Math.PI / 2;
    } else {
        pitch = Math.asin(sinp);
    }

    // Yaw: rotazione sull'asse Z.
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return {
        roll: roll * 180 / Math.PI,
        pitch: pitch * 180 / Math.PI,
        yaw: yaw * 180 / Math.PI
    };
}

function createEmptySessionMetrics() {
    return {
        scenario: currentScenario,
        bag_name: currentBagName,
        started_at: new Date().toISOString(),
        ended_at: null,
        sample_count: 0,
        telemetry_hz_samples: [],
        latency_ms_samples: [],
        max_abs_roll_deg: 0,
        max_abs_pitch_deg: 0,
        min_z: null,
        max_z: null,
        fall_detected_count: 0,
        flight_phase_count: 0,
        left_contact_loss_count: 0,
        right_contact_loss_count: 0,
        previous_fall_detected: false,
        previous_flight_phase: false,
        previous_left_contact: null,
        previous_right_contact: null
    };
}

function resetSessionMetrics(bagName = 'dashboard_session') {
    currentBagName = bagName || 'dashboard_session';
    sessionMetrics = createEmptySessionMetrics();
}

function observeTelemetryRate(hz) {
    if (Number.isFinite(hz) && hz > 0) {
        sessionMetrics.telemetry_hz_samples.push(hz);
    }
}

function observeSessionTelemetry(state, euler, latencyMs) {
    sessionMetrics.scenario = currentScenario;
    sessionMetrics.bag_name = currentBagName;
    sessionMetrics.ended_at = new Date().toISOString();
    sessionMetrics.sample_count += 1;

    if (Number.isFinite(latencyMs)) {
        sessionMetrics.latency_ms_samples.push(latencyMs);
    }

    sessionMetrics.max_abs_roll_deg = Math.max(sessionMetrics.max_abs_roll_deg, Math.abs(euler.roll));
    sessionMetrics.max_abs_pitch_deg = Math.max(sessionMetrics.max_abs_pitch_deg, Math.abs(euler.pitch));

    const z = state.odom.position.z;
    if (Number.isFinite(z)) {
        sessionMetrics.min_z = sessionMetrics.min_z === null ? z : Math.min(sessionMetrics.min_z, z);
        sessionMetrics.max_z = sessionMetrics.max_z === null ? z : Math.max(sessionMetrics.max_z, z);
    }

    const fallDetected = Boolean(state.fall_detected);
    if (fallDetected && !sessionMetrics.previous_fall_detected) {
        sessionMetrics.fall_detected_count += 1;
    }
    sessionMetrics.previous_fall_detected = fallDetected;

    const leftContact = Boolean(state.contacts.left);
    const rightContact = Boolean(state.contacts.right);
    const flightPhase = !leftContact && !rightContact;
    if (flightPhase && !sessionMetrics.previous_flight_phase) {
        sessionMetrics.flight_phase_count += 1;
    }
    sessionMetrics.previous_flight_phase = flightPhase;

    if (sessionMetrics.previous_left_contact === true && !leftContact) {
        sessionMetrics.left_contact_loss_count += 1;
    }
    if (sessionMetrics.previous_right_contact === true && !rightContact) {
        sessionMetrics.right_contact_loss_count += 1;
    }
    sessionMetrics.previous_left_contact = leftContact;
    sessionMetrics.previous_right_contact = rightContact;
}

function summarizeSessionMetrics() {
    const startedAt = new Date(sessionMetrics.started_at).getTime();
    const endedAt = sessionMetrics.ended_at ? new Date(sessionMetrics.ended_at).getTime() : Date.now();
    const durationS = Math.max(0, (endedAt - startedAt) / 1000);
    const latency = sessionMetrics.latency_ms_samples;
    const telemetry = sessionMetrics.telemetry_hz_samples;

    return {
        scenario: currentScenario,
        bag_name: currentBagName,
        started_at: sessionMetrics.started_at,
        ended_at: sessionMetrics.ended_at || new Date().toISOString(),
        duration_s: roundMetric(durationS, 2),
        sample_count: sessionMetrics.sample_count,
        telemetry_hz_avg: roundMetric(average(telemetry), 2),
        telemetry_hz_min: roundMetric(minValue(telemetry), 2),
        latency_ms_avg: roundMetric(average(latency), 2),
        latency_ms_p95: roundMetric(percentile(latency, 0.95), 2),
        latency_ms_max: roundMetric(maxValue(latency), 2),
        max_abs_roll_deg: roundMetric(sessionMetrics.max_abs_roll_deg, 2),
        max_abs_pitch_deg: roundMetric(sessionMetrics.max_abs_pitch_deg, 2),
        min_z: roundMetric(sessionMetrics.min_z, 4),
        max_z: roundMetric(sessionMetrics.max_z, 4),
        fall_detected_count: sessionMetrics.fall_detected_count,
        flight_phase_count: sessionMetrics.flight_phase_count,
        left_contact_loss_count: sessionMetrics.left_contact_loss_count,
        right_contact_loss_count: sessionMetrics.right_contact_loss_count
    };
}

function average(values) {
    if (!values.length) return null;
    return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function minValue(values) {
    return values.length ? Math.min(...values) : null;
}

function maxValue(values) {
    return values.length ? Math.max(...values) : null;
}

function percentile(values, p) {
    if (!values.length) return null;
    const sorted = [...values].sort((a, b) => a - b);
    const index = Math.min(sorted.length - 1, Math.ceil(sorted.length * p) - 1);
    return sorted[index];
}

function roundMetric(value, digits) {
    if (value === null || value === undefined || !Number.isFinite(value)) return null;
    const scale = 10 ** digits;
    return Math.round(value * scale) / scale;
}

function downloadMetricsJson() {
    const summary = summarizeSessionMetrics();
    downloadTextFile(
        `${summary.bag_name}_metrics.json`,
        JSON.stringify(summary, null, 2),
        'application/json'
    );
}

function downloadMetricsCsv() {
    const summary = summarizeSessionMetrics();
    downloadTextFile(
        `${summary.bag_name}_metrics.csv`,
        summaryToCsv(summary),
        'text/csv'
    );
}

function summaryToCsv(summary) {
    const headers = Object.keys(summary);
    const values = headers.map((key) => escapeCsvValue(summary[key]));
    return `${headers.join(',')}\n${values.join(',')}\n`;
}

function saveMetricsToBackend(summary) {
    return fetch('/api/metrics/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(summary)
    }).then(res => res.json());
}

function escapeCsvValue(value) {
    if (value === null || value === undefined) return '';
    const text = String(value);
    if (text.includes(',') || text.includes('"') || text.includes('\n')) {
        return `"${text.replaceAll('"', '""')}"`;
    }
    return text;
}

function downloadTextFile(filename, content, mimeType) {
    const blob = new Blob([content], { type: `${mimeType};charset=utf-8` });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
}

// Teleoperazione da tastiera.
let keysPressed = {};
const keyMapping = {
    'KeyW': 'forward',
    'KeyS': 'backward',
    'KeyA': 'left',
    'KeyD': 'right',
    'KeyQ': 'turn_l',
    'KeyE': 'turn_r',
    'Space': 'stop'
};

document.addEventListener('keydown', (e) => {
    if (keyMapping[e.code]) {
        keysPressed[keyMapping[e.code]] = true;
        sendMovementCommand();
        updateKeypadUI();
    }
});

document.addEventListener('keyup', (e) => {
    if (keyMapping[e.code]) {
        keysPressed[keyMapping[e.code]] = false;
        sendMovementCommand();
        updateKeypadUI();
    }
});

function sendMovementCommand() {
    let vx = 0.0;
    let vy = 0.0;
    let wz = 0.0;

    if (keysPressed['forward']) vx = 0.6;
    if (keysPressed['backward']) vx = -0.6;
    if (keysPressed['left']) vy = 0.3;
    if (keysPressed['right']) vy = -0.3;
    if (keysPressed['turn_l']) wz = 0.6;
    if (keysPressed['turn_r']) wz = -0.6;
    if (keysPressed['stop']) {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
    }

    socket.emit('teleop_cmd', { vx, vy, wz });
}

function updateKeypadUI() {
    // Sincronizza lo stato visuale dei pulsanti con i tasti premuti.
    document.getElementById('btn-forward').className = keysPressed['forward'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-backward').className = keysPressed['backward'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-left').className = keysPressed['left'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-right').className = keysPressed['right'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-turn-l').className = keysPressed['turn_l'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-turn-r').className = keysPressed['turn_r'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-stop').className = keysPressed['stop'] ? 'pad-btn stop-btn active' : 'pad-btn stop-btn';
}

// Eventi mouse/touch per i pulsanti direzionali.
const registerButtonEvents = (btnId, keyName) => {
    const btn = document.getElementById(btnId);
    
    const startAction = (e) => {
        e.preventDefault();
        keysPressed[keyName] = true;
        sendMovementCommand();
        updateKeypadUI();
    };

    const stopAction = (e) => {
        e.preventDefault();
        keysPressed[keyName] = false;
        sendMovementCommand();
        updateKeypadUI();
    };

    btn.addEventListener('mousedown', startAction);
    btn.addEventListener('mouseup', stopAction);
    btn.addEventListener('mouseleave', stopAction);
    btn.addEventListener('touchstart', startAction);
    btn.addEventListener('touchend', stopAction);
};

registerButtonEvents('btn-forward', 'forward');
registerButtonEvents('btn-backward', 'backward');
registerButtonEvents('btn-left', 'left');
registerButtonEvents('btn-right', 'right');
registerButtonEvents('btn-turn-l', 'turn_l');
registerButtonEvents('btn-turn-r', 'turn_r');

// Pulsante stop.
const stopBtn = document.getElementById('btn-stop');
stopBtn.addEventListener('mousedown', (e) => {
    e.preventDefault();
    keysPressed = {};
    keysPressed['stop'] = true;
    sendMovementCommand();
    updateKeypadUI();
});
stopBtn.addEventListener('mouseup', (e) => {
    e.preventDefault();
    keysPressed['stop'] = false;
    sendMovementCommand();
    updateKeypadUI();
});

// Registrazione e gestione sessione.
btnRecordStart.addEventListener('click', () => {
    fetch('/api/record/start', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if (data.success) {
                resetSessionMetrics(data.info);
                btnRecordStart.disabled = true;
                btnRecordStop.disabled = false;
                recordBanner.classList.remove('hidden');
                recordText.innerText = `REC: ${data.info}`;
            }
        });
});

btnRecordStop.addEventListener('click', () => {
    fetch('/api/record/stop', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if (data.success) {
                const summary = summarizeSessionMetrics();
                saveMetricsToBackend(summary).catch((error) => {
                    console.warn('Salvataggio metriche non riuscito:', error);
                });
                btnRecordStart.disabled = false;
                btnRecordStop.disabled = true;
                recordBanner.classList.add('hidden');
            }
        });
});

function checkRecordingStatus() {
    fetch('/api/status')
        .then(res => res.json())
        .then(data => {
            currentScenario = data.scenario || currentScenario;
            if (data.is_recording) {
                currentBagName = data.bag_name || currentBagName;
                btnRecordStart.disabled = true;
                btnRecordStop.disabled = false;
                recordBanner.classList.remove('hidden');
                recordText.innerText = `REC: ${data.bag_name}`;
            } else {
                btnRecordStart.disabled = false;
                btnRecordStop.disabled = true;
                recordBanner.classList.add('hidden');
            }
        });
}

btnDownloadJson.addEventListener('click', downloadMetricsJson);
btnDownloadCsv.addEventListener('click', downloadMetricsCsv);

// Controllo iniziale stato registrazione.
checkRecordingStatus();

document.getElementById('btn-reset').addEventListener('click', () => {
    fetch('/api/reset', { method: 'POST' });
});
