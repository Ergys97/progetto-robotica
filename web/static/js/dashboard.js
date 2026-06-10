// WebSocket connection
const socket = io();

// UI Elements
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

// Plotly Charts Setup
const maxDataPoints = 100;
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

// IMU Plot
const imuPlotDiv = document.getElementById('plot-imu');
const imuTraces = [
    { x: timeData, y: imuData.x, name: 'Acc X', mode: 'lines', line: { color: '#ef4444', width: 2 } },
    { x: timeData, y: imuData.y, name: 'Acc Y', mode: 'lines', line: { color: '#06b6d4', width: 2 } },
    { x: timeData, y: imuData.z, name: 'Acc Z', mode: 'lines', line: { color: '#8b5cf6', width: 2 } }
];
const imuLayout = {
    paper_bgcolor: 'rgba(0,0,0,0)',
    plot_bgcolor: 'rgba(0,0,0,0)',
    margin: { t: 20, b: 30, l: 40, r: 10 },
    xaxis: { color: '#9ca3af', showgrid: false, zeroline: false },
    yaxis: { color: '#9ca3af', gridcolor: 'rgba(255,255,255,0.05)', zeroline: false },
    legend: { font: { color: '#9ca3af' }, orientation: 'h', y: -0.2 }
};
Plotly.newPlot(imuPlotDiv, imuTraces, imuLayout, { displayModeBar: false });

// Joints Plot
const jointsPlotDiv = document.getElementById('plot-joints');
const jointTraces = [
    { x: timeData, y: jointData.left_knee, name: 'L Knee', mode: 'lines', line: { color: '#a855f7', width: 1.5 } },
    { x: timeData, y: jointData.right_knee, name: 'R Knee', mode: 'lines', line: { color: '#ec4899', width: 1.5 } },
    { x: timeData, y: jointData.left_ankle, name: 'L Ankle', mode: 'lines', line: { color: '#3b82f6', width: 1.5 } },
    { x: timeData, y: jointData.right_ankle, name: 'R Ankle', mode: 'lines', line: { color: '#10b981', width: 1.5 } }
];
const jointsLayout = {
    paper_bgcolor: 'rgba(0,0,0,0)',
    plot_bgcolor: 'rgba(0,0,0,0)',
    margin: { t: 20, b: 30, l: 40, r: 10 },
    xaxis: { color: '#9ca3af', showgrid: false, zeroline: false },
    yaxis: { color: '#9ca3af', gridcolor: 'rgba(255,255,255,0.05)', zeroline: false },
    legend: { font: { color: '#9ca3af' }, orientation: 'h', y: -0.2 }
};
Plotly.newPlot(jointsPlotDiv, jointTraces, jointsLayout, { displayModeBar: false });

// Telemetry Rate Counter
let msgCount = 0;
let lastRateCalcTime = Date.now();

// Connection Event Handlers
socket.on('connect', () => {
    statusDot.className = 'status-indicator online';
    statusText.innerText = 'Online';
});

socket.on('disconnect', () => {
    statusDot.className = 'status-indicator offline';
    statusText.innerText = 'Disconnected';
    telemetryRate.innerText = '0 Hz';
});

// Telemetry Data Listener
socket.on('telemetry', (state) => {
    msgCount++;
    const now = Date.now();
    
    // Calculate telemetry rate every 1s
    if (now - lastRateCalcTime >= 1000) {
        telemetryRate.innerText = `${msgCount} Hz`;
        msgCount = 0;
        lastRateCalcTime = now;
        checkRecordingStatus(); // Keep record status synced
    }

    // Parse IMU orientation to Euler angles (Roll/Pitch/Yaw)
    const euler = quaternionToEuler(state.imu.orientation);
    valRoll.innerText = `${euler.roll.toFixed(1)}°`;
    valPitch.innerText = `${euler.pitch.toFixed(1)}°`;
    valYaw.innerText = `${euler.yaw.toFixed(1)}°`;
    valPosz.innerText = `${state.odom.position.z.toFixed(2)} m`;

    // Update safety fall alarm overlay
    // Threshold is 35 degrees
    if (Math.abs(euler.roll) > 35 || Math.abs(euler.pitch) > 35) {
        fallAlarm.classList.add('triggered');
    } else {
        fallAlarm.classList.remove('triggered');
    }

    // Update 2D tilt bubble
    // Map -45 to 45 deg to -50% to 50% dot displacement
    const bubbleDot = document.getElementById('tilt-bubble');
    let xOffset = Math.max(-45, Math.min(45, euler.roll)) / 45 * 50;
    let yOffset = Math.max(-45, Math.min(45, -euler.pitch)) / 45 * 50;
    bubbleDot.style.transform = `translate(calc(-50% + ${xOffset}px), calc(-50% + ${yOffset}px))`;

    // Foot Contacts Indicators
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

    // Flight phase alert if both foot contacts are false
    if (!leftContact && !rightContact) {
        flightBadge.classList.remove('hidden');
    } else {
        flightBadge.classList.add('hidden');
    }

    // Push data to charts lists
    const timestampStr = new Date().toLocaleTimeString();
    timeData.push(timestampStr);
    imuData.x.push(state.imu.linear_acceleration.x);
    imuData.y.push(state.imu.linear_acceleration.y);
    imuData.z.push(state.imu.linear_acceleration.z);

    // Extract joint positions for knee & ankle pitch (indices 3, 4 for left, 9, 10 for right)
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

    // Slidind window crop
    if (timeData.length > maxDataPoints) {
        timeData.shift();
        imuData.x.shift();
        imuData.y.shift();
        imuData.z.shift();
        jointData.left_knee.shift();
        jointData.right_knee.shift();
        jointData.left_ankle.shift();
        jointData.right_ankle.shift();
    }

    // Redraw plots (relayout only x-axis to animate smoothly, restructure data)
    Plotly.update(imuPlotDiv, { x: [timeData, timeData, timeData], y: [imuData.x, imuData.y, imuData.z] });
    Plotly.update(jointsPlotDiv, { x: [timeData, timeData, timeData, timeData], y: [jointData.left_knee, jointData.right_knee, jointData.left_ankle, jointData.right_ankle] });
});

// Helper: Quaternion to Euler (Roll, Pitch, Yaw in degrees)
function quaternionToEuler(q) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    
    // roll (x-axis rotation)
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    const sinp = 2 * (w * y - z * x);
    let pitch;
    if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * Math.PI / 2; // use 90 degrees if out of range
    } else {
        pitch = Math.asin(sinp);
    }

    // yaw (z-axis rotation)
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return {
        roll: roll * 180 / Math.PI,
        pitch: pitch * 180 / Math.PI,
        yaw: yaw * 180 / Math.PI
    };
}

// Keyboard Teleoperation Listener
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

    if (keysPressed['forward']) vx = 0.6;   // Forward velocity
    if (keysPressed['backward']) vx = -0.6; // Backward velocity
    if (keysPressed['left']) vy = 0.3;      // Left velocity
    if (keysPressed['right']) vy = -0.3;    // Right velocity
    if (keysPressed['turn_l']) wz = 0.6;    // Turn left rate
    if (keysPressed['turn_r']) wz = -0.6;   // Turn right rate
    if (keysPressed['stop']) {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
    }

    // Emit commands
    socket.emit('teleop_cmd', { vx, vy, wz });
}

function updateKeypadUI() {
    // Sync keypad button glows with key states
    document.getElementById('btn-forward').className = keysPressed['forward'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-backward').className = keysPressed['backward'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-left').className = keysPressed['left'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-right').className = keysPressed['right'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-turn-l').className = keysPressed['turn_l'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-turn-r').className = keysPressed['turn_r'] ? 'pad-btn active' : 'pad-btn';
    document.getElementById('btn-stop').className = keysPressed['stop'] ? 'pad-btn stop-btn active' : 'pad-btn stop-btn';
}

// Bind D-pad Buttons Mouse/Touch Events
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

// Stop button mouse click
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

// Recording & Session Management
btnRecordStart.addEventListener('click', () => {
    fetch('/api/record/start', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if (data.success) {
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
            if (data.is_recording) {
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

// Run initial status check
checkRecordingStatus();
