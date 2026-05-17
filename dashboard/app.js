// --- State ---
const state = {
    connected: false,
    currentPos: 0,
    targetPos: 0,
    firmwareTarget: 0,
    velSetpoint: 0,
    accSetpoint: 0,
    jogMode: 'COARSE',
    vel: 0,
    acc: 0,
    pwm: 0,
    mode: 'STOPPED',
    sysMode: 'BASE',
    fault: 'NONE',
    joystick: false,
    modbus: 'IDLE',
    ghost: false,
    prox: false,
    estop: false,
    gripper_ud: 0,
    gripper_co: 0,
    override: false,
    waypoints: [],
    seqActive: false,
    seqLoop: false,
    currentWaypointIdx: -1,
};

let tuningSynced = false;

// --- Charts & Visualizer ---
const visualizer        = new RobotVisualizer('robot-visualizer');
const gripperVisualizer = new GripperVisualizer('gripper-visualizer');
/* Constructor: (id, color, liveMin, liveMax, tuneMin, tuneMax, absInTuning) */
/* Constructor: (id, color, liveMin, liveMax, tuneMin, tuneMax, absInTuning) */
const chartPos = new TelemetryChart('chart-pos', 'cyan',    -360,  360,   0, 400, true);
const chartVel = new TelemetryChart('chart-vel', 'magenta', -100,  100,   0, 100, false);
const chartAcc = new TelemetryChart('chart-acc', 'yellow',  -200,  200);

// --- UI Elements ---
const connectBtn      = document.getElementById('connect-btn');
const estopBtn        = document.getElementById('estop-btn');
const statusIndicator = document.getElementById('serial-status');
const logContent      = document.getElementById('log-content');
const statMode        = document.getElementById('stat-mode');
const statFault       = document.getElementById('stat-fault');
const statJoy         = document.getElementById('stat-joy');
const statModbus      = document.getElementById('stat-modbus');
const statSysmode     = document.getElementById('stat-sysmode');
const statJogmode     = document.getElementById('stat-jogmode');
const ioProx          = document.getElementById('io-prox');
const ioEstop         = document.getElementById('io-estop');
const inputTarget     = document.getElementById('input-target');
const rangeTarget     = document.getElementById('range-target');
const btnGripperUD    = document.getElementById('btn-gripper-ud');
const btnGripperCO    = document.getElementById('btn-gripper-co');
const btnOverride     = document.getElementById('btn-override');

// --- Serial ---
let port = null;
let reader = null;
let inputBuffer = "";

async function connectSerial(existingPort = null) {
    try {
        port = existingPort || await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });

        state.connected = true;
        updateUI();
        log("Connected to serial port.");
        sendCommand("CMD:PING=1");

        const textDecoder = new TextDecoderStream();
        port.readable.pipeTo(textDecoder.writable);
        reader = textDecoder.readable.getReader();
        readLoop();
    } catch (err) {
        log("Connection failed: " + err.message, "error");
        state.connected = false;
        updateUI();
    }
}

async function disconnectSerial() {
    try {
        if (reader) { await reader.cancel(); reader = null; }
        if (port)   { await port.close();    port   = null; }
    } catch (e) { /* ignore */ }
    state.connected = false;
    inputBuffer = "";
    updateUI();
    log("Disconnected.");
}

async function readLoop() {
    while (true) {
        try {
            const { value, done } = await reader.read();
            if (done) { log("Port closed."); break; }

            inputBuffer += value;

            if (inputBuffer.length > 1000) {
                const lastDollar = inputBuffer.lastIndexOf('$');
                inputBuffer = lastDollar !== -1 ? inputBuffer.substring(lastDollar) : "";
            }

            let s = inputBuffer.indexOf('$');
            let e = inputBuffer.indexOf('*');
            while (s !== -1 && e !== -1 && e > s) {
                processPacket(inputBuffer.substring(s + 1, e));
                inputBuffer = inputBuffer.substring(e + 1);
                s = inputBuffer.indexOf('$');
                e = inputBuffer.indexOf('*');
            }
        } catch (err) {
            log("Read error: " + err.message, "error");
            break;
        }
    }
    state.connected = false;
    updateUI();
}

let _dbgPacketCount = 0;

function processPacket(packet) {
    if (_dbgPacketCount < 5) {
        log("RAW PKT: " + packet);
        _dbgPacketCount++;
    }
    packet.split(',').forEach(pair => {
        const idx = pair.indexOf(':');
        if (idx === -1) return;
        const key = pair.substring(0, idx).trim();
        const val = pair.substring(idx + 1).trim();
        const num = parseFloat(val);
        const safeNum = isNaN(num) ? 0 : num;

        switch (key) {
            case 'POS':   state.currentPos = safeNum; break;
            case 'VEL':   state.vel        = safeNum; break;
            case 'ACC':   state.acc        = safeNum; break;
            case 'TAR':   state.firmwareTarget  = safeNum; break;
            case 'VSET':  state.velSetpoint     = safeNum; break;
            case 'ASET':  state.accSetpoint     = safeNum; break;
            case 'PWM':   state.pwm        = safeNum; break;
            case 'MODE':  state.mode       = decodeMode(val); break;
            case 'SYSM':  state.sysMode    = val === '1' ? 'JOYSTICK' : 'BASE'; break;
            case 'JOGM':  state.jogMode    = val === '1' ? 'FINE' : 'COARSE'; break;
            case 'JOY':   state.joystick   = val === '1'; break;
            case 'ESTOP': state.estop      = val === '1'; break;
            case 'FAULT': state.fault      = decodeFault(val); break;
            case 'PROX':  state.prox       = val !== '1'; break;
            case 'GHOST': state.ghost      = val === '1'; break;
            case 'GUD':   state.gripper_ud = val === '1'; break;
            case 'GCO':   state.gripper_co = val === '1'; break;
            case 'PKP':   syncTuning('input-pos-kp',    val); break;
            case 'PKI':   syncTuning('input-pos-ki',    val); break;
            case 'PKD':   syncTuning('input-pos-kd',    val); break;
            case 'SKP':   syncTuning('input-speed-kp',  val); break;
            case 'SKI':   syncTuning('input-speed-ki',  val); break;
            case 'SKD':   syncTuning('input-speed-kd',  val); break;
            case 'SKF':   syncTuning('input-speed-kf',  val); break;
            case 'VMAX':  syncTuning('input-move-coarse', val); break;
            case 'AMAX':  syncTuning('input-max-accel',   val); break;
            case 'STEPC': syncTuning('input-step-coarse', val); break;
            case 'STEPF': syncTuning('input-step-fine',   val); break;
            case 'JOGF':  syncTuning('input-jog-fine',    val); break;
            case 'HOMES': syncTuning('input-home-speed',  val); break;
            case 'MINP':  syncTuning('input-min-pwm',     val); break;
        }
    });

    lockTuningAfterSync();
    updateUI();
    visualizer.update(state.currentPos, state.firmwareTarget);
    gripperVisualizer.update(!!state.gripper_ud, !state.gripper_co);
    chartPos.addData(state.currentPos);
    chartPos.addTarget(state.firmwareTarget);
    chartVel.addData(state.vel);
    chartVel.addTarget(state.velSetpoint);
    chartAcc.addData(state.acc);
    chartAcc.addTarget(state.accSetpoint);
    checkGhostStart();
    tuningTick();
}

async function sendCommand(cmd) {
    if (!state.connected || !port) return;
    const writer = port.writable.getWriter();
    writer.write(new TextEncoder().encode(`$${cmd}*`));
    writer.releaseLock();
    log("Sent: $" + cmd + "*");
}

// --- UI ---
function updateUI() {
    statusIndicator.innerText   = state.connected ? "Connected" : "Disconnected";
    statusIndicator.className   = "status-indicator " + (state.connected ? "connected" : "disconnected");
    connectBtn.innerText        = state.connected ? "Disconnect" : "Connect Robot";

    statMode.innerText   = state.mode;
    statFault.innerText  = state.fault;
    statFault.className  = "value " + (state.fault === 'NONE' ? "ok" : "error");
    statJoy.innerText    = state.joystick ? "CONNECTED" : "OFFLINE";
    statJoy.className    = "value " + (state.joystick ? "ok" : "");
    statModbus.innerText = state.modbus;
    statSysmode.innerText = state.sysMode;
    statSysmode.className = "value " + (state.sysMode === 'JOYSTICK' ? "ok" : "");
    statJogmode.innerText = state.jogMode;
    statJogmode.className = "value " + (state.jogMode === 'FINE' ? "ok" : "");

    const bjm = document.getElementById('btn-jog-mode');
    if (bjm) {
        bjm.innerText = 'Jog: ' + state.jogMode;
        bjm.className = 'toggle-btn ' + (state.jogMode === 'FINE' ? 'active' : '');
    }

    document.getElementById('stat-pwm').innerText = (isNaN(state.pwm) ? '0.0' : state.pwm.toFixed(1)) + "%";
    ioProx.className  = "io-item " + (state.prox  ? "active" : "");
    ioEstop.className = "io-item " + (state.estop ? "active" : "");

    btnGripperUD.innerText  = "Gripper: " + (state.gripper_ud ? "DOWN" : "UP");
    btnGripperUD.className  = "toggle-btn " + (state.gripper_ud ? "active" : "active-green");
    btnGripperCO.innerText  = "Claw: "    + (state.gripper_co ? "CLOSED" : "OPEN");
    btnGripperCO.className  = "toggle-btn " + (state.gripper_co ? "active" : "active-green");
    btnOverride.innerText   = "Override: " + (state.override ? "ON" : "OFF");
    btnOverride.className   = "toggle-btn warning " + (state.override ? "active" : "");
    estopBtn.innerText      = state.estop ? "CLEAR FAULT / RESUME" : "EMERGENCY STOP";
    estopBtn.className      = state.estop ? "danger-btn active" : "danger-btn";

    const btnSysMode = document.getElementById('btn-sys-mode');
    btnSysMode.innerText = "Mode: " + state.sysMode;
    btnSysMode.className = "toggle-btn " + (state.sysMode === 'JOYSTICK' ? "active" : "");

}

function log(msg, type = "info") {
    const time = new Date().toLocaleTimeString([], { hour12: false });
    const div  = document.createElement('div');
    div.innerHTML = `<span style="color:#555">[${time}]</span> ${msg}`;
    if (type === "error") div.style.color = "#ff3131";
    logContent.prepend(div);
}

let _tuningReceivedFromFirmware = false;

function syncTuning(id, val) {
    const el = document.getElementById(id);
    if (!el || tuningSynced) return;
    const num = parseFloat(val);
    if (!isNaN(num)) el.value = num;
    _tuningReceivedFromFirmware = true;
}

function lockTuningAfterSync() {
    if (_tuningReceivedFromFirmware) tuningSynced = true;
}

function decodeMode(val) {
    return ['STOPPED','SPEED','POSITION','AUTOTUNE_P','AUTOTUNE_S','TEST','GHOST','HOMING'][parseInt(val)] || 'UNKNOWN';
}

function decodeFault(val) {
    const bits = parseInt(val);
    if (bits === 0) return 'NONE';
    const f = [];
    if (bits & 0x01) f.push('STALL');
    if (bits & 0x02) f.push('ENCODER');
    if (bits & 0x04) f.push('JOY_LOST');
    if (bits & 0x08) f.push('OVER_ROT');
    return f.join(' | ');
}

// --- Event Listeners ---
connectBtn.addEventListener('click', () => {
    if (state.connected) disconnectSerial();
    else connectSerial();
});

estopBtn.addEventListener('click', () => {
    if (state.estop) {
        sendCommand("CMD:CLEAR");
        state.estop = false;
        state.fault = 'NONE';
    } else {
        sendCommand("CMD:ESTOP=1");
        state.estop = true;
        state.vel = 0;
        state.acc = 0;
        chartVel.history = [];
        chartAcc.history = [];
        chartVel.draw();
        chartAcc.draw();
    }
    updateUI();
});

inputTarget.addEventListener('input', e => {
    state.targetPos = parseFloat(e.target.value);
    rangeTarget.value = state.targetPos;
});

rangeTarget.addEventListener('input', e => {
    state.targetPos = parseFloat(e.target.value);
    inputTarget.value = state.targetPos;
});

document.getElementById('btn-negate-target').addEventListener('click', () => {
    const v = parseFloat(inputTarget.value) || 0;
    const neg = -v;
    inputTarget.value = neg;
    rangeTarget.value = neg;
    state.targetPos = neg;
});

document.getElementById('btn-set-target').addEventListener('click', () => {
    const target = parseFloat(inputTarget.value);
    if (!isNaN(target)) {
        sendCommand(`SET:TARGET=${target}`);
        if (tuningMode) { tuningState = 'IDLE'; tuningArmRun(); setMetricsStatus('ARMED (Move) — Waiting for motor motion...', ''); }
    }
});

btnGripperUD.addEventListener('click', () => {
    state.gripper_ud = 1 - state.gripper_ud;
    sendCommand(state.gripper_ud ? "CMD:GRIP_DN" : "CMD:GRIP_UP");
    updateUI();
});

btnGripperCO.addEventListener('click', () => {
    state.gripper_co = 1 - state.gripper_co;
    sendCommand(state.gripper_co ? "CMD:GRIP_CLOSE" : "CMD:GRIP_OPEN");
    updateUI();
});

document.getElementById('btn-sys-mode').addEventListener('click', () => {
    sendCommand('CMD:TOGGLE_MODE');
    log('Sent TOGGLE_MODE — LPUART1 baud will switch. If going to BASE, dashboard will disconnect.');
});

document.getElementById('btn-jog-mode').addEventListener('click', () => {
    const newJog = state.jogMode === 'COARSE' ? 1 : 0;
    sendCommand(`SET:JOG_MODE=${newJog}`);
    state.jogMode = newJog === 1 ? 'FINE' : 'COARSE';
    updateUI();
});

btnOverride.addEventListener('click', () => {
    state.override = !state.override;
    sendCommand(`SET:OVERRIDE=${state.override ? 1 : 0}`);
    updateUI();
});

// --- Fault Modal ---
const FAULT_INFO = {
    'STALL':    { name: 'Motor Stalled',    desc: 'PWM high but rotor not moving for 2s. Check obstruction or wiring.' },
    'ENCODER':  { name: 'Encoder Error',    desc: 'Encoder signal lost or phase inverted. Check encoder cable.' },
    'JOY_LOST': { name: 'Joystick Lost',    desc: 'ESP32 joystick reported disconnected.' },
    'OVER_ROT': { name: 'Over-Rotation',    desc: 'Exceeded soft limit (720° from home). Wire-twist protection.' },
};

function refreshFaultModal() {
    const list = document.getElementById('modal-fault-list');
    list.innerHTML = '';
    if (state.fault === 'NONE') {
        list.innerHTML = '<div style="color:var(--accent-green);">No active faults.</div>';
        return;
    }
    state.fault.split(' | ').forEach(f => {
        const info = FAULT_INFO[f] || { name: f, desc: 'Unknown fault.' };
        const div = document.createElement('div');
        div.className = 'fault-item';
        div.innerHTML = `<div class="fault-name">${info.name}</div><div class="fault-desc">${info.desc}</div>`;
        list.appendChild(div);
    });
}

/* Step Response Config modal */
function openMetricsConfig() {
    document.getElementById('cfg-vel-threshold').value = TUNE_VEL_THRESHOLD;
    document.getElementById('cfg-pos-settle').value    = TUNE_POS_SETTLE_DEG;
    document.getElementById('cfg-vel-settle').value    = TUNE_VEL_SETTLE_RPM;
    document.getElementById('cfg-settle-ms').value     = TUNE_SETTLE_MS;
    document.getElementById('metrics-config-modal').classList.remove('hidden');
}
function saveMetricsConfig() {
    TUNE_VEL_THRESHOLD  = parseFloat(document.getElementById('cfg-vel-threshold').value) || 3.0;
    TUNE_POS_SETTLE_DEG = parseFloat(document.getElementById('cfg-pos-settle').value)    || 2.0;
    TUNE_VEL_SETTLE_RPM = parseFloat(document.getElementById('cfg-vel-settle').value)    || 3.0;
    TUNE_SETTLE_MS      = parseInt(document.getElementById('cfg-settle-ms').value)       || 3000;
    localStorage.setItem('metricsCfg', JSON.stringify({
        velThresh: TUNE_VEL_THRESHOLD,
        posSettle: TUNE_POS_SETTLE_DEG,
        velSettle: TUNE_VEL_SETTLE_RPM,
        settleMs:  TUNE_SETTLE_MS,
    }));
    document.getElementById('metrics-config-modal').classList.add('hidden');
    log(`Settle config saved: vel>${TUNE_VEL_THRESHOLD} pos<${TUNE_POS_SETTLE_DEG}° vel<${TUNE_VEL_SETTLE_RPM} hold=${TUNE_SETTLE_MS}ms`);
}
document.getElementById('btn-metrics-config').addEventListener('click', openMetricsConfig);
document.getElementById('btn-close-metrics-modal').addEventListener('click', () => {
    document.getElementById('metrics-config-modal').classList.add('hidden');
});
document.getElementById('btn-save-metrics-cfg').addEventListener('click', saveMetricsConfig);
document.getElementById('metrics-config-modal').addEventListener('click', e => {
    if (e.target.id === 'metrics-config-modal') e.target.classList.add('hidden');
});

document.getElementById('btn-fault-info').addEventListener('click', () => {
    refreshFaultModal();
    document.getElementById('fault-modal').classList.remove('hidden');
});
document.getElementById('btn-close-modal').addEventListener('click', () => {
    document.getElementById('fault-modal').classList.add('hidden');
});
document.getElementById('fault-modal').addEventListener('click', (e) => {
    if (e.target.id === 'fault-modal') e.target.classList.add('hidden');
});

const SAFETY_TOGGLES = {
    'chk-safe-stall':   'SAFE_STALL',
    'chk-safe-encoder': 'SAFE_ENCODER',
    'chk-safe-overrot': 'SAFE_OVERROT',
    'chk-safe-joy':     'SAFE_JOY',
};
Object.entries(SAFETY_TOGGLES).forEach(([id, key]) => {
    document.getElementById(id).addEventListener('change', e => {
        sendCommand(`SET:${key}=${e.target.checked ? 1 : 0}`);
    });
});

document.getElementById('btn-fine-home').addEventListener('click', () => {
    sendCommand("CMD:HOME");
    log("Homing sequence triggered");
});

document.getElementById('btn-go-home').addEventListener('click', () => {
    sendCommand("SET:TARGET=0");
    log("Returning to home (0°)");
    if (tuningMode) { tuningState = 'IDLE'; tuningArmRun(); setMetricsStatus('ARMED (Go Home) — Waiting for motor motion...', ''); }
});

document.getElementById('send-tuning-btn').addEventListener('click', () => {
    const params = {
        'SPEED_KP':   'input-speed-kp',
        'SPEED_KI':   'input-speed-ki',
        'SPEED_KD':   'input-speed-kd',
        'SPEED_KF':   'input-speed-kf',
        'POS_KP':     'input-pos-kp',
        'POS_KI':     'input-pos-ki',
        'POS_KD':     'input-pos-kd',
        'MAX_ACCEL':  'input-max-accel',
        'MIN_PWM':    'input-min-pwm',
        'HOME_SPEED': 'input-home-speed',
        'JOG_FINE':   'input-jog-fine',
        'MOVE_COARSE':'input-move-coarse',
        'STEP_COARSE':'input-step-coarse',
        'STEP_FINE':  'input-step-fine',
    };
    for (const [key, id] of Object.entries(params)) {
        sendCommand(`SET:${key}=${document.getElementById(id).value}`);
    }
});

document.querySelectorAll('.tuning-scroll-area input').forEach(el => {
    el.addEventListener('mousedown', () => tuningSynced = true);
});

// --- Simulation ---
let simActive = false;
let simInterval;
document.getElementById('sim-btn').addEventListener('click', () => {
    simActive = !simActive;
    const btn = document.getElementById('sim-btn');
    btn.innerText  = simActive ? "Stop Sim" : "Simulate Data";
    btn.className  = simActive ? "primary-btn" : "secondary-btn";

    if (simActive) {
        let t = 0;
        simInterval = setInterval(() => {
            t += 0.05;
            state.currentPos = Math.sin(t) * 180;
            state.targetPos  = Math.cos(t * 0.5) * 180;
            state.vel        = Math.sin(t * 1.2) * 60;
            state.acc        = Math.cos(t * 2) * 150;
            state.pwm        = Math.abs(Math.sin(t)) * 100;
            state.mode       = "SIMULATION";

            updateUI();
            visualizer.update(state.currentPos, state.firmwareTarget);
            chartPos.addData(state.currentPos);
            chartVel.addData(state.vel);
            chartAcc.addData(state.acc);
        }, 50);
    } else {
        clearInterval(simInterval);
    }
});

// --- Tuning Mode State Machine ---
let tuningMode = false;
let tuningState = 'IDLE'; // IDLE | ARMED | CAPTURING | SETTLING | DONE
let tuningRunCount = 0;
let tuningStartTime = 0;
let tuningStartPos = 0;
let tuningDir = 1;
let tuningMoveTarget = 0;
let tuningSettleStart = 0;
let tuningMetricsHistory = [];

/* Mutable thresholds (configurable via gear icon on Step Response card).
 * Persisted in localStorage so they survive page reloads. */
let TUNE_VEL_THRESHOLD  = 3.0;   // RPM — motor considered moving
let TUNE_POS_SETTLE_DEG = 2.0;   // degrees — within target = settling
let TUNE_VEL_SETTLE_RPM = 3.0;   // RPM — velocity settled
let TUNE_SETTLE_MS      = 3000;  // ms to confirm settled

(function loadMetricsCfg() {
    try {
        const c = JSON.parse(localStorage.getItem('metricsCfg') || '{}');
        if (c.velThresh   !== undefined) TUNE_VEL_THRESHOLD  = c.velThresh;
        if (c.posSettle   !== undefined) TUNE_POS_SETTLE_DEG = c.posSettle;
        if (c.velSettle   !== undefined) TUNE_VEL_SETTLE_RPM = c.velSettle;
        if (c.settleMs    !== undefined) TUNE_SETTLE_MS      = c.settleMs;
    } catch (e) {}
})();

// Vel peak tracking
let tuningVelPeak = 0;
let tuningVelSetPeak = 0;

function tuningArmRun() {
    if (!tuningMode) return;
    tuningState = 'ARMED';
    tuningMoveTarget = state.firmwareTarget;
    tuningStartPos = state.currentPos;
    tuningVelPeak = 0;
    tuningVelSetPeak = 0;
    chartPos.startRun(tuningMoveTarget);
    chartVel.startRun(tuningMoveTarget);
    chartAcc.startRun(tuningMoveTarget);
    setMetricsStatus('ARMED — Waiting for motor to move...', '');
}

function tuningTick() {
    if (!tuningMode || tuningState === 'IDLE' || tuningState === 'DONE') return;

    const vel = Math.abs(state.vel);
    const posErr = Math.abs(state.currentPos - state.firmwareTarget);
    const now = Date.now();

    // Feed data to tuning charts
    if (tuningState === 'CAPTURING' || tuningState === 'SETTLING') {
        const relPos    = (state.currentPos - tuningStartPos) * tuningDir;
        const relVel    = state.vel * tuningDir;
        const relAcc    = state.acc * tuningDir;
        const relVSet   = state.velSetpoint * tuningDir;
        const relASet   = state.accSetpoint * tuningDir;
        chartPos.addRunPoint(relPos, (tuningMoveTarget - tuningStartPos) * tuningDir);
        chartVel.addRunPoint(relVel, relVSet);
        chartAcc.addRunPoint(relAcc, relASet);
        tuningVelPeak = Math.max(tuningVelPeak, Math.abs(state.vel));
        tuningVelSetPeak = Math.max(tuningVelSetPeak, Math.abs(state.velSetpoint));
    }

    switch (tuningState) {
        case 'ARMED':
            if (vel > TUNE_VEL_THRESHOLD) {
                tuningState = 'CAPTURING';
                tuningStartTime = now;
                tuningMoveTarget = state.firmwareTarget;
                tuningStartPos = state.currentPos;
                tuningDir = (tuningMoveTarget - tuningStartPos) >= 0 ? 1 : -1;
                // Reset chart runs so t=0 begins at motor start (absolute/relative target)
                const relTarget = (tuningMoveTarget - tuningStartPos) * tuningDir;
                chartPos.startRun(relTarget);
                chartVel.startRun(0);
                chartAcc.startRun(0);
                setMetricsStatus('CAPTURING...', 'capturing');
            }
            break;

        case 'CAPTURING':
            if (posErr < TUNE_POS_SETTLE_DEG && vel < TUNE_VEL_SETTLE_RPM) {
                tuningState = 'SETTLING';
                tuningSettleStart = now;
                setMetricsStatus('SETTLING...', 'settling');
            }
            break;

        case 'SETTLING':
            if (posErr > TUNE_POS_SETTLE_DEG || vel > TUNE_VEL_SETTLE_RPM) {
                tuningState = 'CAPTURING'; // bounced out
                setMetricsStatus('CAPTURING...', 'capturing');
            } else if (now - tuningSettleStart >= TUNE_SETTLE_MS) {
                tuningFinalize(now);
            }
            break;
    }
}

function tuningFinalize(now) {
    tuningState = 'DONE';

    // Compute pos metrics from captured data
    const posRun  = chartPos.currentRun  || (chartPos.tuningRuns.length  ? null : null);
    const velRunData = chartVel.currentRun;

    // Settle time = elapsed from motor start to entering settle zone
    const settleTimeSec = (tuningSettleStart - tuningStartTime) / 1000;

    // Pos overshoot: chart vals are already in relative space (target = relTarget)
    let posOver = 0;
    const relTarget = (tuningMoveTarget - tuningStartPos) * tuningDir;
    if (chartPos.currentRun && Math.abs(relTarget) > 0.5) {
        const peak = chartPos.currentRun.vals.reduce((m, v) => Math.max(m, v), 0);
        posOver = Math.max(0, ((peak - relTarget) / relTarget) * 100);
    }

    // Vel overshoot: how much actual vel exceeded setpoint vel
    let velOver = 0;
    if (tuningVelSetPeak > 1) velOver = Math.max(0, ((tuningVelPeak - tuningVelSetPeak) / tuningVelSetPeak) * 100);

    // Vel settling: when vel dropped back within threshold
    let velSettleSec = settleTimeSec;
    if (chartVel.currentRun) {
        const vTimes = chartVel.currentRun.times;
        const vVals  = chartVel.currentRun.vals;
        let lastOutIdx = -1;
        vVals.forEach((v, i) => { if (Math.abs(v) > TUNE_VEL_SETTLE_RPM) lastOutIdx = i; });
        if (lastOutIdx >= 0) velSettleSec = vTimes[lastOutIdx];
    }

    chartPos.finalizeRun(settleTimeSec, posOver);
    chartVel.finalizeRun(velSettleSec, velOver);
    chartAcc.finalizeRun(settleTimeSec, 0);

    tuningRunCount++;
    const entry = { run: tuningRunCount, posSettle: settleTimeSec, posOver, velSettle: velSettleSec, velOver };
    tuningMetricsHistory.push(entry);
    if (tuningMetricsHistory.length > 10) tuningMetricsHistory.shift();

    // Update metrics panel
    document.getElementById('m-pos-settle').innerText = settleTimeSec.toFixed(2) + 's';
    document.getElementById('m-pos-over').innerText   = posOver.toFixed(1) + '%';
    document.getElementById('m-vel-settle').innerText = velSettleSec.toFixed(2) + 's';
    document.getElementById('m-vel-over').innerText   = velOver.toFixed(1) + '%';
    renderMetricsHistory();

    setMetricsStatus(`Run #${tuningRunCount} done — Pos settle: ${settleTimeSec.toFixed(2)}s  OS: ${posOver.toFixed(1)}% — Trigger next move to capture again`, 'done');
}

function setMetricsStatus(msg, cls) {
    const el = document.getElementById('metrics-status');
    el.innerText = msg;
    el.className = 'metrics-status-bar ' + cls;
}

function renderMetricsHistory() {
    const tbody = document.getElementById('metrics-history-body');
    tbody.innerHTML = '';
    [...tuningMetricsHistory].reverse().forEach(e => {
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${e.run}</td><td>${e.posSettle.toFixed(2)}</td><td>${e.posOver.toFixed(1)}%</td><td>${e.velSettle.toFixed(2)}</td><td>${e.velOver.toFixed(1)}%</td>`;
        tbody.appendChild(tr);
    });
}

// Toggle button
document.getElementById('mode-toggle-btn').addEventListener('click', () => {
    tuningMode = !tuningMode;
    const btn = document.getElementById('mode-toggle-btn');
    btn.innerText = tuningMode ? 'Tuning' : 'Live';
    btn.className = tuningMode ? 'secondary-btn tuning-active' : 'secondary-btn';
    chartPos.setMode(tuningMode);
    chartVel.setMode(tuningMode);
    chartAcc.setMode(tuningMode);
    tuningState = 'IDLE';
    // Toggle Path Sequencer / Metrics card visibility
    document.querySelector('.path-card').style.display    = tuningMode ? 'none' : '';
    document.querySelector('.metrics-card').style.display = tuningMode ? '' : 'none';
    if (tuningMode) {
        setMetricsStatus('IDLE — Send Move / Go Home / Ghost start to begin capture', '');
    } else {
        setMetricsStatus('IDLE — Switch to Tuning Mode and move motor', '');
    }
});

// Default: hide metrics card on load (Live mode default)
document.addEventListener('DOMContentLoaded', () => {
    const m = document.querySelector('.metrics-card');
    if (m) m.style.display = 'none';
});

// --- Gripper Config ---
const gripperConfig = {
    enabled: false,
    mode: 'sim',
    delays: { open: 600, close: 600, up: 600, down: 600 },
    realTimeout: 5000,
};

document.getElementById('chk-use-gripper').addEventListener('change', e => {
    gripperConfig.enabled = e.target.checked;
});

document.getElementById('btn-gripper-settings').addEventListener('click', () => {
    document.getElementById('gripper-modal').classList.remove('hidden');
});
document.getElementById('btn-close-gripper-modal').addEventListener('click', () => {
    document.getElementById('gripper-modal').classList.add('hidden');
    gripperConfig.delays.open  = parseInt(document.getElementById('delay-open').value)  || 600;
    gripperConfig.delays.close = parseInt(document.getElementById('delay-close').value) || 600;
    gripperConfig.delays.up    = parseInt(document.getElementById('delay-up').value)    || 600;
    gripperConfig.delays.down  = parseInt(document.getElementById('delay-down').value)  || 600;
});
document.getElementById('gripper-modal').addEventListener('click', e => {
    if (e.target.id === 'gripper-modal') {
        e.target.classList.add('hidden');
        gripperConfig.delays.open  = parseInt(document.getElementById('delay-open').value)  || 600;
        gripperConfig.delays.close = parseInt(document.getElementById('delay-close').value) || 600;
        gripperConfig.delays.up    = parseInt(document.getElementById('delay-up').value)    || 600;
        gripperConfig.delays.down  = parseInt(document.getElementById('delay-down').value)  || 600;
    }
});
document.querySelectorAll('input[name="gripper-mode"]').forEach(radio => {
    radio.addEventListener('change', e => {
        gripperConfig.mode = e.target.value;
        document.getElementById('sim-delays-section').style.display  = gripperConfig.mode === 'sim'  ? '' : 'none';
        document.getElementById('real-sensor-info').style.display    = gripperConfig.mode === 'real' ? '' : 'none';
    });
});

// --- Gripper Sequence Helpers ---
function msDelay(ms) { return new Promise(r => setTimeout(r, ms)); }

function waitForGripperState(predicate, timeout) {
    return new Promise(resolve => {
        if (predicate()) { resolve(); return; }
        const t0 = Date.now();
        const iv = setInterval(() => {
            if (predicate() || Date.now() - t0 >= timeout) { clearInterval(iv); resolve(); }
        }, 50);
    });
}

async function gripperStep(cmd, confirmFn, simMs) {
    sendCommand(cmd);
    if (gripperConfig.mode === 'real') {
        await waitForGripperState(confirmFn, gripperConfig.realTimeout);
    } else {
        await msDelay(simMs);
    }
    gripperVisualizer.update(!!state.gripper_ud, !state.gripper_co);
}

async function runGripperPick() {
    await gripperStep('CMD:GRIP_OPEN',  () => !state.gripper_co, gripperConfig.delays.open);
    await gripperStep('CMD:GRIP_DN',    () =>  state.gripper_ud, gripperConfig.delays.down);
    await gripperStep('CMD:GRIP_CLOSE', () =>  state.gripper_co, gripperConfig.delays.close);
    await gripperStep('CMD:GRIP_UP',    () => !state.gripper_ud, gripperConfig.delays.up);
}

async function runGripperPlace() {
    await gripperStep('CMD:GRIP_DN',    () =>  state.gripper_ud, gripperConfig.delays.down);
    await gripperStep('CMD:GRIP_OPEN',  () => !state.gripper_co, gripperConfig.delays.open);
    await gripperStep('CMD:GRIP_UP',    () => !state.gripper_ud, gripperConfig.delays.up);
    await gripperStep('CMD:GRIP_CLOSE', () =>  state.gripper_co, gripperConfig.delays.close);
}

// --- Path Sequencer (Live mode) ---
const waypointList = document.getElementById('waypoint-list');
const btnRunSeq    = document.getElementById('btn-run-seq');
const btnLoopSeq   = document.getElementById('btn-loop-seq');
const inputNewWaypoint = document.getElementById('input-new-waypoint');

document.getElementById('btn-add-waypoint').addEventListener('click', () => {
    const val = parseFloat(inputNewWaypoint.value);
    if (!isNaN(val)) {
        state.waypoints.push(val);
        inputNewWaypoint.value = "";
        renderWaypoints();
    }
});

btnRunSeq.addEventListener('click', () => {
    state.seqActive = !state.seqActive;
    btnRunSeq.innerText = state.seqActive ? 'Stop' : 'Run';
    btnRunSeq.className = 'toggle-btn ' + (state.seqActive ? 'active' : '');
    if (state.seqActive) {
        state.currentWaypointIdx = 0;
        executeNextWaypoint();
    }
});

btnLoopSeq.addEventListener('click', () => {
    state.seqLoop = !state.seqLoop;
    btnLoopSeq.className = 'toggle-btn ' + (state.seqLoop ? 'active' : '');
});

function renderWaypoints() {
    waypointList.innerHTML = '';
    state.waypoints.forEach((wp, i) => {
        const li = document.createElement('li');
        if (i === state.currentWaypointIdx) li.className = 'active';
        li.innerHTML = `<span>${wp}°</span><span class="remove-waypoint" onclick="removeWaypoint(${i})">×</span>`;
        waypointList.appendChild(li);
    });
}

window.removeWaypoint = (i) => { state.waypoints.splice(i, 1); renderWaypoints(); };

async function executeNextWaypoint() {
    if (!state.seqActive || state.waypoints.length === 0) return;
    const target = state.waypoints[state.currentWaypointIdx];
    sendCommand(`SET:TARGET=${target}`);
    renderWaypoints();

    const movDelay = parseFloat(document.getElementById('input-seq-delay').value) * 1000;
    await msDelay(movDelay);
    if (!state.seqActive) return;

    if (gripperConfig.enabled) {
        const isPick = (state.currentWaypointIdx % 2 === 0);
        if (isPick) await runGripperPick();
        else        await runGripperPlace();
    }
    if (!state.seqActive) return;

    state.currentWaypointIdx++;
    if (state.currentWaypointIdx >= state.waypoints.length) {
        if (state.seqLoop) {
            state.currentWaypointIdx = 0;
        } else {
            state.seqActive = false;
            state.currentWaypointIdx = -1;
            btnRunSeq.innerText = 'Run';
            btnRunSeq.className = 'toggle-btn';
            renderWaypoints();
            return;
        }
    }
    executeNextWaypoint();
}

// Auto-arm capture on ghost-mode rising edge
let _lastGhost = false;
function checkGhostStart() {
    if (tuningMode && !_lastGhost && state.ghost && tuningState === 'IDLE') {
        tuningArmRun();
        setMetricsStatus('ARMED (Ghost replay) — Waiting for motor motion...', '');
    }
    _lastGhost = state.ghost;
}

// --- Default tuning values (mirrors params.h) ---
const DEFAULTS = {
    'input-speed-kp':    1.0,
    'input-speed-ki':    2.0,
    'input-speed-kd':    0.0,
    'input-speed-kf':    0.0,
    'input-pos-kp':      1.2,
    'input-pos-ki':      0.05,
    'input-pos-kd':      0.1,
    'input-max-accel':   2000,
    'input-min-pwm':     0.0,
    'input-home-speed':  30,
    'input-jog-fine':    10,
    'input-move-coarse': 100,
    'input-step-coarse': 10,
    'input-step-fine':   1.0,
};

function applyDefaults() {
    for (const [id, val] of Object.entries(DEFAULTS)) {
        const el = document.getElementById(id);
        if (el) el.value = val;
    }
}

// --- Auto-reconnect on page load ---
window.addEventListener('DOMContentLoaded', async () => {
    applyDefaults();
    gripperVisualizer.resize();
    if (!("serial" in navigator)) {
        log("Web Serial API not supported. Use Chrome/Edge over HTTPS or localhost.", "error");
        return;
    }

    const ports = await navigator.serial.getPorts();
    if (ports.length > 0) {
        log("Auto-connecting to previously granted port...");
        connectSerial(ports[0]);
    }
});

updateUI();
