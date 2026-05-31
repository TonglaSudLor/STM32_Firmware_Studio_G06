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
    positionUnknown: false,
    gripper_ud: 0,
    gripper_co: 0,
    current: 0,
    override: false,
    // Reed switch inputs (from firmware RSUP/RSDN/RSCL/RSOP)
    reed_up: -1,    // -1 = no data yet
    reed_down: -1,
    reed_close: -1,
    reed_open: -1,
    baseAlive: false,
    pnpState: 0,
    connectTime: null,
    waypoints: [],
    seqActive: false,
    seqLoop: false,
    currentWaypointIdx: -1,

    // ZVD Input Shaper
    shaperEnabled: false,
    shaperOmegaN: 12.13,
    shaperZeta: 0.041,

    // Kalman filter telemetry
    kfEnabled: false,
    kfTheta: 0,         // deg
    kfOmega: 0,         // RPM
    kfTau: 0,           // N·m
    kfIa: 0,            // A
    kfInnov: 0,
    kfP00: 0, kfP11: 0, kfP22: 0, kfP33: 0,
    kfSanityTheta: 0,   // deg, open-loop model
    kfSanityOmega: 0,   // RPM
    kfSanityShow: false,
};

/* Mission Readiness — enforces Connect → HW Self-Test → Home before motion.
 * Declared early so telemetry/UI callbacks can touch it safely. */
const readiness = {
    diag: 'idle',          // idle | running | pass | fail
    home: 'no',            // no | homing | done | fail
    diagText: 'Not run yet.',
    diagHint: '',
    homeText: 'Not homed.',
};
let _faultLatched = false; // set when a fault/E-Stop invalidates the home

/* Motion controls disabled until the readiness gate passes (or Override is on).
 * Declared early: updateUI() runs at load and reads this through the gate. */
const RD_GATED_IDS = [
    'btn-set-target', 'input-target', 'range-target', 'btn-negate-target',
    'btn-go-home', 'btn-run-seq', 'btn-loop-seq', 'btn-sine-toggle',
];

let tuningSynced = false;
// Hoisted because renderPosLoopBtn() (called during script eval) references it.
let tuningMode = false;

/* Shared map of firmware SET-key → input element id. Used by Apply All,
 * autosave draft, and named profiles so the field list lives in one place. */
const TUNING_FIELDS = {
    'SPEED_KP':   'input-speed-kp',
    'SPEED_KI':   'input-speed-ki',
    'SPEED_KD':   'input-speed-kd',
    'K_VFF':      'input-k-vff',
    'K_AFF':      'input-k-aff',
    'K_TFF':      'input-k-tff',
    'V_MAX_RAD':  'input-vmax-rad',
    'A_MAX_RAD':  'input-amax-rad',
    'J_MAX_RAD':  'input-jmax-rad',
    'POS_KP':     'input-pos-kp',
    'POS_KI':     'input-pos-ki',
    'POS_KD':     'input-pos-kd',
    'MIN_PWM':    'input-min-pwm',
    'HOME_SPEED': 'input-home-speed',
    'HOME_OFFSET':'input-home-offset',
    'JOG_FINE':   'input-jog-fine',
    'STEP_COARSE':'input-step-coarse',
    'STEP_FINE':  'input-step-fine',
    'SHPWN':      'input-shaper-wn',
    'SHPZT':      'input-shaper-zeta',
    'KF_Q_THETA': 'input-kf-q-theta',
    'KF_Q_OMEGA': 'input-kf-q-omega',
    'KF_Q_TAU':   'input-kf-q-tau',
    'KF_Q_I':     'input-kf-q-i',
    'KF_R':       'input-kf-r',
};
const TUNING_DRAFT_KEY    = 'tuningDraft';
const TUNING_PROFILES_KEY = 'tuningProfiles';

// --- Charts & Visualizer ---
const visualizer = new RobotVisualizer('robot-visualizer');
const gripperVisualizer = new GripperVisualizer('gripper-visualizer');
/* Constructor: (id, color, liveMin, liveMax, tuneMin, tuneMax, absInTuning) */
/* Constructor: (id, color, liveMin, liveMax, tuneMin, tuneMax, absInTuning) */
const chartPos = new TelemetryChart('chart-pos', 'cyan', -360, 360, 0, 400, true);
const chartVel = new TelemetryChart('chart-vel', 'magenta', -100, 100, 0, 100, false);
const chartAcc = new TelemetryChart('chart-acc', 'yellow', -200, 200);

// --- UI Elements ---
const connectBtn = document.getElementById('connect-btn');
const estopBtn = document.getElementById('estop-btn');
const statusIndicator = document.getElementById('serial-status');
const logContent = document.getElementById('log-content');
const statMode = document.getElementById('stat-mode');
const statFault = document.getElementById('stat-fault');
const statModbus = document.getElementById('stat-modbus');
const statSysmode = document.getElementById('stat-sysmode');
const statJogmode = document.getElementById('stat-jogmode');
const ioProx = document.getElementById('io-prox');
const ioEstop = document.getElementById('io-estop');
const inputTarget = document.getElementById('input-target');
const rangeTarget = document.getElementById('range-target');
const btnGripperUD = document.getElementById('btn-gripper-ud');
const btnGripperCO = document.getElementById('btn-gripper-co');
const btnOverride = document.getElementById('btn-override');

// --- Serial ---
let port = null;
let reader = null;
let inputBuffer = "";

async function connectSerial(existingPort = null) {
    try {
        port = existingPort || await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });

        state.connected = true;
        state.connectTime = Date.now();
        state.reed_up = -1; state.reed_down = -1;
        state.reed_close = -1; state.reed_open = -1;
        updateUI();
        log("Connected to serial port.");
        sendCommand("CMD:PING=1");
        pushSafetyConfig();
        if (typeof onSerialConnected === 'function') onSerialConnected();

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
        if (port) { await port.close(); port = null; }
    } catch (e) { /* ignore */ }
    state.connected = false;
    inputBuffer = "";
    if (typeof resetReadiness === 'function') resetReadiness();
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

            // Display plain-text firmware debug output ([DIAG], [SAFETY], [SYSTEM], etc.)
            // Only shows lines that start with '[' to ignore high-frequency CSV/numeric data.
            const dollarPos = inputBuffer.indexOf('$');
            const plainPart = dollarPos !== -1 ? inputBuffer.substring(0, dollarPos) : inputBuffer;
            const lastNewline = plainPart.lastIndexOf('\n');
            if (lastNewline !== -1) {
                plainPart.substring(0, lastNewline).split('\n').forEach(line => {
                    const clean = line.replace(/\r/g, '').trim();
                    if (clean.startsWith('[')) { log('STM: ' + clean, 'info'); handleFirmwareLine(clean); }
                });
                inputBuffer = inputBuffer.substring(lastNewline + 1);
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
let _hbPktCount = 0, _hbPktWindowStart = Date.now();

function processPacket(packet) {
    if (_dbgPacketCount < 5) {
        log("RAW PKT: " + packet);
        _dbgPacketCount++;
    }
    _hbPktCount++;
    const _now = Date.now();
    if (_now - _hbPktWindowStart >= 1000) {
        const rate = Math.round(_hbPktCount * 1000 / (_now - _hbPktWindowStart));
        const rateEl = document.getElementById('hb-rate');
        if (rateEl) rateEl.textContent = rate + ' Hz';
        _hbPktCount = 0;
        _hbPktWindowStart = _now;
    }
    packet.split(',').forEach(pair => {
        const idx = pair.indexOf(':');
        if (idx === -1) return;
        const key = pair.substring(0, idx).trim();
        const val = pair.substring(idx + 1).trim();
        const num = parseFloat(val);
        const safeNum = isNaN(num) ? 0 : num;

        switch (key) {
            case 'POS': state.currentPos = safeNum; break;
            case 'VEL': state.vel = safeNum; break;
            case 'ACC': state.acc = safeNum; break;
            case 'TAR': state.firmwareTarget = safeNum; break;
            case 'VSET': state.velSetpoint = safeNum; break;
            case 'ASET': state.accSetpoint = safeNum; break;
            case 'PWM': state.pwm = safeNum; break;
            case 'MODE':
                state.mode = decodeMode(val);
                if (state.mode === 'HOMING' && readiness.home !== 'done') {
                    readiness.home = 'homing';
                    readiness.homeText = 'Homing in progress…';
                }
                break;
            case 'SYSM': state.sysMode = val === '1' ? 'JOYSTICK' : 'BASE'; break;
            case 'JOGM': state.jogMode = val === '1' ? 'FINE' : 'COARSE'; break;
            case 'JOY': state.joystick = val === '1'; break;
            case 'ESTOP': {
                const wasEstop = state.estop;
                state.estop = val === '1';
                if (state.estop && !wasEstop) onFaultOrEstop();
                break;
            }
            case 'FAULT': {
                const prevFault = state.fault;
                state.fault = decodeFault(val);
                if (state.fault !== 'NONE' && (prevFault === 'NONE' || prevFault === undefined)) onFaultOrEstop();
                break;
            }
            case 'PROX': state.prox = val !== '1'; break;
            case 'GHOST': state.ghost = val === '1'; break;
            case 'GUP': state.gripper_ud = val === '0'; break;  // GUP=1 → UP relay ON → gripper is UP (ud=false=not-down)
            case 'GDN': state.gripper_co = val === '1'; break;  // GDN=1 → claw CLOSE relay ON → claw is CLOSED
            case 'CURR': state.current = safeNum; break;
            case 'RSUP': state.reed_up    = parseInt(val); break;
            case 'RSDN': state.reed_down  = parseInt(val); break;
            case 'RSCL': state.reed_close = parseInt(val); break;
            case 'RSOP': state.reed_open  = parseInt(val); break;
            case 'PUNK': {
                const wasUnknown = state.positionUnknown;
                state.positionUnknown = val === '1';
                if (state.positionUnknown && !wasUnknown && readiness.home === 'done') {
                    readiness.home = 'fail';
                    readiness.homeText = 'Position unknown after relay cut — re-home required.';
                }
                break;
            }
            case 'BSALV': state.baseAlive = val === '1'; break;
            case 'PNPS': state.pnpState = parseInt(val); break;
            case 'KFEN': state.kfEnabled = val === '1'; break;
            case 'KTH': state.kfTheta = safeNum; break;
            case 'KOM': state.kfOmega = safeNum; break;
            case 'KTL': state.kfTau = safeNum; break;
            case 'KIA': state.kfIa = safeNum; break;
            case 'KIV': state.kfInnov = safeNum; break;
            case 'KP00': state.kfP00 = safeNum; break;
            case 'KP11': state.kfP11 = safeNum; break;
            case 'KP22': state.kfP22 = safeNum; break;
            case 'KP33': state.kfP33 = safeNum; break;
            case 'KSTH': state.kfSanityTheta = safeNum; break;
            case 'KSOM': state.kfSanityOmega = safeNum; break;
            case 'PKP': syncTuning('input-pos-kp', val); break;
            case 'PKI': syncTuning('input-pos-ki', val); break;
            case 'PKD': syncTuning('input-pos-kd', val); break;
            case 'SKP': syncTuning('input-speed-kp', val); break;
            case 'SKI': syncTuning('input-speed-ki', val); break;
            case 'SKD': syncTuning('input-speed-kd', val); break;
            case 'KVFF': syncTuning('input-k-vff', val); break;
            case 'KAFF': syncTuning('input-k-aff', val); break;
            case 'KTFF': syncTuning('input-k-tff', val); break;
            case 'JMAX': syncTuning('input-max-jerk-rpm', val); /* legacy RPM/s² field if present */
                /* Also convert to rad/s³ for the SI input box */
                syncTuning('input-jmax-rad', (parseFloat(val) || 0) * (2 * Math.PI / 60)); break;
            case 'VMAX': syncTuning('input-vmax-rad', (parseFloat(val) || 0) * (2 * Math.PI / 60)); break;
            case 'AMAX': syncTuning('input-amax-rad', (parseFloat(val) || 0) * (2 * Math.PI / 60)); break;
            case 'STEPC': syncTuning('input-step-coarse', val); break;
            case 'STEPF': syncTuning('input-step-fine', val); break;
            case 'JOGF': syncTuning('input-jog-fine', val); break;
            case 'HOMES': syncTuning('input-home-speed', val); break;
            case 'HOFS':  syncTuning('input-home-offset', val); break;
            case 'MINP': syncTuning('input-min-pwm', val); break;
            case 'PLOOP': {
                // Ignore for 2 s after the user clicked, otherwise a stale
                // in-flight slow telemetry packet would overwrite the click.
                if (Date.now() - posLoopUserClickedAt < 2000) break;
                const newState = (val > 0.5);
                if (newState !== posLoopEnabled) {
                    posLoopEnabled = newState;
                    renderPosLoopBtn();
                }
                break;
            }
            case 'SHPEN': {
                if (Date.now() - shaperUserClickedAt < 2000) break;
                const newShaper = val === '1';
                if (newShaper !== state.shaperEnabled) {
                    state.shaperEnabled = newShaper;
                    updateShaperBtn();
                }
                break;
            }
            case 'SHPWN':
                state.shaperOmegaN = safeNum;
                syncTuning('input-shaper-wn', val);
                updateShaperDelayInfo();
                break;
            case 'SHPZT':
                state.shaperZeta = safeNum;
                syncTuning('input-shaper-zeta', val);
                updateShaperDelayInfo();
                break;
        }
    });

    lockTuningAfterSync();
    updateUI();
    visualizer.update(state.currentPos, state.firmwareTarget);
    chartPos.addData(state.currentPos);
    chartPos.addTarget(state.firmwareTarget);
    chartVel.addData(state.vel);
    chartVel.addTarget(state.velSetpoint);
    chartAcc.addData(state.acc);
    chartAcc.addTarget(state.accSetpoint);

    // KF overlays (always pushed; rendering on each chart is unconditional
    // because the chart class hides empty arrays). When KF is disabled the
    // estimate is just the open-loop integration with corrections.
    chartPos.addEstimate(state.kfTheta);
    chartVel.addEstimate(state.kfOmega);
    if (state.kfSanityShow) {
        chartPos.addSanity(state.kfSanityTheta);
        chartVel.addSanity(state.kfSanityOmega);
    }
    updateKalmanCard();
    checkGhostStart();
    tuningTick();
    if (window.TestSuite) window.TestSuite.onPacket(state, packet);
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
    statusIndicator.innerText = state.connected ? "Connected" : "Disconnected";
    statusIndicator.className = "status-indicator " + (state.connected ? "connected" : "disconnected");
    connectBtn.innerText = state.connected ? "Disconnect" : "Connect Robot";

    statMode.innerText = state.mode;
    statMode.className = "value " + ({
        POSITION: 'ok', GHOST: 'ok', VELOCITY: 'ok',
        HOMING: 'warn', STOPPED: ''
    }[state.mode] ?? '');
    statFault.innerText = state.fault;
    statFault.className = "value " + (state.fault === 'NONE' ? "ok" : "error");
    const baseText = state.sysMode === 'BASE' ? (state.baseAlive ? 'ALIVE' : 'TIMEOUT') : '--';
    statModbus.innerText = baseText;
    statModbus.className = 'value ' + (state.baseAlive ? 'ok' : (state.sysMode === 'BASE' ? 'error' : ''));
    const statPnp = document.getElementById('stat-pnp');
    if (statPnp) {
        statPnp.innerText = decodePnP(state.pnpState);
        statPnp.className = 'value ' + (state.pnpState > 0 ? 'warn' : '');
    }
    statSysmode.innerText = state.sysMode;
    statSysmode.className = "value " + (state.sysMode === 'JOYSTICK' ? "ok" : "");
    statJogmode.innerText = state.jogMode;
    statJogmode.className = "value " + (state.jogMode === 'FINE' ? "ok" : "");

    const bjm = document.getElementById('btn-jog-mode');
    if (bjm) {
        bjm.innerText = 'Jog: ' + state.jogMode;
        bjm.className = 'toggle-btn ' + (state.jogMode === 'FINE' ? 'active' : '');
    }

    ioProx.className = "io-item " + (state.prox ? "active" : "");
    ioEstop.className = "io-item " + (state.estop ? "active" : "");

    btnGripperUD.innerText = "Gripper: " + (state.gripper_ud ? "DOWN" : "UP");
    btnGripperUD.className = "toggle-btn " + (state.gripper_ud ? "active" : "active-green");
    btnGripperCO.innerText = "Claw: " + (state.gripper_co ? "CLOSED" : "OPEN");
    btnGripperCO.className = "toggle-btn " + (state.gripper_co ? "active" : "active-green");
    gripperVisualizer.update(!!state.gripper_ud, !state.gripper_co);
    btnOverride.innerText = "Override: " + (state.override ? "ON" : "OFF");
    btnOverride.className = "toggle-btn warning " + (state.override ? "active" : "");
    estopBtn.innerText = state.estop ? "CLEAR FAULT / RESUME" : "EMERGENCY STOP";
    estopBtn.className = state.estop ? "danger-btn active" : "danger-btn";

    const btnSysMode = document.getElementById('btn-sys-mode');
    btnSysMode.innerText = "Mode: " + state.sysMode;
    btnSysMode.className = "toggle-btn " + (state.sysMode === 'JOYSTICK' ? "active" : "");

    if (typeof updateReadinessUI === 'function') updateReadinessUI();
    updateHeartbeat();
    // Keep the fault modal live while it's open
    if (!document.getElementById('fault-modal')?.classList.contains('hidden')) refreshFaultModal();
}

function setHbTile(id, status, val) {
    const tile = document.getElementById(id);
    if (!tile) return;
    tile.dataset.status = status;
    const v = tile.querySelector('.hb-val');
    if (v) v.textContent = val;
}

function updateHeartbeat() {
    if (!state.connected) {
        document.querySelectorAll('.hb-tile').forEach(t => {
            t.dataset.status = 'idle';
            const v = t.querySelector('.hb-val');
            if (v) v.textContent = '--';
        });
        const rateEl = document.getElementById('hb-rate');
        if (rateEl) rateEl.textContent = '-- Hz';
        return;
    }

    const f = state.fault;

    // ENCODER
    setHbTile('hbt-encoder',
        f.includes('ENCODER') ? 'fault' : 'ok',
        state.vel.toFixed(0) + 'RPM'
    );

    // CURRENT SENSOR (WCS1800 ADC)
    setHbTile('hbt-current', 'ok', state.current.toFixed(2) + 'A');

    // PROXIMITY SENSOR
    setHbTile('hbt-prox',
        f.includes('PROX_LOST') ? 'fault' : 'ok',
        state.prox ? 'CLR' : 'TRIP'
    );

    // JOYSTICK (ESP32)
    setHbTile('hbt-joy',
        f.includes('JOY_LOST') ? 'fault' : (state.joystick ? 'ok' : 'warn'),
        state.joystick ? 'ON' : 'OFF'
    );

    // E-STOP
    setHbTile('hbt-estop',
        state.estop ? 'fault' : 'ok',
        state.estop ? 'ACTV' : 'CLR'
    );

    // MOTOR / H-BRIDGE
    const motorFault = f.includes('STALL') || f.includes('OVER_ROT');
    setHbTile('hbt-motor',
        motorFault ? 'fault' : 'ok',
        Math.abs(state.pwm).toFixed(0) + '%'
    );

    // GRIP (vertical) — reed switches RSUP / RSDN
    {
        const up = state.reed_up, dn = state.reed_down;
        let gs, gv;
        if (up < 0) { gs = 'idle'; gv = '--'; }           // no data yet
        else if (up === 1 && dn === 0) { gs = 'ok';   gv = 'UP'; }
        else if (up === 0 && dn === 1) { gs = 'ok';   gv = 'DOWN'; }
        else if (up === 0 && dn === 0) { gs = 'warn'; gv = 'TRNST'; }
        else                            { gs = 'fault'; gv = 'ERR'; }  // both 1 = impossible
        setHbTile('hbt-grip', gs, gv);
    }

    // CLAW — reed switches RSCL / RSOP
    {
        const cl = state.reed_close, op = state.reed_open;
        let cs, cv;
        if (cl < 0) { cs = 'idle'; cv = '--'; }
        else if (cl === 1 && op === 0) { cs = 'ok';   cv = 'CLSD'; }
        else if (cl === 0 && op === 1) { cs = 'ok';   cv = 'OPEN'; }
        else if (cl === 0 && op === 0) { cs = 'warn'; cv = 'TRNST'; }
        else                            { cs = 'fault'; cv = 'ERR'; }
        setHbTile('hbt-claw', cs, cv);
    }

    // ── STM32 HEALTH TILES ──

    // LINK — serial packet rate quality
    const rateEl = document.getElementById('hb-rate');
    const rateStr = rateEl ? rateEl.textContent : '-- Hz';
    const rateNum = parseInt(rateStr);
    setHbTile('hbt-link',
        isNaN(rateNum) ? 'idle' : (rateNum >= 40 ? 'ok' : rateNum >= 10 ? 'warn' : 'fault'),
        rateStr
    );

    // CTRL — active control loop mode
    const ctrlModes = { 'STOPPED': 'warn', 'POSITION': 'ok', 'VELOCITY': 'ok', 'GHOST': 'ok', 'HOMING': 'warn' };
    setHbTile('hbt-ctrl',
        ctrlModes[state.mode] || 'ok',
        (state.mode || 'STP').substring(0, 4)
    );

    // SYS — system mode (BASE / JOYSTICK)
    setHbTile('hbt-sys', 'ok',
        state.sysMode === 'BASE' ? 'BASE' : 'JOY'
    );

    // BASE — Modbus master heartbeat + P&P task
    {
        const pnpLabel = decodePnP(state.pnpState).substring(0, 4);
        const baseStatus = state.baseAlive ? 'ok' : (state.sysMode === 'BASE' ? 'fault' : 'warn');
        setHbTile('hbt-base',
            baseStatus,
            state.pnpState > 0 ? pnpLabel : (state.baseAlive ? 'IDLE' : 'OFF')
        );
    }
}

function log(msg, type = "info") {
    const time = new Date().toLocaleTimeString([], { hour12: false });
    const div = document.createElement('div');
    div.innerHTML = `<span style="color:#555">[${time}]</span> ${msg}`;
    if (type === "error") div.style.color = "#ff3131";
    logContent.appendChild(div);
    // Keep at most 200 entries
    while (logContent.children.length > 200) logContent.removeChild(logContent.firstChild);
    logContent.scrollTop = logContent.scrollHeight;
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
    if (_tuningReceivedFromFirmware) {
        tuningSynced = true;
        // Fields now mirror firmware values, so clear the unsynced indicator.
        if (typeof markTuningSynced === 'function') markTuningSynced();
    }
}

function decodeMode(val) {
    return ['STOPPED', 'SPEED', 'POSITION', 'AUTOTUNE_P', 'AUTOTUNE_S', 'TEST', 'GHOST', 'HOMING'][parseInt(val)] || 'UNKNOWN';
}

function decodePnP(n) {
    return ['IDLE','→PICK','WAIT♦','OPEN','↓','CLOSE','↑','→PLACE','WAIT♦','↓','OPEN','↑','CLOSE','NEXT','DONE'][n] || '??';
}

function decodeFault(val) {
    const bits = parseInt(val);
    if (bits === 0) return 'NONE';
    const f = [];
    if (bits & 0x001) f.push('STALL');
    if (bits & 0x002) f.push('ENCODER');
    if (bits & 0x004) f.push('JOY_LOST');
    if (bits & 0x008) f.push('OVER_ROT');
    if (bits & 0x010) f.push('ESTOP_HW');
    if (bits & 0x020) f.push('PROX_LOST');
    if (bits & 0x040) f.push('ESTOP_JOY');
    if (bits & 0x080) f.push('ESTOP_DASH');
    if (bits & 0x100) f.push('ESTOP_MBUS');
    if (bits & 0x200) f.push('STARTUP_ESTOP');
    if (bits & 0x400) f.push('OVERCURRENT');
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
        chartVel.clearLiveData();
        chartAcc.clearLiveData();
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

document.querySelectorAll('.mo-step-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        const step = parseFloat(btn.dataset.step);
        const cur = parseFloat(inputTarget.value) || 0;
        const next = Math.round((cur + step) * 10) / 10;
        inputTarget.value = next;
        rangeTarget.value = Math.max(-180, Math.min(180, next));
        state.targetPos = next;
        sendCommand(`SET:TARGET=${next}`);
        if (tuningMode) { tuningState = 'IDLE'; tuningArmRun(); setMetricsStatus('ARMED (Move) — Waiting for motor motion...', ''); }
    });
});

let _lastGripUDClick = 0;
let _lastGripCOClick = 0;
const GRIP_DEBOUNCE_MS = 500;

btnGripperUD.addEventListener('click', () => {
    const now = Date.now();
    if (now - _lastGripUDClick < GRIP_DEBOUNCE_MS) return;
    _lastGripUDClick = now;
    state.gripper_ud = 1 - state.gripper_ud;
    sendCommand(state.gripper_ud ? "CMD:GRIP_DN" : "CMD:GRIP_UP");
    updateUI();
});

btnGripperCO.addEventListener('click', () => {
    const now = Date.now();
    if (now - _lastGripCOClick < GRIP_DEBOUNCE_MS) return;
    _lastGripCOClick = now;
    state.gripper_co = 1 - state.gripper_co;
    sendCommand(state.gripper_co ? "CMD:CLAW_CLOSE" : "CMD:CLAW_OPEN");
    updateUI();
});

document.getElementById('btn-sys-mode').addEventListener('click', () => {
    sendCommand('CMD:TOGGLE_MODE');
    log('Sent TOGGLE_MODE — LPUART1 baud will switch. If going to BASE, dashboard will disconnect.');
});

// Position loop enable/disable — bypass outer loop to tune velocity loop alone
const btnPosLoop = document.getElementById('btn-pos-loop');
let posLoopEnabled = true;
let posLoopUserClickedAt = 0;   // timestamp of last user click on the Loop button
const POS_INPUT_IDS = ['input-pos-kp', 'input-pos-ki', 'input-pos-kd'];
let sineActive = false;
let sineCapturing = false;
let sineStartMs = 0;
let prevTuningModeBeforeLoopOff = null;  // remembers the user's mode so we can restore it

// Snapshot the default tuning-axis scaling so we can swap it while the
// position loop is OFF (sine wave needs a symmetric, signed Y range).
const TUNE_SCALE_DEFAULT = {
    pos: { min: chartPos.tuningMinVal, max: chartPos.tuningMaxVal, abs: chartPos.absInTuning },
    vel: { min: chartVel.tuningMinVal, max: chartVel.tuningMaxVal, abs: chartVel.absInTuning },
    acc: { min: chartAcc.tuningMinVal, max: chartAcc.tuningMaxVal, abs: chartAcc.absInTuning },
};
function applyTuningChartScale(loopOff) {
    if (loopOff) {
        chartPos.tuningMinVal = -360; chartPos.tuningMaxVal = 360; chartPos.absInTuning = false;
        chartVel.tuningMinVal = -60; chartVel.tuningMaxVal = 60; chartVel.absInTuning = false;
        // acc has no tuning override -> nothing to do
    } else {
        chartPos.tuningMinVal = TUNE_SCALE_DEFAULT.pos.min;
        chartPos.tuningMaxVal = TUNE_SCALE_DEFAULT.pos.max;
        chartPos.absInTuning = TUNE_SCALE_DEFAULT.pos.abs;
        chartVel.tuningMinVal = TUNE_SCALE_DEFAULT.vel.min;
        chartVel.tuningMaxVal = TUNE_SCALE_DEFAULT.vel.max;
        chartVel.absInTuning = TUNE_SCALE_DEFAULT.vel.abs;
    }
    chartPos.draw(); chartVel.draw(); chartAcc.draw();
}
function renderPosLoopBtn() {
    btnPosLoop.textContent = posLoopEnabled ? 'Loop: ON' : 'Loop: OFF';
    btnPosLoop.classList.toggle('active', posLoopEnabled);
    btnPosLoop.classList.toggle('danger', !posLoopEnabled);

    // Gray out + lock the position-PID inputs when the outer loop is bypassed
    const section = btnPosLoop.closest('.tl-sec') || btnPosLoop.closest('.tuning-section');
    if (section) section.classList.toggle('disabled', !posLoopEnabled);
    POS_INPUT_IDS.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = !posLoopEnabled;
    });

    // Swap Manual Override target row <-> sine wave generator
    const posGrp = document.getElementById('target-pos-group');
    const sineGrp = document.getElementById('sine-gen-group');
    if (posGrp && sineGrp) {
        posGrp.style.display = posLoopEnabled ? '' : 'none';
        sineGrp.style.display = posLoopEnabled ? 'none' : '';
    }
    // Hide Gripper/Claw/Mode/Jog/Override during tuning — keep only Fine/Go Home
    document.querySelectorAll('.manual-extra').forEach(el => {
        el.style.display = posLoopEnabled ? '' : 'none';
    });

    // Loop OFF: switch the telemetry view to Tuning so the preview/run lives
    // in the right chart mode. Loop ON: restore the user's prior mode.
    if (!posLoopEnabled) {
        if (prevTuningModeBeforeLoopOff === null) prevTuningModeBeforeLoopOff = tuningMode;
        if (!tuningMode) setTuningMode(true);
    } else if (prevTuningModeBeforeLoopOff !== null) {
        if (tuningMode !== prevTuningModeBeforeLoopOff) setTuningMode(prevTuningModeBeforeLoopOff);
        prevTuningModeBeforeLoopOff = null;
    }

    // Tuning Y-axis: when loop is OFF the signal can go negative (sine wave),
    // so use symmetric ranges and disable abs-folding on the pos chart.
    applyTuningChartScale(!posLoopEnabled);

    // Sine preview on velocity chart (only when loop OFF and sine not yet started)
    if (!posLoopEnabled && !sineActive) {
        refreshSinePreview();
    } else {
        if (chartVel.clearPreviewSine) chartVel.clearPreviewSine();
    }

    // Killing the loop also stops any running sine — make sure firmware agrees
    if (posLoopEnabled && sineActive) stopSine();

    // Loop OFF → show Kalman card in place of Step Response Metrics.
    // Loop ON  → restore Step Response Metrics (only when in Tuning mode).
    const kf = document.querySelector('.kalman-card');
    const metrics = document.querySelector('.metrics-card');
    if (kf && metrics) {
        if (!posLoopEnabled) {
            kf.style.display = '';
            metrics.style.display = 'none';
        } else {
            kf.style.display = 'none';
            metrics.style.display = tuningMode ? '' : 'none';
        }
    }
}

function refreshSinePreview() {
    if (!chartVel.setPreviewSine) return;
    const amp = parseFloat(document.getElementById('input-sine-amp').value) || 0;
    const freq = parseFloat(document.getElementById('input-sine-freq').value) || 0;
    chartVel.setPreviewSine(amp, freq);
}

function stopSine() {
    sineActive = false;
    sendCommand('SET:SINE_EN=0');
    // End the capture and lock the trace in tuning-run history
    if (sineCapturing) {
        sineCapturing = false;
        chartVel.finalizeRun(null, null);
    }
    chartPos.scrollWindowSec = null;
    chartVel.scrollWindowSec = null;
    chartAcc.scrollWindowSec = null;
    const btn = document.getElementById('btn-sine-toggle');
    if (btn) { btn.textContent = 'Start Sine'; btn.classList.remove('danger'); }
    // Restore preview overlay (loop is still OFF)
    if (!posLoopEnabled) refreshSinePreview();
}
function startSine() {
    const amp = parseFloat(document.getElementById('input-sine-amp').value) || 0;
    const freq = parseFloat(document.getElementById('input-sine-freq').value) || 0;
    sendCommand(`SET:SINE_AMP=${amp}`);
    sendCommand(`SET:SINE_FREQ=${freq}`);
    sendCommand('SET:SINE_EN=1');
    sineActive = true;
    if (chartVel.clearPreviewSine) chartVel.clearPreviewSine();  // captured trace takes over
    // Begin a tuning-mode capture window on the velocity chart.
    // Use a scrolling 10-second window so the wave keeps the same
    // horizontal density as the run grows.
    sineStartMs = Date.now();
    chartVel.startRun(0);
    chartPos.scrollWindowSec = 10;
    chartVel.scrollWindowSec = 10;
    chartAcc.scrollWindowSec = 10;
    sineCapturing = true;
    const btn = document.getElementById('btn-sine-toggle');
    if (btn) { btn.textContent = 'Stop Sine'; btn.classList.add('danger'); }
    log(`Sine generator ON — ${amp} RPM @ ${freq} Hz`);
}
document.getElementById('btn-sine-toggle').addEventListener('click', () => {
    sineActive ? stopSine() : startSine();
});
// Live-update amp/freq + refresh preview as you type
['input-sine-amp', 'input-sine-freq'].forEach(id => {
    const el = document.getElementById(id);
    el.addEventListener('input', () => { if (!sineActive && !posLoopEnabled) refreshSinePreview(); });
    el.addEventListener('change', () => {
        if (sineActive) {
            const k = id === 'input-sine-amp' ? 'SINE_AMP' : 'SINE_FREQ';
            sendCommand(`SET:${k}=${parseFloat(el.value) || 0}`);
        }
    });
});
btnPosLoop.addEventListener('click', () => {
    posLoopEnabled = !posLoopEnabled;
    posLoopUserClickedAt = Date.now();
    sendCommand(`SET:POS_LOOP=${posLoopEnabled ? 1 : 0}`);
    renderPosLoopBtn();
    log(`Position loop ${posLoopEnabled ? 'ENABLED' : 'BYPASSED — velocity loop tuning mode'}`);
});
renderPosLoopBtn();

// --- ZVD Input Shaper controls ---
let shaperUserClickedAt = 0;

function updateShaperBtn() {
    const btn = document.getElementById('btn-shaper-toggle');
    if (!btn) return;
    btn.textContent = state.shaperEnabled ? 'Shaper: ON' : 'Shaper: OFF';
    btn.classList.toggle('active', state.shaperEnabled);
    btn.classList.toggle('warning', !state.shaperEnabled);
}

function updateShaperDelayInfo() {
    const el = document.getElementById('shaper-info-delay');
    if (!el) return;
    const wn   = parseFloat(document.getElementById('input-shaper-wn').value)   || 12.13;
    const zeta = parseFloat(document.getElementById('input-shaper-zeta').value) || 0.041;
    if (wn <= 0 || zeta < 0 || zeta >= 1) { el.textContent = 'Invalid parameters'; return; }
    const sqrt1z2 = Math.sqrt(1 - zeta * zeta);
    const Td = Math.PI / (wn * sqrt1z2);
    const N  = Math.round(Td * 100);
    el.textContent = `N ≈ ${N} ticks · 2N ≈ ${2 * N} ticks  (Td = ${Td.toFixed(3)} s)`;
}

document.getElementById('btn-shaper-toggle').addEventListener('click', () => {
    state.shaperEnabled = !state.shaperEnabled;
    shaperUserClickedAt = Date.now();
    sendCommand(`SET:SHPEN=${state.shaperEnabled ? 1 : 0}`);
    updateShaperBtn();
    log(`ZVD Input Shaper ${state.shaperEnabled ? 'ENABLED' : 'DISABLED'}`);
});

document.getElementById('input-shaper-wn').addEventListener('input', updateShaperDelayInfo);
document.getElementById('input-shaper-zeta').addEventListener('input', updateShaperDelayInfo);

document.getElementById('input-shaper-wn').addEventListener('change', e => {
    const v = parseFloat(e.target.value);
    if (!isNaN(v) && v > 0) {
        sendCommand(`SET:SHPWN=${v}`);
        state.shaperOmegaN = v;
        updateShaperDelayInfo();
    }
});
document.getElementById('input-shaper-zeta').addEventListener('change', e => {
    const v = parseFloat(e.target.value);
    if (!isNaN(v) && v >= 0 && v < 1) {
        sendCommand(`SET:SHPZT=${v}`);
        state.shaperZeta = v;
        updateShaperDelayInfo();
    }
});

// Seed the delay info display on load
updateShaperDelayInfo();

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
    /* Automatic faults (gated by safety_config) */
    'STALL':        { name: 'Motor Stalled',         desc: 'PWM high but rotor not moving for the stall sustain time. Check for obstruction, mechanical jam, or wiring fault.' },
    'ENCODER':      { name: 'Encoder Error',          desc: 'Encoder signal lost or phase inverted. Check encoder cable and connector.' },
    'JOY_LOST':     { name: 'Joystick Lost',          desc: 'ESP32 joystick link timed out or reported disconnected. Check joystick power and Bluetooth.' },
    'OVER_ROT':     { name: 'Over-Rotation',          desc: 'Absolute position exceeded the soft limit. Virtual hard-stop active. Sustained >1 s escalates to E-Stop.' },
    /* User / external e-stop sources */
    'ESTOP_HW':     { name: 'E-Stop: Physical Button',desc: 'Hardware E-Stop button held down. Release button, then clear fault.' },
    'PROX_LOST':    { name: 'Proximity Sensor Lost',  desc: 'Proximity sensor input opened. Check sensor wiring and power.' },
    'ESTOP_JOY':    { name: 'E-Stop: Joystick',       desc: 'E-Stop triggered by joystick safety button (P/X). Source: ESP32.' },
    'ESTOP_DASH':   { name: 'E-Stop: Dashboard',      desc: 'EMERGENCY STOP button clicked on the dashboard.' },
    'ESTOP_MBUS':   { name: 'E-Stop: Modbus',         desc: 'Modbus register 0x25 soft-stop request from Base System.' },
    'STARTUP_ESTOP':{ name: 'Startup E-Stop Latch',   desc: 'Power-on safety latch. Run Self-Test (DIAG) to release. Prevents motion before hardware is verified.' },
    'OVERCURRENT':  { name: 'Overcurrent Trip',       desc: 'Motor current exceeded the hardware limit for 50 ms. Motor relay opened. Check for jam, short circuit, or overload. Clear fault once safe.' },
};

function refreshFaultModal() {
    // ── Status strip ───────────────────────────────────────────────────────
    const posEl  = document.getElementById('modal-pos-status');
    const wdgEl  = document.getElementById('modal-wdg-status');
    const currEl = document.getElementById('modal-curr-status');
    if (wdgEl) {
        wdgEl.textContent = state.connected ? 'IWDG Active (15 s)' : 'IWDG: --';
        wdgEl.className   = `modal-status-pill ${state.connected ? 'ok' : 'neutral'}`;
    }
    if (posEl) {
        if (!state.connected) {
            posEl.textContent = 'Position: Disconnected';
            posEl.className   = 'modal-status-pill neutral';
        } else if (state.positionUnknown) {
            posEl.textContent = 'Position UNKNOWN — Re-home required';
            posEl.className   = 'modal-status-pill warn';
        } else {
            posEl.textContent = 'Position KNOWN';
            posEl.className   = 'modal-status-pill ok';
        }
    }
    if (currEl) {
        const amps = typeof state.current === 'number' ? state.current : 0;
        const pct  = Math.min(100, Math.abs(amps) / 15 * 100);
        const cls  = pct > 80 ? 'warn' : (pct > 50 ? 'caution' : 'ok');
        currEl.textContent = state.connected ? `Current: ${amps.toFixed(1)} A` : 'Current: --';
        currEl.className   = `modal-status-pill ${state.connected ? cls : 'neutral'}`;
    }
    // Live current in the overcurrent info block
    const ocCurr = document.getElementById('disp-oc-current');
    if (ocCurr) ocCurr.textContent = state.connected ? (state.current ?? 0).toFixed(2) : '--';

    // ── Active faults ───────────────────────────────────────────────────────
    const list = document.getElementById('modal-fault-list');
    list.innerHTML = '';
    const visibleFaults = (state.fault && state.fault !== 'NONE') ? state.fault.split(' | ') : [];

    // Treat positionUnknown as a fault-level condition
    if (state.positionUnknown) visibleFaults.unshift('_PUNK');

    if (visibleFaults.length === 0) {
        list.innerHTML = '<div style="color:var(--accent-green);">No active faults.</div>';
        return;
    }
    visibleFaults.forEach(f => {
        let info;
        if (f === '_PUNK') {
            info = { name: 'Position Unknown', desc: 'Physical E-Stop cut power to the encoder. Arm may have moved. Run the homing sequence before commanding motion.' };
        } else {
            info = FAULT_INFO[f] || { name: f, desc: 'Unknown fault code.' };
        }
        const div = document.createElement('div');
        div.className = 'fault-item' + (f === '_PUNK' ? ' fault-item-warn' : '');
        div.innerHTML = `<div class="fault-name">${info.name}</div><div class="fault-desc">${info.desc}</div>`;
        list.appendChild(div);
    });
}

/* Step Response Config modal */
function openMetricsConfig() {
    document.getElementById('cfg-vel-threshold').value = TUNE_VEL_THRESHOLD;
    document.getElementById('cfg-pos-settle').value = TUNE_POS_SETTLE_DEG;
    document.getElementById('cfg-vel-settle').value = TUNE_VEL_SETTLE_RPM;
    document.getElementById('cfg-settle-ms').value = TUNE_SETTLE_MS;
    const bandEl = document.getElementById('cfg-settle-band');
    if (bandEl) bandEl.value = TUNE_SETTLE_BAND_PCT;
    document.getElementById('metrics-config-modal').classList.remove('hidden');
}
function saveMetricsConfig() {
    TUNE_VEL_THRESHOLD = parseFloat(document.getElementById('cfg-vel-threshold').value) || 3.0;
    TUNE_POS_SETTLE_DEG = parseFloat(document.getElementById('cfg-pos-settle').value) || 2.0;
    TUNE_VEL_SETTLE_RPM = parseFloat(document.getElementById('cfg-vel-settle').value) || 3.0;
    TUNE_SETTLE_MS = parseInt(document.getElementById('cfg-settle-ms').value) || 3000;
    const bandEl = document.getElementById('cfg-settle-band');
    if (bandEl) TUNE_SETTLE_BAND_PCT = parseFloat(bandEl.value) || 2.0;
    localStorage.setItem('metricsCfg', JSON.stringify({
        velThresh: TUNE_VEL_THRESHOLD,
        posSettle: TUNE_POS_SETTLE_DEG,
        velSettle: TUNE_VEL_SETTLE_RPM,
        settleMs: TUNE_SETTLE_MS,
        settleBand: TUNE_SETTLE_BAND_PCT,
    }));
    document.getElementById('metrics-config-modal').classList.add('hidden');
    log(`Settle config saved: band ±${TUNE_SETTLE_BAND_PCT}% · vel>${TUNE_VEL_THRESHOLD} pos<${TUNE_POS_SETTLE_DEG}° vel<${TUNE_VEL_SETTLE_RPM} hold=${TUNE_SETTLE_MS}ms`);
}
document.getElementById('btn-metrics-config').addEventListener('click', openMetricsConfig);
document.getElementById('btn-close-metrics-modal').addEventListener('click', () => {
    document.getElementById('metrics-config-modal').classList.add('hidden');
});
document.getElementById('btn-save-metrics-cfg').addEventListener('click', saveMetricsConfig);
{
    const csvBtn = document.getElementById('btn-metrics-csv');
    if (csvBtn) csvBtn.addEventListener('click', exportMetricsCsv);
}
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
    'chk-safe-stall':      'SAFE_STALL',
    'chk-safe-encoder':    'SAFE_ENCODER',
    'chk-safe-overrot':    'SAFE_OVERROT',
    'chk-safe-joy':        'SAFE_JOY',
    'chk-safe-phys-estop': 'SAFE_ESTOP',
};
/* Editable numeric thresholds → firmware SET keys. */
const SAFETY_THRESHOLDS = {
    'cfg-stall-pwm': 'STALL_PWM',
    'cfg-stall-vel': 'STALL_VEL',
    'cfg-stall-time': 'STALL_TIME',
    'cfg-stall-err': 'STALL_ERR',
    'cfg-max-rot': 'MAX_ROT',
};

/* Persist toggles + thresholds so they survive page reloads. */
function saveSafetyConfig() {
    const data = { toggles: {}, thresholds: {} };
    Object.keys(SAFETY_TOGGLES).forEach(id => {
        const el = document.getElementById(id); if (el) data.toggles[id] = el.checked;
    });
    Object.keys(SAFETY_THRESHOLDS).forEach(id => {
        const el = document.getElementById(id); if (el) data.thresholds[id] = el.value;
    });
    localStorage.setItem('safetyConfig', JSON.stringify(data));
}

function loadSafetyConfig() {
    let data = {};
    try { data = JSON.parse(localStorage.getItem('safetyConfig') || '{}'); } catch (e) { data = {}; }
    Object.entries(data.toggles || {}).forEach(([id, v]) => {
        const el = document.getElementById(id); if (el) el.checked = v;
    });
    Object.entries(data.thresholds || {}).forEach(([id, v]) => {
        const el = document.getElementById(id); if (el && v !== '' && v != null) el.value = v;
    });
}

/* Re-send the full safety config to the firmware. Called after a connection is
 * established so the robot always matches what the dashboard shows, even if the
 * user unchecked a box / changed a limit while disconnected. */
function pushSafetyConfig() {
    Object.entries(SAFETY_TOGGLES).forEach(([id, key]) => {
        const el = document.getElementById(id); if (el) sendCommand(`SET:${key}=${el.checked ? 1 : 0}`);
    });
    Object.entries(SAFETY_THRESHOLDS).forEach(([id, key]) => {
        const el = document.getElementById(id);
        if (el && el.value !== '') sendCommand(`SET:${key}=${el.value}`);
    });
    log('Safety config pushed to robot.');
}

Object.entries(SAFETY_TOGGLES).forEach(([id, key]) => {
    const el = document.getElementById(id);
    el.addEventListener('change', e => {
        saveSafetyConfig();
        sendCommand(`SET:${key}=${e.target.checked ? 1 : 0}`);
    });
});
Object.entries(SAFETY_THRESHOLDS).forEach(([id, key]) => {
    const el = document.getElementById(id);
    el.addEventListener('change', () => {
        saveSafetyConfig();
        if (el.value !== '') sendCommand(`SET:${key}=${el.value}`);
    });
});
loadSafetyConfig();

/**
 * triggerHoming() — shared helper used by both Fine Home and Save Offset & Home.
 *
 * If an E-Stop is currently active the firmware's Motor_RunHomingSequence() will
 * return immediately (silently) without moving.  We therefore auto-clear it here
 * before sending CMD:HOME, then wait 200 ms to let the firmware settle before
 * the homing command arrives.
 */
async function triggerHoming() {
    if (state.estop) {
        log('E-Stop active — clearing automatically before homing…', 'warn');
        sendCommand('CMD:CLEAR');
        state.estop = false;
        state.fault = 'NONE';
        updateUI();
        await new Promise(r => setTimeout(r, 200));
    }
    sendCommand('CMD:HOME');
    log('Homing sequence triggered');
}

document.getElementById('btn-fine-home').addEventListener('click', () => {
    requestHoming();
});

document.getElementById('btn-go-home').addEventListener('click', () => {
    sendCommand("SET:TARGET=0");
    log("Returning to home (0°)");
    if (tuningMode) { tuningState = 'IDLE'; tuningArmRun(); setMetricsStatus('ARMED (Go Home) — Waiting for motor motion...', ''); }
});

/* Home Offset: +/− step buttons, direct input, and Apply & Home button */
function applyHomeOffset(v) {
    if (isNaN(v)) return;
    document.getElementById('input-home-offset').value = v;
    sendCommand(`SET:HOME_OFFSET=${v}`);
    log(`Home offset saved → ${v}°`);
}

document.getElementById('btn-offset-dec').addEventListener('click', () => {
    const step = parseFloat(document.getElementById('input-home-offset').step) || 0.5;
    const cur  = parseFloat(document.getElementById('input-home-offset').value) || 0;
    applyHomeOffset(Math.round((cur - step) * 10) / 10);
});

document.getElementById('btn-offset-inc').addEventListener('click', () => {
    const step = parseFloat(document.getElementById('input-home-offset').step) || 0.5;
    const cur  = parseFloat(document.getElementById('input-home-offset').value) || 0;
    applyHomeOffset(Math.round((cur + step) * 10) / 10);
});

document.getElementById('input-home-offset').addEventListener('change', e => {
    applyHomeOffset(parseFloat(e.target.value));
});

/* Save Offset & Home — saves the offset value then immediately triggers homing.
 * Uses triggerHoming() so an active E-Stop is cleared automatically first. */
document.getElementById('btn-apply-home-offset').addEventListener('click', async () => {
    const v = parseFloat(document.getElementById('input-home-offset').value);
    if (isNaN(v)) { log('Home offset value is invalid — enter a number first.', 'error'); return; }
    sendCommand(`SET:HOME_OFFSET=${v}`);          // 1. persist the offset in firmware
    log(`Offset saved (${v}°) → starting homing sequence…`);
    requestHoming();                               // 2. confirm if faulted, then home
});

document.getElementById('send-tuning-btn').addEventListener('click', () => {
    for (const [key, id] of Object.entries(TUNING_FIELDS)) {
        const el = document.getElementById(id);
        if (el) sendCommand(`SET:${key}=${el.value}`);
    }
    saveTuningDraft();
    markTuningSynced();
});

document.querySelectorAll('.tuning-scroll-area input').forEach(el => {
    el.addEventListener('mousedown', () => tuningSynced = true);
    // Autosave the draft and flag unsynced on every edit.
    el.addEventListener('input', () => { saveTuningDraft(); markTuningUnsynced(); });
});

/* ---- Tuning persistence: autosave draft + named profiles ----------------
 * Draft auto-saves on every edit and restores on load so ~20 fields survive a
 * page reload. Profiles are named snapshots. Loading a profile/draft only fills
 * the fields — nothing is sent to firmware until "Apply All Parameters". */
function collectTuning() {
    const out = {};
    for (const [key, id] of Object.entries(TUNING_FIELDS)) {
        const el = document.getElementById(id);
        if (el) out[key] = el.value;
    }
    return out;
}
function applyTuningValues(obj) {
    if (!obj) return;
    for (const [key, id] of Object.entries(TUNING_FIELDS)) {
        if (obj[key] === undefined) continue;
        const el = document.getElementById(id);
        if (el) el.value = obj[key];
    }
}
function saveTuningDraft() {
    try { localStorage.setItem(TUNING_DRAFT_KEY, JSON.stringify(collectTuning())); } catch (e) { }
}
function loadTuningDraft() {
    try {
        const d = JSON.parse(localStorage.getItem(TUNING_DRAFT_KEY) || 'null');
        if (d) applyTuningValues(d);
    } catch (e) { }
}
function markTuningUnsynced() {
    const b = document.getElementById('send-tuning-btn');
    if (b) b.classList.add('needs-apply');
}
function markTuningSynced() {
    const b = document.getElementById('send-tuning-btn');
    if (b) b.classList.remove('needs-apply');
}

function getProfiles() {
    try { return JSON.parse(localStorage.getItem(TUNING_PROFILES_KEY) || '{}') || {}; }
    catch (e) { return {}; }
}
function setProfiles(p) {
    try { localStorage.setItem(TUNING_PROFILES_KEY, JSON.stringify(p)); } catch (e) { }
}
function refreshProfileSelect(selectName) {
    const sel = document.getElementById('tuning-profile-select');
    if (!sel) return;
    const profiles = getProfiles();
    const names = Object.keys(profiles).sort((a, b) => a.localeCompare(b));
    sel.textContent = '';
    const ph = document.createElement('option');
    ph.value = '';
    ph.textContent = names.length ? '— Select profile —' : '— No profiles —';
    sel.appendChild(ph);
    for (const n of names) {
        const o = document.createElement('option');
        o.value = n;
        o.textContent = n;               // textContent → safe against HTML injection
        sel.appendChild(o);
    }
    if (selectName && profiles[selectName]) sel.value = selectName;
}

(function setupTuningProfiles() {
    const sel = document.getElementById('tuning-profile-select');
    if (!sel) return;   // markup not present yet — skip wiring

    refreshProfileSelect();

    document.getElementById('btn-profile-save').addEventListener('click', () => {
        const existing = sel.value;
        const name = (prompt('Save tuning profile as:', existing || '') || '').trim();
        if (!name) return;
        const profiles = getProfiles();
        if (profiles[name] && !confirm(`Overwrite profile "${name}"?`)) return;
        profiles[name] = collectTuning();
        setProfiles(profiles);
        refreshProfileSelect(name);
        log(`Tuning profile "${name}" saved.`);
    });

    document.getElementById('btn-profile-load').addEventListener('click', () => {
        const name = sel.value;
        if (!name) { log('Pick a profile to load first.', 'error'); return; }
        const profiles = getProfiles();
        if (!profiles[name]) { log(`Profile "${name}" not found.`, 'error'); return; }
        applyTuningValues(profiles[name]);
        tuningSynced = true;             // keep firmware sync from overwriting the load
        saveTuningDraft();
        markTuningUnsynced();            // loaded but not pushed — click Apply to send
        log(`Profile "${name}" loaded into fields. Click "Apply All Parameters" to send.`);
    });

    document.getElementById('btn-profile-delete').addEventListener('click', () => {
        const name = sel.value;
        if (!name) { log('Pick a profile to delete first.', 'error'); return; }
        if (!confirm(`Delete profile "${name}"?`)) return;
        const profiles = getProfiles();
        delete profiles[name];
        setProfiles(profiles);
        refreshProfileSelect();
        log(`Profile "${name}" deleted.`);
    });

    document.getElementById('btn-profile-export').addEventListener('click', () => {
        const data = JSON.stringify(getProfiles(), null, 2);
        const blob = new Blob([data], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'robocook-tuning-profiles.json';
        a.click();
        URL.revokeObjectURL(url);
    });

    const fileInput = document.getElementById('profile-import-file');
    document.getElementById('btn-profile-import').addEventListener('click', () => fileInput.click());
    fileInput.addEventListener('change', e => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = () => {
            try {
                const incoming = JSON.parse(reader.result);
                if (!incoming || typeof incoming !== 'object' || Array.isArray(incoming))
                    throw new Error('not an object');
                const profiles = getProfiles();
                let n = 0;
                for (const [name, vals] of Object.entries(incoming)) {
                    if (vals && typeof vals === 'object') { profiles[name] = vals; n++; }
                }
                setProfiles(profiles);
                refreshProfileSelect();
                log(`Imported ${n} tuning profile(s).`);
            } catch (err) {
                log('Import failed — not a valid profiles JSON file.', 'error');
            }
        };
        reader.readAsText(file);
        fileInput.value = '';            // allow re-importing the same file
    });
})();

// Restore the autosaved draft so a reload keeps the last-entered values.
loadTuningDraft();

// --- Simulation ---
let simActive = false;
let simInterval;
document.getElementById('sim-btn').addEventListener('click', () => {
    simActive = !simActive;
    const btn = document.getElementById('sim-btn');
    btn.innerText = simActive ? "Stop Sim" : "Simulate Data";
    btn.className = simActive ? "primary-btn" : "secondary-btn";

    if (simActive) {
        state.connected = true;
        state.connectTime = Date.now();
        state.joystick = true;
        state.current = 1.2;
        state.reed_up = 1; state.reed_down = 0;
        state.reed_close = 0; state.reed_open = 1;
        let t = 0;
        simInterval = setInterval(() => {
            t += 0.05;
            state.currentPos = Math.sin(t) * 180;
            state.targetPos = Math.cos(t * 0.5) * 180;
            state.vel = Math.sin(t * 1.2) * 60;
            state.acc = Math.cos(t * 2) * 150;
            state.pwm = Math.abs(Math.sin(t)) * 100;
            state.mode = "POSITION";
            state.sysMode = "BASE";
            state.current = 1.2 + Math.sin(t * 3) * 0.3;

            // Simulate packet rate for heartbeat LINK tile
            _hbPktCount++;

            updateUI();
            visualizer.update(state.currentPos, state.firmwareTarget);
            chartPos.addData(state.currentPos);
            chartVel.addData(state.vel);
            chartAcc.addData(state.acc);
        }, 50);
    } else {
        clearInterval(simInterval);
        state.connected = false;
        state.connectTime = null;
        updateUI();
    }
});

// --- Tuning Mode State Machine ---
// (tuningMode is hoisted to the top of the file — see initial declaration.)
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
let TUNE_VEL_THRESHOLD = 3.0;   // RPM — motor considered moving
let TUNE_POS_SETTLE_DEG = 2.0;   // degrees — within target = settling
let TUNE_VEL_SETTLE_RPM = 3.0;   // RPM — velocity settled
let TUNE_SETTLE_MS = 3000;  // ms to confirm settled
let TUNE_SETTLE_BAND_PCT = 2.0;  // % of step magnitude — textbook settling band (±2%)

(function loadMetricsCfg() {
    try {
        const c = JSON.parse(localStorage.getItem('metricsCfg') || '{}');
        if (c.velThresh !== undefined) TUNE_VEL_THRESHOLD = c.velThresh;
        if (c.posSettle !== undefined) TUNE_POS_SETTLE_DEG = c.posSettle;
        if (c.velSettle !== undefined) TUNE_VEL_SETTLE_RPM = c.velSettle;
        if (c.settleMs !== undefined) TUNE_SETTLE_MS = c.settleMs;
        if (c.settleBand !== undefined) TUNE_SETTLE_BAND_PCT = c.settleBand;
    } catch (e) { }
})();

/* ---- Control-response metric helpers ---------------------------------------
 * Operate on a captured run {times[], vals[], vsets[], target}. The run is in
 * "relative toward-target" space (start ≈ 0), exactly how tuningTick stores it.
 * computeStepMetrics → 2nd-order step descriptors (best on the POSITION run).
 * computeTracking    → trajectory-following error (best on the VELOCITY run,
 *                       where vsets is the live S-curve velocity setpoint). */
function computeStepMetrics(run, bandPct) {
    if (!run || !run.vals || run.vals.length < 3) return null;
    const t = run.times, y = run.vals, N = y.length;
    const Tn = Math.abs(run.target);
    if (Tn < 1e-6) return null;
    const sgn = Math.sign(run.target) || 1;
    const yn = y.map(v => v * sgn);          // normalise so target is positive

    // Peak (in the toward-target direction) → peak time + % overshoot
    let peakV = -Infinity, peakIdx = 0;
    for (let i = 0; i < N; i++) if (yn[i] > peakV) { peakV = yn[i]; peakIdx = i; }
    const peakTime = t[peakIdx];
    const overshoot = Math.max(0, (peakV - Tn) / Tn * 100);

    // Rise time: 10% → 90% of final target
    let i10 = -1, i90 = -1;
    for (let i = 0; i < N; i++) {
        if (i10 < 0 && yn[i] >= 0.1 * Tn) i10 = i;
        if (yn[i] >= 0.9 * Tn) { i90 = i; break; }
    }
    const riseTime = (i10 >= 0 && i90 >= 0) ? (t[i90] - t[i10]) : null;

    // Steady-state value = mean of last 10% of samples
    const tail = Math.max(0, Math.floor(N * 0.9));
    let sum = 0, cnt = 0;
    for (let i = tail; i < N; i++) { sum += yn[i]; cnt++; }
    const yss = cnt ? sum / cnt : yn[N - 1];
    const ess = Math.abs(Tn - yss);

    // Settling time: last instant the signal is outside ±band of target
    const band = (bandPct / 100) * Tn;
    let settleIdx = -1;
    for (let i = 0; i < N; i++) if (Math.abs(yn[i] - Tn) > band) settleIdx = i;
    const settleTime = settleIdx < 0 ? 0 : t[Math.min(settleIdx + 1, N - 1)];

    // 2nd-order descriptors from the overshoot (only valid if there IS overshoot)
    let zeta = null, wn = null;
    if (overshoot > 0.5) {
        const lnos = Math.log(overshoot / 100);
        zeta = -lnos / Math.sqrt(Math.PI * Math.PI + lnos * lnos);
        if (peakTime > 0 && zeta < 1) wn = Math.PI / (peakTime * Math.sqrt(1 - zeta * zeta));
    }
    return { riseTime, peakTime, settleTime, overshoot, ess, zeta, wn };
}

function computeTracking(run) {
    if (!run || !run.vals || run.vals.length < 2) return { rms: null, max: null };
    let sq = 0, mx = 0, n = 0;
    for (let i = 0; i < run.vals.length; i++) {
        const e = run.vals[i] - (run.vsets[i] || 0);
        sq += e * e; mx = Math.max(mx, Math.abs(e)); n++;
    }
    return { rms: Math.sqrt(sq / n), max: mx };
}

function fmtMetric(v, unit, dp) {
    if (v === null || v === undefined || isNaN(v)) return '--';
    return v.toFixed(dp === undefined ? 2 : dp) + (unit || '');
}

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
    // Sine-driven capture (loop OFF): just push velocity samples for as long
    // as the sine is running. The settle state machine doesn't apply here.
    if (sineCapturing) {
        const amp = parseFloat(document.getElementById('input-sine-amp').value) || 0;
        const freq = parseFloat(document.getElementById('input-sine-freq').value) || 0;
        const t = (Date.now() - sineStartMs) / 1000;
        const ref = amp * Math.sin(2 * Math.PI * freq * t);
        chartVel.addRunPoint(state.vel, ref);
        return;
    }
    if (!tuningMode || tuningState === 'IDLE' || tuningState === 'DONE') return;

    const vel = Math.abs(state.vel);
    const posErr = Math.abs(state.currentPos - state.firmwareTarget);
    const now = Date.now();

    // Feed data to tuning charts
    if (tuningState === 'CAPTURING' || tuningState === 'SETTLING') {
        const relPos = (state.currentPos - tuningStartPos) * tuningDir;
        const relVel = state.vel * tuningDir;
        const relAcc = state.acc * tuningDir;
        const relVSet = state.velSetpoint * tuningDir;
        const relASet = state.accSetpoint * tuningDir;
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
    const posRun = chartPos.currentRun || (chartPos.tuningRuns.length ? null : null);
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
        const vVals = chartVel.currentRun.vals;
        let lastOutIdx = -1;
        vVals.forEach((v, i) => { if (Math.abs(v) > TUNE_VEL_SETTLE_RPM) lastOutIdx = i; });
        if (lastOutIdx >= 0) velSettleSec = vTimes[lastOutIdx];
    }

    // Rich metrics — compute BEFORE finalizeRun() nulls currentRun.
    // Position run → 2nd-order step descriptors; velocity run → tracking error.
    const stepM = computeStepMetrics(chartPos.currentRun, TUNE_SETTLE_BAND_PCT) || {};
    const track = computeTracking(chartVel.currentRun);

    chartPos.finalizeRun(settleTimeSec, posOver);
    chartVel.finalizeRun(velSettleSec, velOver);
    chartAcc.finalizeRun(settleTimeSec, 0);

    tuningRunCount++;
    const entry = {
        run: tuningRunCount,
        posSettle: settleTimeSec, posOver, velSettle: velSettleSec, velOver,
        // richer 2nd-order descriptors (position step) + tracking error (velocity)
        rise: stepM.riseTime, peak: stepM.peakTime, ess: stepM.ess,
        zeta: stepM.zeta, wn: stepM.wn,
        trackRms: track.rms, trackMax: track.max,
        band: TUNE_SETTLE_BAND_PCT,
    };
    tuningMetricsHistory.push(entry);
    if (tuningMetricsHistory.length > 10) tuningMetricsHistory.shift();

    // Update metrics panel
    document.getElementById('m-pos-settle').innerText = settleTimeSec.toFixed(2) + 's';
    document.getElementById('m-pos-over').innerText = posOver.toFixed(1) + '%';
    document.getElementById('m-vel-settle').innerText = velSettleSec.toFixed(2) + 's';
    document.getElementById('m-vel-over').innerText = velOver.toFixed(1) + '%';
    setMetricText('m-pos-rise', fmtMetric(stepM.riseTime, 's', 2));
    setMetricText('m-pos-peak', fmtMetric(stepM.peakTime, 's', 2));
    setMetricText('m-pos-ess', fmtMetric(stepM.ess, '°', 2));
    setMetricText('m-pos-zeta', fmtMetric(stepM.zeta, '', 3));
    setMetricText('m-pos-wn', fmtMetric(stepM.wn, ' rad/s', 1));
    setMetricText('m-vel-trms', fmtMetric(track.rms, ' rpm', 1));
    setMetricText('m-vel-tmax', fmtMetric(track.max, ' rpm', 1));
    renderMetricsHistory();
    if (typeof renderWorkspaceTargets === 'function') renderWorkspaceTargets();

    const zStr = stepM.zeta != null ? `  ζ≈${stepM.zeta.toFixed(2)}` : '';
    setMetricsStatus(`Run #${tuningRunCount} — settle ${settleTimeSec.toFixed(2)}s · OS ${posOver.toFixed(1)}%${zStr} · trackRMS ${fmtMetric(track.rms, '', 1)} rpm — trigger next move`, 'done');
}

/* Safe setter — new metric spans may not exist if the markup is older. */
function setMetricText(id, text) {
    const el = document.getElementById(id);
    if (el) el.innerText = text;
}

function setMetricsStatus(msg, cls) {
    const el = document.getElementById('metrics-status');
    el.innerText = msg;
    el.className = 'metrics-status-bar ' + cls;
}

function renderMetricsHistory() {
    const tbody = document.getElementById('metrics-history-body');
    tbody.innerHTML = '';
    const cell = (v, dp, suf) => (v === null || v === undefined || isNaN(v)) ? '--' : v.toFixed(dp) + (suf || '');
    [...tuningMetricsHistory].reverse().forEach(e => {
        const tr = document.createElement('tr');
        tr.innerHTML =
            `<td>${e.run}</td>` +
            `<td>${cell(e.posSettle, 2)}</td>` +
            `<td>${cell(e.posOver, 1, '%')}</td>` +
            `<td>${cell(e.rise, 2)}</td>` +
            `<td>${cell(e.zeta, 3)}</td>` +
            `<td>${cell(e.trackRms, 1)}</td>`;
        tbody.appendChild(tr);
    });
}

/* Export the full metrics history (all computed fields) as a CSV download. */
function exportMetricsCsv() {
    if (!tuningMetricsHistory.length) { log('No metric runs captured yet.', 'error'); return; }
    const cols = ['run', 'band', 'posSettle', 'posOver', 'velSettle', 'velOver',
        'rise', 'peak', 'ess', 'zeta', 'wn', 'trackRms', 'trackMax'];
    const head = ['run', 'band_%', 'pos_settle_s', 'pos_OS_%', 'vel_settle_s', 'vel_OS_%',
        'rise_s', 'peak_s', 'ess_deg', 'zeta', 'wn_rad_s', 'track_rms_rpm', 'track_max_rpm'];
    const rows = tuningMetricsHistory.map(e =>
        cols.map(c => (e[c] === null || e[c] === undefined || isNaN(e[c])) ? '' : e[c]).join(','));
    const csv = head.join(',') + '\n' + rows.join('\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `robocook-metrics-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;
    a.click();
    URL.revokeObjectURL(url);
    log(`Exported ${tuningMetricsHistory.length} metric run(s) to CSV.`);
}

/* ============================================================================
 * Phase D — A/B before/after compare.
 * Pin the latest captured run as A (baseline) then B (after a tuning change).
 * Shows a delta table (with better/worse arrows) and overlays both response
 * traces on the charts so improvement is visible and quantified.
 * ==========================================================================*/
let abSnapshot = { A: null, B: null };

/* Each metric: key into the run entry, label, unit, decimals, and direction
 * ('down' = lower is better, 'info' = no judgement, just show the delta). */
const AB_METRICS = [
    { key: 'posSettle', label: 'Pos settle', unit: 's', dp: 2, dir: 'down' },
    { key: 'posOver', label: 'Pos overshoot', unit: '%', dp: 1, dir: 'down' },
    { key: 'ess', label: 'Steady-state err', unit: '°', dp: 2, dir: 'down' },
    { key: 'rise', label: 'Rise time', unit: 's', dp: 2, dir: 'down' },
    { key: 'velSettle', label: 'Vel settle', unit: 's', dp: 2, dir: 'down' },
    { key: 'velOver', label: 'Vel overshoot', unit: '%', dp: 1, dir: 'down' },
    { key: 'trackRms', label: 'Track RMS', unit: ' rpm', dp: 1, dir: 'down' },
    { key: 'trackMax', label: 'Track max', unit: ' rpm', dp: 1, dir: 'down' },
    { key: 'zeta', label: 'ζ (damping)', unit: '', dp: 3, dir: 'info' },
    { key: 'wn', label: 'ωₙ est', unit: ' rad/s', dp: 1, dir: 'info' },
];

function captureAB(slot) {
    const m = latestRun();
    if (!m) { log('No captured run yet — do a tuning move first, then Pin.', 'warn'); return; }
    const lastTrace = ch => (ch.tuningRuns && ch.tuningRuns.length)
        ? JSON.parse(JSON.stringify(ch.tuningRuns[ch.tuningRuns.length - 1])) : null;
    abSnapshot[slot] = {
        metrics: { ...m },
        pos: lastTrace(chartPos),
        vel: lastTrace(chartVel),
    };
    // Push the trace onto the charts as a persistent pinned overlay.
    chartPos.pinnedRuns[slot] = abSnapshot[slot].pos;
    chartVel.pinnedRuns[slot] = abSnapshot[slot].vel;
    chartPos.draw(); chartVel.draw();
    renderAB();
    log(`Pinned run #${m.run} as ${slot} (${slot === 'A' ? 'before' : 'after'}).`);
}

function clearAB() {
    abSnapshot = { A: null, B: null };
    chartPos.pinnedRuns = { A: null, B: null };
    chartVel.pinnedRuns = { A: null, B: null };
    chartPos.draw(); chartVel.draw();
    renderAB();
}

function renderAB() {
    const tbody = document.getElementById('ab-table-body');
    if (!tbody) return;
    const A = abSnapshot.A && abSnapshot.A.metrics;
    const B = abSnapshot.B && abSnapshot.B.metrics;
    if (!A && !B) {
        tbody.innerHTML = '<tr><td colspan="4" class="ab-empty">Capture a run, Pin A, tune, capture again, Pin B.</td></tr>';
        return;
    }
    const fmt = (v, dp, unit) => (v === null || v === undefined || isNaN(v)) ? '--' : v.toFixed(dp) + (unit || '');
    tbody.innerHTML = '';
    AB_METRICS.forEach(mt => {
        const a = A ? A[mt.key] : null;
        const b = B ? B[mt.key] : null;
        let deltaTxt = '--', deltaCls = '';
        if (a !== null && a !== undefined && !isNaN(a) && b !== null && b !== undefined && !isNaN(b)) {
            const d = b - a;
            const arrow = d > 0 ? '▲' : (d < 0 ? '▼' : '·');
            deltaTxt = `${arrow} ${Math.abs(d).toFixed(mt.dp)}${mt.unit || ''}`;
            if (mt.dir === 'down' && Math.abs(d) > 1e-9) deltaCls = d < 0 ? 'ab-better' : 'ab-worse';
        }
        const tr = document.createElement('tr');
        tr.innerHTML =
            `<td class="ab-metric">${mt.label}</td>` +
            `<td>${fmt(a, mt.dp, mt.unit)}</td>` +
            `<td>${fmt(b, mt.dp, mt.unit)}</td>` +
            `<td class="${deltaCls}">${deltaTxt}</td>`;
        tbody.appendChild(tr);
    });
}

(function initAB() {
    const a = document.getElementById('btn-ab-pin-a');
    const b = document.getElementById('btn-ab-pin-b');
    const c = document.getElementById('btn-ab-clear');
    if (a) a.addEventListener('click', () => captureAB('A'));
    if (b) b.addEventListener('click', () => captureAB('B'));
    if (c) c.addEventListener('click', clearAB);
})();

/* ============================================================================
 * Phase B — per-element Tuning Workspace
 * A guide that focuses one control element at a time: shows the method, how to
 * prove it, target metrics with pass/fail badges (editable thresholds), and a
 * safe "set up test" button that puts the dashboard into the right state.
 * It does NOT duplicate the parameter inputs — it points at the real fields.
 * ==========================================================================*/

function latestRun() {
    return tuningMetricsHistory.length ? tuningMetricsHistory[tuningMetricsHistory.length - 1] : null;
}
/* Read the live Kalman RMSE-θ readout (degrees) straight from the DOM. */
function kfRmseThetaVal() {
    const el = document.getElementById('kf-rmse-theta');
    if (!el) return null;
    const v = parseFloat((el.textContent || '').replace(/[^0-9.eE+-]/g, ''));
    return isNaN(v) ? null : v;
}

const TUNE_ELEMENTS = {
    velocity: {
        method: 'Bypass the position loop, drive a velocity sine/step. Raise Speed Kp for speed, add Ki to erase steady-state error, a touch of Kd to damp.',
        prove: 'Velocity follows its setpoint with low overshoot and quick settle. Watch raw-vel vs setpoint on the VELOCITY chart.',
        fields: ['input-speed-kp', 'input-speed-ki', 'input-speed-kd'],
        test: 'velLoop',
        hint: 'Sets Position Loop OFF → use the Sine generator (no motion until you press Start Sine).',
        targets: [
            { label: 'Overshoot', get: r => r ? r.velOver : null, cmp: 'lte', thr: 15, unit: '%', dp: 1, step: 1 },
            { label: 'Settling', get: r => r ? r.velSettle : null, cmp: 'lte', thr: 0.5, unit: 's', dp: 2, step: 0.05 },
        ],
    },
    feedforward: {
        method: 'With the loop closed, add K_vff (velocity) and K_aff (accel) so the drive is pushed open-loop along the trajectory; K_tff cancels measured load torque.',
        prove: 'Tracking error (actual − S-curve setpoint) shrinks. Track RMS is the headline number — lower is better.',
        fields: ['input-k-vff', 'input-k-aff', 'input-k-tff'],
        test: 'closedMove',
        hint: 'Loop ON + Tuning mode. Command a Move and watch Track RMS fall as FF improves.',
        targets: [
            { label: 'Track RMS', get: r => r ? r.trackRms : null, cmp: 'lte', thr: 3, unit: ' rpm', dp: 1, step: 0.5 },
            { label: 'Track max', get: r => r ? r.trackMax : null, cmp: 'lte', thr: 8, unit: ' rpm', dp: 1, step: 0.5 },
        ],
    },
    position: {
        method: 'Close the loop. Raise Pos Kp until the step is fast, add Kd to damp overshoot, small Ki to remove residual error. Inside-out: velocity loop first.',
        prove: '2nd-order step looks clean: overshoot under target, e_ss near zero, settle quick. ζ is estimated from overshoot.',
        fields: ['input-pos-kp', 'input-pos-ki', 'input-pos-kd'],
        test: 'closedMove',
        hint: 'Loop ON + Tuning mode. Command a step Move; metrics capture automatically.',
        targets: [
            { label: 'Overshoot', get: r => r ? r.posOver : null, cmp: 'lte', thr: 10, unit: '%', dp: 1, step: 1 },
            { label: 'e_ss', get: r => r ? r.ess : null, cmp: 'lte', thr: 1.0, unit: '°', dp: 2, step: 0.1 },
            { label: 'Settling', get: r => r ? r.posSettle : null, cmp: 'lte', thr: 0.8, unit: 's', dp: 2, step: 0.05 },
        ],
    },
    trajectory: {
        method: 'Shape the motion with v_max / a_max / j_max. Jerk-limited S-curves keep accel continuous so the arm does not jolt the structure.',
        prove: 'Smooth move with little overshoot and low tracking error — the trajectory is feasible for the drive, not clipped.',
        fields: ['input-vmax-rad', 'input-amax-rad', 'input-jmax-rad', 'input-min-pwm'],
        test: 'closedMove',
        hint: 'Loop ON + Tuning mode. Command a Move; if Track RMS spikes the profile is too aggressive.',
        targets: [
            { label: 'Track RMS', get: r => r ? r.trackRms : null, cmp: 'lte', thr: 3, unit: ' rpm', dp: 1, step: 0.5 },
            { label: 'Overshoot', get: r => r ? r.posOver : null, cmp: 'lte', thr: 5, unit: '%', dp: 1, step: 1 },
        ],
    },
    shaper: {
        method: 'ZVD input shaper splits each command into impulses spaced by the half-period of the flexible mode (ωₙ, ζ) so the residual vibration cancels itself.',
        prove: 'Turn the shaper ON and the residual overshoot/ringing after a move drops sharply versus OFF. Get ωₙ/ζ from a frequency sweep.',
        fields: ['input-shaper-wn', 'input-shaper-zeta'],
        test: 'closedMove',
        hint: 'Loop ON + Tuning mode. Do a Move with Shaper OFF, then ON, and compare overshoot.',
        targets: [
            { label: 'Resid. OS', get: r => r ? r.posOver : null, cmp: 'lte', thr: 3, unit: '%', dp: 1, step: 0.5 },
        ],
    },
    kalman: {
        method: 'Tune Q/R in the Kalman section: large σ (Q) trusts the sensor, large R trusts the model. Balance so the estimate is smooth yet tracks changes.',
        prove: 'RMSE θ̂ vs measurement stays small while ω̂ is far smoother than raw velocity. Innovation should look like white noise.',
        fields: ['input-kf-q-theta', 'input-kf-q-omega', 'input-kf-q-tau', 'input-kf-q-i', 'input-kf-r'],
        test: 'kalman',
        hint: 'Enables the filter and reveals the Kalman card. Compare θ̂/ω̂ against Encoder + Model overlays.',
        targets: [
            { label: 'RMSE θ̂', get: () => kfRmseThetaVal(), cmp: 'lte', thr: 1.0, unit: '°', dp: 2, step: 0.1 },
        ],
    },
};

let wsCurrent = 'velocity';
let tuneThr = {};
(function loadTuneThr() {
    try { tuneThr = JSON.parse(localStorage.getItem('tuneTargets') || '{}'); } catch (e) { tuneThr = {}; }
})();
function thrKey(el, i) { return el + '.' + i; }
function getTuneThr(el, i, def) { const k = thrKey(el, i); return tuneThr[k] !== undefined ? tuneThr[k] : def; }
function setTuneThr(el, i, v) {
    if (isNaN(v)) return;
    tuneThr[thrKey(el, i)] = v;
    localStorage.setItem('tuneTargets', JSON.stringify(tuneThr));
}

function renderWorkspaceTargets() {
    const cfg = TUNE_ELEMENTS[wsCurrent];
    const wrap = document.getElementById('ws-targets');
    if (!cfg || !wrap) return;
    const run = latestRun();
    wrap.innerHTML = '';
    let anyData = false;
    cfg.targets.forEach((t, i) => {
        const val = t.get(run);
        const thr = getTuneThr(wsCurrent, i, t.thr);
        let cls = 'na';
        if (val !== null && val !== undefined && !isNaN(val)) {
            anyData = true;
            cls = (t.cmp === 'gte' ? val >= thr : val <= thr) ? 'pass' : 'fail';
        }
        const div = document.createElement('div');
        div.className = 'ws-target ' + cls;
        const valTxt = (val === null || val === undefined || isNaN(val)) ? '--' : val.toFixed(t.dp) + (t.unit || '');
        div.innerHTML =
            `<span class="wt-label">${t.label}</span>` +
            `<span class="wt-val">${valTxt}</span>` +
            `<span class="wt-cmp">${t.cmp === 'gte' ? '≥' : '≤'}</span>` +
            `<input class="wt-thr" type="number" step="${t.step || 0.1}" value="${thr}" title="Editable pass/fail target">`;
        const inp = div.querySelector('.wt-thr');
        inp.addEventListener('change', () => { setTuneThr(wsCurrent, i, parseFloat(inp.value)); renderWorkspaceTargets(); });
        // Clicking the badge body (not the input) should not steal focus from the number field
        wrap.appendChild(div);
    });
    return anyData;
}

function renderWorkspace(key) {
    if (key) wsCurrent = key;
    const cfg = TUNE_ELEMENTS[wsCurrent];
    if (!cfg) return;
    document.querySelectorAll('#ws-selector .ws-chip').forEach(c =>
        c.classList.toggle('active', c.dataset.el === wsCurrent));
    const mt = document.getElementById('ws-method-text');
    const pt = document.getElementById('ws-prove-text');
    const ht = document.getElementById('ws-hint');
    if (mt) mt.textContent = cfg.method;
    if (pt) pt.textContent = cfg.prove;
    if (ht) ht.textContent = cfg.hint || '';
    renderWorkspaceTargets();
    // Panes are gone — sections are always visible (scrollable list)
}

/* Scroll to + pulse the real parameter fields for this element. */
function workspaceFocus() {
    const cfg = TUNE_ELEMENTS[wsCurrent];
    if (!cfg || !cfg.fields) return;
    let first = null;
    cfg.fields.forEach(id => {
        const el = document.getElementById(id);
        if (!el) return;
        const grp = el.closest('.input-group') || el;
        if (!first) first = grp;
        grp.classList.add('ws-highlight');
        setTimeout(() => grp.classList.remove('ws-highlight'), 2200);
    });
    if (first) first.scrollIntoView({ behavior: 'smooth', block: 'center' });
}

/* Put the dashboard into the right (safe) state to test this element.
 * No motion is ever commanded here — the user still triggers the move/sine. */
function workspaceSetupTest() {
    const cfg = TUNE_ELEMENTS[wsCurrent];
    if (!cfg) return;
    switch (cfg.test) {
        case 'velLoop':
            // Position loop OFF reveals the sine generator and forces Tuning mode.
            if (posLoopEnabled) btnPosLoop.click();
            else if (!tuningMode) setTuningMode(true);
            log('Velocity-loop test ready — press Start Sine (or jog) to excite the loop.');
            break;
        case 'closedMove':
            // Loop ON + Tuning mode so a step Move is captured by the metrics engine.
            if (!posLoopEnabled) btnPosLoop.click();
            if (!tuningMode) setTuningMode(true);
            log('Closed-loop test ready — command a Move; metrics capture automatically.');
            break;
        case 'kalman':
            // Enable the filter and make sure the Kalman card is visible.
            if (!state.kfEnabled) document.getElementById('btn-kf-toggle').click();
            if (posLoopEnabled && !tuningMode) setTuningMode(true);
            document.querySelector('.kalman-card')?.scrollIntoView({ behavior: 'smooth', block: 'center' });
            log('Kalman test ready — filter enabled; compare θ̂/ω̂ with Encoder + Model overlays.');
            break;
    }
    renderWorkspace();
}

(function initWorkspace() {
    const sel = document.getElementById('ws-selector');
    const scrollArea = document.getElementById('tl-scroll');
    if (sel) {
        sel.querySelectorAll('.ws-chip').forEach(chip => {
            chip.addEventListener('click', () => {
                const key = chip.dataset.el;
                // Update active chip highlight
                sel.querySelectorAll('.ws-chip').forEach(c => c.classList.toggle('active', c === chip));
                wsCurrent = key;
                // Scroll the tl-scroll area to the matching section
                const sec = document.getElementById('tl-sec-' + key);
                if (sec && scrollArea) {
                    sec.scrollIntoView({ behavior: 'smooth', block: 'start' });
                }
            });
        });
    }
    const tb = document.getElementById('ws-test-btn');
    if (tb) tb.addEventListener('click', workspaceSetupTest);
    const gk = document.getElementById('btn-goto-kalman');
    if (gk) gk.addEventListener('click', () => document.querySelector('.kalman-card')?.scrollIntoView({ behavior: 'smooth', block: 'start' }));
    // Freq Sweep — show on demand from Velocity / Shaper panes, hide via ×
    const showSweep = () => {
        const card = document.getElementById('freq-sweep-card');
        if (card) { card.style.display = ''; card.scrollIntoView({ behavior: 'smooth', block: 'start' }); }
    };
    const hideSweep = () => {
        const card = document.getElementById('freq-sweep-card');
        if (card) card.style.display = 'none';
    };
    document.getElementById('btn-goto-sweep-vel')?.addEventListener('click', showSweep);
    document.getElementById('btn-goto-sweep-shaper')?.addEventListener('click', showSweep);
    document.getElementById('btn-sweep-close')?.addEventListener('click', hideSweep);
    renderWorkspace('velocity');
})();

/* ============================================================================
 * Phase C — Frequency Sweep (mini-Bode) for system identification.
 * Drives the firmware velocity-loop sine at a log-spaced set of frequencies,
 * and at each one measures the steady-state gain & phase of the actual velocity
 * (state.vel) relative to its commanded setpoint (state.velSetpoint) using a
 * single-bin DFT (Goertzel). From the magnitude curve we read the closed-loop
 * bandwidth (-3 dB) and any mechanical resonance peak ωₙ, which feeds the ZVD
 * input shaper. No motion command is issued except the sine the user opted into.
 * ==========================================================================*/
const bode = {
    running: false,
    abort: false,
    results: [],   // [{ f, gainLin, gainDb, phaseDeg }]
    wnRad: null,
    zeta: null,
};

function bodeLogFreqs(fmin, fmax, n) {
    const a = Math.log10(fmin), b = Math.log10(fmax);
    const out = [];
    for (let i = 0; i < n; i++) out.push(Math.pow(10, a + (b - a) * (n === 1 ? 0 : i / (n - 1))));
    return out;
}

/* Single-bin DFT: amplitude & phase of a sampled signal at frequency f (Hz).
 * times[] in seconds. For s(t)=A·cos(2πft+φ):  Re→A cosφ, Im→−A sinφ. */
function bodeDftBin(times, vals, f) {
    let re = 0, im = 0;
    const N = vals.length;
    if (!N) return { amp: 0, phase: 0 };
    for (let i = 0; i < N; i++) {
        const th = 2 * Math.PI * f * times[i];
        re += vals[i] * Math.cos(th);
        im += vals[i] * Math.sin(th);
    }
    re *= 2 / N; im *= 2 / N;
    return { amp: Math.hypot(re, im), phase: Math.atan2(-im, re) };
}

const bodeSleep = ms => new Promise(r => setTimeout(r, ms));

/* Sample the live velocity + setpoint for durationMs at ~stepMs spacing. */
async function bodeCapture(durationMs, stepMs = 20) {
    const t0 = performance.now();
    const times = [], vel = [], vset = [];
    while (performance.now() - t0 < durationMs) {
        if (bode.abort) break;
        times.push((performance.now() - t0) / 1000);
        vel.push(state.vel || 0);
        vset.push(state.velSetpoint || 0);
        await bodeSleep(stepMs);
    }
    return { times, vel, vset };
}

function setBodeStatus(msg, cls) {
    const el = document.getElementById('bode-status');
    if (el) { el.textContent = msg; el.className = 'bode-status ' + (cls || ''); }
}
function setBodeBtn(running) {
    const b = document.getElementById('btn-bode-start');
    if (!b) return;
    b.textContent = running ? '■ Stop' : '▶ Start';
    b.classList.toggle('danger', running);
}

async function runBodeSweep() {
    if (bode.running) { bode.abort = true; bode.running = false; return; }
    if (!state.connected) { log('Connect the robot before a frequency sweep.', 'warn'); return; }

    const fmin = parseFloat(document.getElementById('input-bode-fmin').value) || 0.3;
    const fmax = parseFloat(document.getElementById('input-bode-fmax').value) || 8;
    const n = Math.max(3, Math.min(40, parseInt(document.getElementById('input-bode-points').value) || 14));
    const amp = parseFloat(document.getElementById('input-bode-amp').value) || 20;
    const cyc = Math.max(2, parseInt(document.getElementById('input-bode-cycles').value) || 6);
    if (fmax <= fmin) { log('Bode: f max must be greater than f min.', 'warn'); return; }

    // The sine generator only runs with the position loop bypassed.
    if (posLoopEnabled) {
        btnPosLoop.click();
        await bodeSleep(300);
    }

    bode.running = true; bode.abort = false; bode.results = [];
    bode.wnRad = null; bode.zeta = null;
    setBodeBtn(true);
    document.getElementById('btn-bode-to-shaper').disabled = true;
    const freqs = bodeLogFreqs(fmin, fmax, n);
    sendCommand(`SET:SINE_AMP=${amp}`);

    for (let i = 0; i < freqs.length; i++) {
        if (bode.abort) break;
        const f = freqs[i];
        const period = 1000 / f;
        setBodeStatus(`Point ${i + 1}/${n} — ${f.toFixed(2)} Hz`, 'busy');
        sendCommand(`SET:SINE_FREQ=${f}`);
        sendCommand('SET:SINE_EN=1');
        // Let the loop reach steady state: ≥3 cycles, ≥1.2 s.
        await bodeSleep(Math.max(1200, 3 * period));
        if (bode.abort) break;
        // Capture an integer number of cycles for a clean DFT bin.
        const cap = await bodeCapture(Math.max(1500, cyc * period));
        const sIn = bodeDftBin(cap.times, cap.vset, f);
        const sOut = bodeDftBin(cap.times, cap.vel, f);
        if (sIn.amp > 0.5) {           // ignore points with no real excitation
            const gainLin = sOut.amp / sIn.amp;
            let ph = (sOut.phase - sIn.phase) * 180 / Math.PI;
            while (ph > 180) ph -= 360;
            while (ph < -180) ph += 360;
            bode.results.push({ f, gainLin, gainDb: 20 * Math.log10(Math.max(1e-6, gainLin)), phaseDeg: ph });
            drawBode();
        }
    }

    sendCommand('SET:SINE_EN=0');
    bode.running = false;
    setBodeBtn(false);
    if (bode.abort) { setBodeStatus('Sweep stopped.', ''); return; }
    finishBode();
}

/* After the sweep: find -3 dB bandwidth and the resonance peak, estimate ζ. */
function finishBode() {
    const R = bode.results;
    // Reveal the results panel now that we have data
    const fsResults = document.getElementById('fs-results');
    if (fsResults) fsResults.classList.remove('hidden');
    if (R.length < 3) { setBodeStatus('Sweep done — too few valid points.', ''); return; }

    // Low-frequency reference gain (mean of the lowest two points).
    const g0Db = (R[0].gainDb + R[1].gainDb) / 2;

    // Resonance: the maximum-gain point, if it rises meaningfully above g0.
    let peak = R[0];
    R.forEach(p => { if (p.gainDb > peak.gainDb) peak = p; });
    const hasPeak = peak.gainDb > g0Db + 0.5;

    // Bandwidth: first crossing below g0 - 3 dB after the peak.
    const startIdx = R.indexOf(peak);
    let bwHz = null;
    for (let i = Math.max(1, startIdx); i < R.length; i++) {
        if (R[i].gainDb <= g0Db - 3) {
            // linear-interpolate in log-freq for a smoother estimate
            const a = R[i - 1], b = R[i];
            const t = (g0Db - 3 - a.gainDb) / (b.gainDb - a.gainDb);
            const lf = Math.log10(a.f) + t * (Math.log10(b.f) - Math.log10(a.f));
            bwHz = Math.pow(10, lf);
            break;
        }
    }

    const bwEl = document.getElementById('bode-bw');
    const wnEl = document.getElementById('bode-wn');
    const pkEl = document.getElementById('bode-peak');
    const zEl = document.getElementById('bode-zeta');

    bwEl.textContent = bwHz ? `${bwHz.toFixed(2)} Hz · ${(2 * Math.PI * bwHz).toFixed(1)} rad/s` : 'not reached';

    if (hasPeak) {
        // Resonant-peak magnitude ratio Mr = peak/low-freq gain (linear).
        const Mr = peak.gainLin / Math.pow(10, g0Db / 20);
        // For a 2nd-order system Mr = 1/(2ζ√(1−ζ²)); for light damping ζ ≈ 1/(2·Mr).
        let zeta = 1 / (2 * Mr);
        if (zeta > 0 && zeta < 0.707) {
            // refine: invert Mr = 1/(2ζ√(1−ζ²)) once
            const z2 = 1 / (2 * Mr);
            zeta = z2;  // good first-order estimate for sharp peaks
        }
        // ωr ≈ ωn√(1−2ζ²) → ωn ≈ ωr/√(1−2ζ²)
        const wr = 2 * Math.PI * peak.f;
        const corr = Math.sqrt(Math.max(0.0001, 1 - 2 * zeta * zeta));
        bode.wnRad = wr / corr;
        bode.zeta = zeta;
        wnEl.textContent = `${peak.f.toFixed(2)} Hz · ${bode.wnRad.toFixed(1)} rad/s`;
        pkEl.textContent = `${peak.gainDb.toFixed(1)} dB`;
        zEl.textContent = zeta.toFixed(3);
        document.getElementById('btn-bode-to-shaper').disabled = false;
        setBodeStatus(`Done — resonance ${peak.f.toFixed(2)} Hz, ζ≈${zeta.toFixed(3)}. Send ωₙ to the shaper.`, 'done');
    } else {
        wnEl.textContent = 'none found';
        pkEl.textContent = `${peak.gainDb.toFixed(1)} dB`;
        zEl.textContent = '--';
        setBodeStatus('Done — no resonance peak (well damped). Bandwidth shown above.', 'done');
    }
}

/* Draw both Bode panels (magnitude + phase) on their canvases. */
function drawBode() {
    // Show results panel as soon as any data exists
    if (bode.results.length > 0) {
        const fsResults = document.getElementById('fs-results');
        if (fsResults) fsResults.classList.remove('hidden');
    }
    drawBodePanel('bode-mag', r => r.gainDb, 'dB', true);
    drawBodePanel('bode-phase', r => r.phaseDeg, '°', false);
}

function drawBodePanel(canvasId, accessor, unit, isMag) {
    const cv = document.getElementById(canvasId);
    if (!cv) return;
    const w = cv.clientWidth || 300, h = cv.clientHeight || 90;
    if (cv.width !== w) cv.width = w;
    if (cv.height !== h) cv.height = h;
    const ctx = cv.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    const R = bode.results;
    const padL = 34, padR = 8, padT = 8, padB = 16;
    const x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;

    // Axes background
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    if (R.length < 1) {
        ctx.fillStyle = 'rgba(255,255,255,0.3)';
        ctx.font = '10px monospace';
        ctx.fillText('awaiting sweep…', x0 + 8, (y0 + y1) / 2);
        return;
    }

    const fmin = parseFloat(document.getElementById('input-bode-fmin').value) || R[0].f;
    const fmax = parseFloat(document.getElementById('input-bode-fmax').value) || R[R.length - 1].f;
    const lxMin = Math.log10(fmin), lxMax = Math.log10(fmax);
    const vals = R.map(accessor);
    let vMin = Math.min(...vals), vMax = Math.max(...vals);
    if (isMag) { vMin = Math.min(vMin, -3); vMax = Math.max(vMax, 3); }
    else { vMin = Math.min(vMin, -190); vMax = Math.max(vMax, 10); }
    if (vMax - vMin < 1e-6) { vMax += 1; vMin -= 1; }
    const pad = (vMax - vMin) * 0.1;
    vMin -= pad; vMax += pad;

    const xOf = f => x0 + (Math.log10(f) - lxMin) / (lxMax - lxMin) * (x1 - x0);
    const yOf = v => y1 - (v - vMin) / (vMax - vMin) * (y1 - y0);

    // Y gridlines / labels
    ctx.fillStyle = 'rgba(255,255,255,0.4)';
    ctx.font = '9px monospace';
    const ticks = 4;
    for (let i = 0; i <= ticks; i++) {
        const v = vMin + (vMax - vMin) * i / ticks;
        const y = yOf(v);
        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
        ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
        ctx.fillText(v.toFixed(0), 2, y + 3);
    }
    // Decade gridlines on the log-x axis
    for (let d = Math.ceil(lxMin); d <= Math.floor(lxMax); d++) {
        const x = xOf(Math.pow(10, d));
        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
        ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke();
        ctx.fillText(`${Math.pow(10, d)}`, x - 4, y1 + 11);
    }

    // Reference lines: -3 dB band (mag) or -90° (phase)
    ctx.setLineDash([3, 3]);
    if (isMag) {
        const g0 = (R.length > 1 ? (accessor(R[0]) + accessor(R[1])) / 2 : accessor(R[0])) - 3;
        ctx.strokeStyle = 'rgba(244,255,77,0.4)';
        ctx.beginPath(); ctx.moveTo(x0, yOf(g0)); ctx.lineTo(x1, yOf(g0)); ctx.stroke();
    } else {
        ctx.strokeStyle = 'rgba(244,255,77,0.4)';
        ctx.beginPath(); ctx.moveTo(x0, yOf(-90)); ctx.lineTo(x1, yOf(-90)); ctx.stroke();
    }
    ctx.setLineDash([]);

    // Data trace
    ctx.strokeStyle = isMag ? '#00f2ff' : '#ff00ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    R.forEach((r, i) => {
        const x = xOf(r.f), y = yOf(accessor(r));
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
    // Data points
    ctx.fillStyle = isMag ? '#00f2ff' : '#ff00ff';
    R.forEach(r => {
        const x = xOf(r.f), y = yOf(accessor(r));
        ctx.beginPath(); ctx.arc(x, y, 2, 0, 2 * Math.PI); ctx.fill();
    });

    // Mark the resonance peak on the magnitude panel
    if (isMag && bode.wnRad) {
        const fRes = bode.wnRad / (2 * Math.PI);
        const x = xOf(fRes);
        ctx.strokeStyle = 'rgba(57,255,20,0.7)';
        ctx.setLineDash([2, 2]);
        ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke();
        ctx.setLineDash([]);
    }
}

(function initBode() {
    const start = document.getElementById('btn-bode-start');
    if (start) start.addEventListener('click', runBodeSweep);
    const toShaper = document.getElementById('btn-bode-to-shaper');
    if (toShaper) toShaper.addEventListener('click', () => {
        if (!bode.wnRad) return;
        const wnEl = document.getElementById('input-shaper-wn');
        const zEl = document.getElementById('input-shaper-zeta');
        wnEl.value = bode.wnRad.toFixed(2);
        zEl.value = Math.min(0.999, Math.max(0, bode.zeta || 0.04)).toFixed(3);
        // Fire change handlers so the values are pushed to the firmware.
        wnEl.dispatchEvent(new Event('change'));
        zEl.dispatchEvent(new Event('change'));
        if (typeof updateShaperDelayInfo === 'function') updateShaperDelayInfo();
        log(`Shaper set from sweep: ωₙ=${bode.wnRad.toFixed(2)} rad/s, ζ=${(bode.zeta || 0).toFixed(3)}`);
    });
    // Redraw on resize so the canvases stay crisp.
    window.addEventListener('resize', () => { if (bode.results.length) drawBode(); });
})();

// Show/hide individual telemetry charts
const CHART_TOGGLES = [
    { chk: 'chk-show-pos', wrap: 'wrap-chart-pos', chart: () => chartPos },
    { chk: 'chk-show-vel', wrap: 'wrap-chart-vel', chart: () => chartVel },
    { chk: 'chk-show-acc', wrap: 'wrap-chart-acc', chart: () => chartAcc },
];
function applyChartVisibility() {
    CHART_TOGGLES.forEach(t => {
        const wrap = document.getElementById(t.wrap);
        const checked = document.getElementById(t.chk).checked;
        if (wrap) wrap.style.display = checked ? '' : 'none';
    });
    // Force the surviving canvases to re-measure their container
    requestAnimationFrame(() => {
        CHART_TOGGLES.forEach(t => {
            if (document.getElementById(t.chk).checked) t.chart().resize();
        });
    });
}
CHART_TOGGLES.forEach(t => {
    document.getElementById(t.chk).addEventListener('change', applyChartVisibility);
});
applyChartVisibility();

// View mode: 'live' | 'tuning' | 'test'
let viewMode = 'live';

function setTuningMode(on) {
    setViewMode(on ? 'tuning' : 'live');
}

function setViewMode(mode) {
    if (viewMode === mode) return;
    viewMode = mode;
    tuningMode = (mode === 'tuning');

    // Update segmented switch
    document.querySelectorAll('#view-mode-switch .vm-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === mode);
    });

    // Charts switch live/tuning behavior (Test reuses live scaling)
    chartPos.setMode(tuningMode);
    chartVel.setMode(tuningMode);
    chartAcc.setMode(tuningMode);
    tuningState = 'IDLE';

    // Telemetry card: show test panel instead of charts when in Test mode
    const telCard = document.querySelector('.telemetry-card');
    if (telCard) telCard.classList.toggle('test-mode', mode === 'test');
    const testPanel = document.getElementById('test-panel');
    if (testPanel) testPanel.classList.toggle('hidden', mode !== 'test');

    // Path card hides in tuning mode (preserve original behavior)
    document.querySelector('.path-card').style.display = (mode === 'tuning') ? 'none' : '';

    const _metrics = document.querySelector('.metrics-card');
    if (_metrics) _metrics.style.display = (tuningMode && posLoopEnabled) ? '' : 'none';
    const _kf = document.querySelector('.kalman-card');
    if (_kf) _kf.style.display = !posLoopEnabled ? '' : 'none';

    if (mode === 'tuning') {
        setMetricsStatus('IDLE — Send Move / Go Home / Ghost start to begin capture', '');
    } else if (mode === 'live') {
        setMetricsStatus('IDLE — Switch to Tuning Mode and move motor', '');
    }
}

// Segmented switch — wires Live / Tuning / Test buttons
document.querySelectorAll('#view-mode-switch .vm-btn').forEach(btn => {
    btn.addEventListener('click', () => setViewMode(btn.dataset.mode));
});

// Default: hide metrics card on load (Live mode default)
document.addEventListener('DOMContentLoaded', () => {
    const m = document.querySelector('.metrics-card');
    if (m) m.style.display = 'none';
});

/* ============================================================================
 * Kalman Filter dashboard
 * ========================================================================== */

/* Relocate the Kalman card from the right column to the left column so it
 * lives next to (and replaces) the Step Response Metrics panel. */
(function relocateKalmanCard() {
    const kf = document.querySelector('.kalman-card');
    const metrics = document.querySelector('.metrics-card');
    if (kf && metrics && metrics.parentElement) {
        metrics.parentElement.insertBefore(kf, metrics.nextSibling);
        kf.style.display = 'none';  // visibility is driven by Loop / Tuning state
    }
})();
const KF_RMSE_WINDOW = 100;         // last N samples for RMSE
const kfRmseTheta = [];
const kfRmseOmega = [];

function updateKalmanCard() {
    const set = (id, txt) => { const el = document.getElementById(id); if (el) el.innerText = txt; };
    set('kf-theta', state.kfTheta.toFixed(2));
    set('kf-omega', state.kfOmega.toFixed(2));
    set('kf-tau', state.kfTau.toFixed(3));
    set('kf-ia', state.kfIa.toFixed(3));
    set('kf-innov', state.kfInnov.toExponential(2));
    set('kf-p00', state.kfP00.toExponential(2));
    set('kf-p11', state.kfP11.toExponential(2));
    set('kf-p22', state.kfP22.toExponential(2));

    // RMSE: compare KF estimate to the raw signal we'd otherwise use.
    // For pos: kfTheta vs raw encoder (state.currentPos).
    // For vel: kfOmega vs lowpass-derived RPM (state.vel).
    kfRmseTheta.push(state.kfTheta - state.currentPos);
    kfRmseOmega.push(state.kfOmega - state.vel);
    if (kfRmseTheta.length > KF_RMSE_WINDOW) kfRmseTheta.shift();
    if (kfRmseOmega.length > KF_RMSE_WINDOW) kfRmseOmega.shift();
    const rms = arr => {
        if (!arr.length) return 0;
        let s = 0; for (const v of arr) s += v * v;
        return Math.sqrt(s / arr.length);
    };
    set('kf-rmse-theta', rms(kfRmseTheta).toFixed(3) + ' °');
    set('kf-rmse-omega', rms(kfRmseOmega).toFixed(2) + ' RPM');

    // Toggle button reflects firmware state (so the dashboard syncs after reload)
    const btn = document.getElementById('btn-kf-toggle');
    if (btn) {
        btn.textContent = state.kfEnabled ? 'KF: ON' : 'KF: OFF';
        btn.classList.toggle('active', state.kfEnabled);
        btn.classList.toggle('warning', !state.kfEnabled);
    }
}

document.getElementById('btn-kf-toggle').addEventListener('click', () => {
    const next = !state.kfEnabled;
    sendCommand(`SET:KF_EN=${next ? 1 : 0}`);
    log(`Kalman filter ${next ? 'ENABLED' : 'DISABLED'}`);
});

document.getElementById('btn-kf-reset').addEventListener('click', () => {
    sendCommand('SET:KF_RESET=1');
    kfRmseTheta.length = 0;
    kfRmseOmega.length = 0;
    log('Kalman filter reset to current encoder reading');
});

document.getElementById('btn-kf-sanity').addEventListener('click', () => {
    state.kfSanityShow = !state.kfSanityShow;
    const b = document.getElementById('btn-kf-sanity');
    b.textContent = state.kfSanityShow ? 'Model: ON' : 'Model: OFF';
    b.classList.toggle('active', state.kfSanityShow);
    b.classList.toggle('warning', !state.kfSanityShow);
    if (!state.kfSanityShow) {
        chartPos.clearSanity();
        chartVel.clearSanity();
    }
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
    gripperConfig.delays.open = parseInt(document.getElementById('delay-open').value) || 600;
    gripperConfig.delays.close = parseInt(document.getElementById('delay-close').value) || 600;
    gripperConfig.delays.up = parseInt(document.getElementById('delay-up').value) || 600;
    gripperConfig.delays.down = parseInt(document.getElementById('delay-down').value) || 600;
});
document.getElementById('gripper-modal').addEventListener('click', e => {
    if (e.target.id === 'gripper-modal') {
        e.target.classList.add('hidden');
        gripperConfig.delays.open = parseInt(document.getElementById('delay-open').value) || 600;
        gripperConfig.delays.close = parseInt(document.getElementById('delay-close').value) || 600;
        gripperConfig.delays.up = parseInt(document.getElementById('delay-up').value) || 600;
        gripperConfig.delays.down = parseInt(document.getElementById('delay-down').value) || 600;
    }
});
document.querySelectorAll('input[name="gripper-mode"]').forEach(radio => {
    radio.addEventListener('change', e => {
        gripperConfig.mode = e.target.value;
        document.getElementById('sim-delays-section').style.display = gripperConfig.mode === 'sim' ? '' : 'none';
        document.getElementById('real-sensor-info').style.display = gripperConfig.mode === 'real' ? '' : 'none';
    });
});

// --- Gripper Sequence Helpers ---
function msDelay(ms) {
    // Guard: NaN / undefined / negative → setTimeout fires immediately, which
    // causes the path sequencer to skip its inter-waypoint wait.
    const safe = (Number.isFinite(ms) && ms > 0) ? ms : 0;
    return new Promise(r => setTimeout(r, safe));
}

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
}

async function runGripperPick() {
    await gripperStep('CMD:CLAW_OPEN',  () => !state.gripper_co, gripperConfig.delays.open);
    await gripperStep('CMD:GRIP_DN',    () => state.gripper_ud,  gripperConfig.delays.down);
    await gripperStep('CMD:CLAW_CLOSE', () => state.gripper_co,  gripperConfig.delays.close);
    await gripperStep('CMD:GRIP_UP',    () => !state.gripper_ud, gripperConfig.delays.up);
}

async function runGripperPlace() {
    await gripperStep('CMD:GRIP_DN',    () => state.gripper_ud,  gripperConfig.delays.down);
    await gripperStep('CMD:CLAW_OPEN',  () => !state.gripper_co, gripperConfig.delays.open);
    await gripperStep('CMD:GRIP_UP',    () => !state.gripper_ud, gripperConfig.delays.up);
    await gripperStep('CMD:CLAW_CLOSE', () => state.gripper_co,  gripperConfig.delays.close);
}

// --- Path Sequencer (Live mode) ---
const waypointList = document.getElementById('waypoint-list');
const btnRunSeq = document.getElementById('btn-run-seq');
const btnLoopSeq = document.getElementById('btn-loop-seq');
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
        state.gripperHasRod = false;  // start each Run with empty gripper
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

    const rawDelay = parseFloat(document.getElementById('input-seq-delay').value);
    const movDelay = (Number.isFinite(rawDelay) && rawDelay >= 0) ? rawDelay * 1000 : 2000;
    await msDelay(movDelay);
    if (!state.seqActive) return;

    if (gripperConfig.enabled) {
        // Single-rod shuttle. Action at each waypoint depends on whether the
        // gripper is currently holding the rod, not on waypoint index — so it
        // works across loops without dropping the rod.
        //
        //   empty gripper  → PICK (grab here)
        //   holding rod, last waypoint, no loop → PLACE (final drop)
        //   holding rod, otherwise              → PLACE then PICK
        const idx = state.currentWaypointIdx;
        const isLast = (idx === state.waypoints.length - 1);

        if (!state.gripperHasRod) {
            await runGripperPick();
            state.gripperHasRod = true;
        } else if (isLast && !state.seqLoop) {
            await runGripperPlace();
            state.gripperHasRod = false;
        } else {
            await runGripperPlace();
            state.gripperHasRod = false;
            if (!state.seqActive) return;
            await runGripperPick();
            state.gripperHasRod = true;
        }
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
    'input-speed-kp': 1.0,
    'input-speed-ki': 2.0,
    'input-speed-kd': 0.0,
    'input-k-vff': 0.0,
    'input-k-aff': 0.0,
    'input-k-tff': 0.0,
    'input-pos-kp': 0.4,
    'input-pos-ki': 0.05,
    'input-pos-kd': 0.1,
    'input-min-pwm': 0.0,
    'input-home-speed': 30,
    'input-jog-fine': 10,
    'input-vmax-rad': 7.304,
    'input-amax-rad': 27.49,
    'input-jmax-rad': 1400,
    'input-step-coarse': 10,
    'input-step-fine': 1.0,
    'input-shaper-wn': 12.13,
    'input-shaper-zeta': 0.041,
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

document.addEventListener('pointerdown', e => {
    if (e.target.matches('input[type="number"], input[type="text"]')) {
        const el = e.target;
        requestAnimationFrame(() => el.select());
    }
});

updateUI();

/* ============================================================================
 * Chart freezing toggle
 * ========================================================================== */
(function setupChartFreezing() {
    const freezeBtn = document.getElementById('btn-freeze-charts');
    if (!freezeBtn) return;

    const charts = [chartPos, chartVel, chartAcc];
    let isFrozen = false;

    freezeBtn.addEventListener('click', () => {
        isFrozen = !isFrozen;
        
        charts.forEach(c => c.paused = isFrozen);
        
        if (isFrozen) {
            freezeBtn.innerText = 'Resume';
            freezeBtn.classList.add('active');
        } else {
            freezeBtn.innerText = 'Freeze';
            freezeBtn.classList.remove('active');
            // Force a redraw to catch up to background data immediately
            charts.forEach(c => c.draw());
        }
    });

    document.getElementById('btn-run-diag').addEventListener('click', () => {
        if (!state.connected) { log('Connect the robot before running the self-test.', 'warn'); return; }
        readiness.diag = 'running';
        readiness.diagText = 'Running self-test…';
        readiness.diagHint = '';
        updateReadinessUI();
        sendCommand('CMD:DIAG');
    });
})();

/* ============================================================================
 * Mission Readiness state machine, firmware-line parsing & motion gate
 * (RD_GATED_IDS is hoisted to the top of the file — see initial declaration —
 *  because updateUI() runs at load before this point and reads it via the gate.)
 * ========================================================================== */

function readinessReady() {
    return readiness.diag === 'pass' && readiness.home === 'done';
}
function motionAllowed() {
    return state.override || readinessReady();
}

function setStep(id, status) {
    const el = document.getElementById(id);
    if (el) el.className = 'rd-step rd-' + status;
}

function applyMotionGate() {
    const locked = !motionAllowed();
    RD_GATED_IDS.forEach(id => {
        const el = document.getElementById(id);
        if (!el) return;
        el.disabled = locked;
        el.classList.toggle('gate-locked', locked);
        if (locked) el.title = 'Locked — run HW Self-Test + Home, or enable Override.';
        else if (el.title && el.title.startsWith('Locked')) el.removeAttribute('title');
    });
}

function updateReadinessUI() {
    setStep('rd-step-connect', state.connected ? 'pass' : 'idle');
    setStep('rd-step-diag', readiness.diag);
    const homeStatus = readiness.home === 'done' ? 'pass'
        : readiness.home === 'fail' ? 'fail'
        : readiness.home === 'homing' ? 'running' : 'idle';
    setStep('rd-step-home', homeStatus);

    const badge = document.getElementById('rd-ready-badge');
    if (badge) {
        if (state.override)       { badge.textContent = 'OVERRIDE — gate bypassed'; badge.className = 'rd-ready-badge override'; }
        else if (state.positionUnknown) { badge.textContent = 'RE-HOME REQUIRED'; badge.className = 'rd-ready-badge warn'; }
        else if (readinessReady()) { badge.textContent = 'READY'; badge.className = 'rd-ready-badge ready'; }
        else                       { badge.textContent = 'MOTION LOCKED'; badge.className = 'rd-ready-badge not-ready'; }
    }

    const dt = document.getElementById('rd-diag-text');
    if (dt) dt.textContent = readiness.diagText;
    const hintRow = document.getElementById('rd-diag-hint-row');
    const hint = document.getElementById('rd-diag-hint');
    if (hint) hint.textContent = readiness.diagHint;
    if (hintRow) hintRow.style.display = readiness.diagHint ? '' : 'none';
    const ht = document.getElementById('rd-home-text');
    if (ht) ht.textContent = readiness.homeText;

    const diagBtn = document.getElementById('btn-run-diag');
    const homeBtn = document.getElementById('btn-readiness-home');
    if (diagBtn) diagBtn.disabled = !state.connected || readiness.diag === 'running';
    if (homeBtn) homeBtn.disabled = !state.connected || readiness.home === 'homing';

    applyMotionGate();
}

function resetReadiness() {
    readiness.diag = 'idle';
    readiness.diagText = 'Not run yet.';
    readiness.diagHint = '';
    readiness.home = 'no';
    readiness.homeText = 'Not homed.';
    _faultLatched = false;
}

/* Fresh session on (re)connect — hardware must be re-verified. */
function onSerialConnected() {
    resetReadiness();
    updateReadinessUI();
}

/* A new fault / E-Stop invalidates the home (encoder phase may have shifted). */
function onFaultOrEstop() {
    _faultLatched = true;
    if (readiness.home === 'done' || readiness.home === 'homing') {
        readiness.home = 'no';
        readiness.homeText = 'Re-home required — a fault / E-Stop occurred.';
    }
    if (typeof updateReadinessUI === 'function') updateReadinessUI();
}

/* Parse the plain-text [DIAG] / [HOMING] firmware lines into readiness state. */
function handleFirmwareLine(line) {
    if (line.includes('[DIAG] Starting')) {
        readiness.diag = 'running';
        readiness.diagText = 'Running self-test…';
        readiness.diagHint = '';
        readiness._deltas = '';
        updateReadinessUI();
        return;
    }
    if (line.includes('[DIAG] Fwd Delta')) {
        const m = line.split(']')[1];
        readiness._deltas = m ? m.trim() : '';
        return;
    }
    if (line.includes('[DIAG] RESULT:')) {
        const r = line.split('RESULT:')[1].trim();
        if (r.startsWith('HARDWARE OK')) {
            readiness.diag = 'pass';
            readiness.diagText = '✓ Hardware OK — motion matches commands.';
            readiness.diagHint = '';
        } else if (r.startsWith('ENCODER DEAD')) {
            readiness.diag = 'fail';
            readiness.diagText = '✗ Encoder dead — no movement detected during the test.';
            readiness.diagHint = 'Check: encoder cable, TIM3 CH1/CH2 wiring, and 5 V supply to the encoder.';
        } else if (r.startsWith('DIRECTION PIN STUCK')) {
            readiness.diag = 'fail';
            readiness.diagText = '✗ Direction pin stuck — motor turned the same way both times.';
            readiness.diagHint = 'Check: H-bridge DIR/IN pins and the PWM sign in motor_controller.';
        } else if (r.startsWith('PHASE INVERTED')) {
            readiness.diag = 'fail';
            readiness.diagText = '✗ Phase inverted — encoder counts opposite to PWM.';
            readiness.diagHint = 'Fix: swap encoder A/B, or swap the two motor leads (or invert in firmware).';
        } else {
            readiness.diag = 'fail';
            readiness.diagText = '✗ ' + r;
            readiness.diagHint = '';
        }
        if (readiness._deltas) readiness.diagText += '  (' + readiness._deltas + ')';
        updateReadinessUI();
        return;
    }
    if (line.includes('[HOMING] SUCCESS')) {
        readiness.home = 'done';
        const detail = (line.split('SUCCESS.')[1] || '').trim();
        readiness.homeText = '✓ Homed.' + (detail ? ' ' + detail : '');
        _faultLatched = false;
        updateReadinessUI();
        return;
    }
    if (line.includes('[HOMING] ERROR') || line.includes('[HOMING] ABORTED') || line.includes('Verify abort')) {
        readiness.home = 'fail';
        readiness.homeText = '✗ ' + line.replace(/^\[HOMING\]\s*/, '');
        updateReadinessUI();
        return;
    }
}

/* Homing entry point with the force-confirm guard (only after a fault/E-Stop). */
function requestHoming() {
    if (!state.connected) { log('Connect the robot before homing.', 'warn'); return; }
    const needsConfirm = _faultLatched || state.estop || state.fault !== 'NONE';
    if (needsConfirm) {
        document.getElementById('home-confirm-modal').classList.remove('hidden');
    } else {
        triggerHoming();
    }
}

/* Wire readiness-bar controls. */
(function setupReadinessControls() {
    const homeBtn = document.getElementById('btn-readiness-home');
    if (homeBtn) homeBtn.addEventListener('click', () => requestHoming());

    const detailBtn = document.getElementById('rd-detail-btn');
    const detailPanel = document.getElementById('rd-detail-panel');
    if (detailBtn && detailPanel) {
        detailBtn.addEventListener('click', () => {
            const open = detailPanel.classList.toggle('hidden') === false;
            detailBtn.textContent = open ? 'Details ▴' : 'Details ▾';
        });
    }

    const modal = document.getElementById('home-confirm-modal');
    const close = () => modal.classList.add('hidden');
    document.getElementById('btn-close-home-confirm').addEventListener('click', close);
    document.getElementById('btn-home-confirm-cancel').addEventListener('click', close);
    document.getElementById('btn-home-confirm-go').addEventListener('click', () => {
        close();
        triggerHoming();
    });
    modal.addEventListener('click', e => { if (e.target.id === 'home-confirm-modal') close(); });
})();

updateReadinessUI();
