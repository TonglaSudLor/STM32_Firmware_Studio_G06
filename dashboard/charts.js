class TelemetryChart {
    constructor(canvasId, color, minVal, maxVal, tuningMinVal, tuningMaxVal, absInTuning) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.history = [];
        this.targetHistory = [];
        this.maxDataPoints = 200;
        this.color = color || '#00f2ff';
        this.minVal = (minVal !== undefined && minVal !== null) ? minVal : -10;
        this.maxVal = (maxVal !== undefined && maxVal !== null) ? maxVal : 360;
        // Optional Y-axis range override for tuning mode only
        this.tuningMinVal = tuningMinVal;
        this.tuningMaxVal = tuningMaxVal;
        this.absInTuning  = !!absInTuning;

        // Tuning mode
        this.tuningMode = false;
        this.tuningRuns = [];        // [{times[], vals[], vsets[], target, settleTime, overshoot}]
        this.currentRun = null;

        window.addEventListener('resize', () => this.resize());
        this.resize();
    }

    resize() {
        const container = this.canvas.parentElement;
        if (!container) return;
        this.canvas.width = container.clientWidth;
        this.canvas.height = container.clientHeight;
        this.draw();
    }

    setMode(tuning) {
        this.tuningMode = tuning;
        this.draw();
    }

    // Helpers: effective Y range in tuning mode
    _effMin() {
        return (this.tuningMode && this.tuningMinVal !== undefined) ? this.tuningMinVal : this.minVal;
    }
    _effMax() {
        return (this.tuningMode && this.tuningMaxVal !== undefined) ? this.tuningMaxVal : this.maxVal;
    }
    _xform(v) {
        return (this.tuningMode && this.absInTuning) ? Math.abs(v) : v;
    }

    // --- Live mode ---
    addData(val) {
        if (this.canvas.width === 0 || this.canvas.height === 0) this.resize();
        this.history.push({ value: val });
        if (this.history.length > this.maxDataPoints) this.history.shift();
        if (!this.tuningMode) this.draw();
    }

    addTarget(val) {
        this.targetHistory.push({ value: val });
        if (this.targetHistory.length > this.maxDataPoints) this.targetHistory.shift();
    }

    // --- Tuning mode ---
    startRun(target) {
        this.currentRun = { times: [], vals: [], vsets: [], target, startTime: Date.now() };
    }

    addRunPoint(val, vset) {
        if (!this.currentRun) return;
        const t = (Date.now() - this.currentRun.startTime) / 1000;
        this.currentRun.times.push(t);
        this.currentRun.vals.push(val);
        this.currentRun.vsets.push(vset !== undefined ? vset : 0);
        if (this.tuningMode) this.draw();
    }

    finalizeRun(settleTime, overshoot) {
        if (!this.currentRun) return;
        this.tuningRuns.push({ ...this.currentRun, settleTime, overshoot });
        if (this.tuningRuns.length > 3) this.tuningRuns.shift();
        this.currentRun = null;
        if (this.tuningMode) this.draw();
    }

    clearRuns() {
        this.tuningRuns = [];
        this.currentRun = null;
        this.draw();
    }

    // --- Shared draw ---
    draw() {
        if (this.tuningMode) this._drawTuning();
        else this._drawLive();
    }

    _drawLine(history, color, dashed, lineWidth) {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const range = this.maxVal - this.minVal;
        const xStep = width / (this.maxDataPoints - 1);
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth || 2;
        ctx.lineJoin = 'round';
        if (dashed) ctx.setLineDash([6, 4]);
        else ctx.setLineDash([]);
        history.forEach((point, i) => {
            const x = i * xStep;
            let val = Math.min(Math.max(point.value, this.minVal), this.maxVal);
            const y = height - ((val - this.minVal) / range) * height;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }

    _drawLive() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const range = this.maxVal - this.minVal;
        ctx.clearRect(0, 0, width, height);
        this._drawGrid(width, height, range);

        if (this.history.length < 2) return;
        if (this.targetHistory.length >= 2)
            this._drawLine(this.targetHistory, 'rgba(255,255,255,0.35)', true, 1.5);
        this._drawLine(this.history, this.color, false, 2);

        ctx.fillStyle = this.color;
        ctx.font = 'bold 11px "JetBrains Mono"';
        const lastVal = this.history[this.history.length - 1].value;
        const text = isNaN(lastVal) ? '--' : lastVal.toFixed(1);
        const tw = ctx.measureText(text).width;
        ctx.fillText(text, width - tw - 5, 15);

        if (this.targetHistory.length > 0) {
            const lt = this.targetHistory[this.targetHistory.length - 1].value;
            const tt = 'T:' + (isNaN(lt) ? '--' : lt.toFixed(1));
            ctx.fillStyle = 'rgba(255,255,255,0.5)';
            ctx.font = '9px "JetBrains Mono"';
            ctx.fillText(tt, width - ctx.measureText(tt).width - 5, 28);
        }
    }

    _drawTuning() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const minV = this._effMin();
        const maxV = this._effMax();
        const range = maxV - minV;
        ctx.clearRect(0, 0, width, height);

        // Compute time window
        let maxT = 10;
        this.tuningRuns.forEach(r => { if (r.times.length) maxT = Math.max(maxT, r.times[r.times.length - 1] + 1); });
        if (this.currentRun && this.currentRun.times.length)
            maxT = Math.max(maxT, this.currentRun.times[this.currentRun.times.length - 1] + 1);

        this._drawGrid(width, height, range);

        // Time axis labels
        ctx.fillStyle = 'rgba(255,255,255,0.4)';
        ctx.font = '9px "JetBrains Mono"';
        for (let s = 0; s <= maxT; s += 2) {
            const x = (s / maxT) * width;
            ctx.fillText(s + 's', x + 2, height - 2);
        }

        // Ghost runs (older = more faded)
        const runAlphas = [0.05, 0.12, 0.22];
        this.tuningRuns.forEach((run, ri) => {
            const alpha = runAlphas[Math.min(ri, runAlphas.length - 1)];
            const col = this._hexToRgba(this.color, alpha);
            this._drawTuningLine(run.times, run.vals, maxT, col, false, 1.5, width, height, range, minV, maxV);

            // Target line for this run
            if (run.target !== undefined) {
                const t = this._xform(run.target);
                const ty = height - ((Math.min(Math.max(t, minV), maxV) - minV) / range) * height;
                ctx.beginPath();
                ctx.strokeStyle = `rgba(255,255,255,${alpha * 1.5})`;
                ctx.setLineDash([4, 6]);
                ctx.lineWidth = 1;
                ctx.moveTo(0, ty); ctx.lineTo(width, ty);
                ctx.stroke();
                ctx.setLineDash([]);
            }

            // Settling marker
            if (run.settleTime !== null && run.settleTime !== undefined) {
                const sx = (run.settleTime / maxT) * width;
                ctx.beginPath();
                ctx.strokeStyle = `rgba(100,255,100,${alpha * 2})`;
                ctx.setLineDash([3, 4]);
                ctx.lineWidth = 1;
                ctx.moveTo(sx, 0); ctx.lineTo(sx, height);
                ctx.stroke();
                ctx.setLineDash([]);
            }
        });

        // Current run (bright)
        if (this.currentRun) {
            // Target bar (bright magenta dashed)
            if (this.currentRun.target !== undefined && this.currentRun.target !== 0) {
                const ct = this._xform(this.currentRun.target);
                const ty = height - ((Math.min(Math.max(ct, minV), maxV) - minV) / range) * height;
                ctx.beginPath();
                ctx.strokeStyle = '#ff4ddd';
                ctx.setLineDash([8, 5]);
                ctx.lineWidth = 1.5;
                ctx.moveTo(0, ty); ctx.lineTo(width, ty);
                ctx.stroke();
                ctx.setLineDash([]);
                // Target label
                ctx.fillStyle = '#ff4ddd';
                ctx.font = 'bold 10px "JetBrains Mono"';
                const lbl = 'TARGET ' + this.currentRun.target.toFixed(1);
                ctx.fillText(lbl, 8, ty - 4);
            }
            if (this.currentRun.times.length >= 2) {
                this._drawTuningLine(this.currentRun.times, this.currentRun.vals, maxT, this.color, false, 2, width, height, range, minV, maxV);
            }

            // Current value label
            if (this.currentRun.vals.length > 0) {
                const lastV = this.currentRun.vals[this.currentRun.vals.length - 1];
                ctx.fillStyle = this.color;
                ctx.font = 'bold 11px "JetBrains Mono"';
                const ltext = isNaN(lastV) ? '--' : lastV.toFixed(1);
                ctx.fillText(ltext, width - ctx.measureText(ltext).width - 5, 15);
            }
        }

        // Scale labels
        ctx.fillStyle = 'rgba(255,255,255,0.5)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.fillText(maxV, 5, 10);
        ctx.fillText(minV, 5, height - 14);

        // "TUNING" watermark
        ctx.fillStyle = 'rgba(255,200,0,0.08)';
        ctx.font = 'bold 28px "JetBrains Mono"';
        ctx.fillText('TUNING', 8, height / 2 + 10);
    }

    _drawTuningLine(times, vals, maxT, color, dashed, lw, width, height, range, minV, maxV) {
        if (times.length < 2) return;
        if (minV === undefined) minV = this.minVal;
        if (maxV === undefined) maxV = this.maxVal;
        const ctx = this.ctx;
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = lw;
        ctx.lineJoin = 'round';
        if (dashed) ctx.setLineDash([6, 4]); else ctx.setLineDash([]);
        times.forEach((t, i) => {
            const x = (t / maxT) * width;
            let val = Math.min(Math.max(this._xform(vals[i]), minV), maxV);
            const y = height - ((val - minV) / range) * height;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }

    _drawGrid(width, height, range) {
        const ctx = this.ctx;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
        for (let i = 0; i <= 4; i++) { const y = (height / 4) * i; ctx.moveTo(0, y); ctx.lineTo(width, y); }
        ctx.stroke();
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255,255,255,0.3)';
        ctx.moveTo(1, 0); ctx.lineTo(1, height);
        ctx.stroke();
        const zeroY = height - ((0 - this.minVal) / range) * height;
        if (zeroY >= 0 && zeroY <= height) {
            ctx.beginPath();
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.moveTo(0, zeroY); ctx.lineTo(width, zeroY);
            ctx.stroke();
        }
    }

    _hexToRgba(hex, alpha) {
        const c = hex.replace('#', '');
        if (c.length === 6) {
            const r = parseInt(c.slice(0,2),16), g = parseInt(c.slice(2,4),16), b = parseInt(c.slice(4,6),16);
            return `rgba(${r},${g},${b},${alpha})`;
        }
        return hex;
    }
}
