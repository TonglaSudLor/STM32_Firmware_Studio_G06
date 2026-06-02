/**
 * charts.js — TelemetryChart
 *
 * Live mode  : uPlot (fast, auto-scales, proper time axis, built-in zoom/pan)
 * Tuning mode: canvas overlay (ghost runs, A/B pins, settling markers — all custom)
 *
 * Public API is unchanged from the canvas-only version so app.js needs no rework
 * except the two `chartVel/Acc.history = []` lines → `clearLiveData()`.
 */
class TelemetryChart {
    constructor(mountId, color, liveMin, liveMax, tuningMinVal, tuningMaxVal, absInTuning) {
        this._mountEl = document.getElementById(mountId);
        this.color        = color || '#00f2ff';
        this.minVal       = (liveMin  != null) ? liveMin  : -10;
        this.maxVal       = (liveMax  != null) ? liveMax  : 360;
        this.tuningMinVal = tuningMinVal;
        this.tuningMaxVal = tuningMaxVal;
        this.absInTuning  = !!absInTuning;

        // ── Public state (API-compatible) ────────────────────────────────────
        this.paused          = false;
        this.tuningMode      = false;
        this.scrollWindowSec = null;
        this.pinnedRuns      = { A: null, B: null };
        this.tuningRuns      = [];
        this.currentRun      = null;
        this.previewSine     = null;

        // ── Live rolling buffer (4 series + timestamps) ──────────────────────
        this._MAX = 200;
        this._ts  = [];
        this._act = [];   // actual value  (main line)
        this._tar = [];   // target / setpoint
        this._kf  = [];   // Kalman estimate
        this._san = [];   // open-loop model sanity

        // ── Tuning overlay canvas (created in JS, shown only in tuning mode) ──
        this.canvas = document.createElement('canvas');
        this.canvas.style.cssText =
            'display:none; position:absolute; top:0; left:0; z-index:5; pointer-events:none;';
        this.ctx = this.canvas.getContext('2d');

        // Current-value label — simple HTML, no coordinate system headaches
        this._valLabel = document.createElement('div');
        this._valLabel.style.cssText =
            'position:absolute; top:4px; right:6px; font:bold 11px "JetBrains Mono",monospace;' +
            'color:' + this._cssColor() + '; pointer-events:none; z-index:10; line-height:1.3;';
        this._tarLabel = document.createElement('div');
        this._tarLabel.style.cssText =
            'position:absolute; top:18px; right:6px; font:9px "JetBrains Mono",monospace;' +
            'color:rgba(255,255,255,0.5); pointer-events:none; z-index:10;';

        // Mount el needs relative positioning for the absolute canvas overlay
        this._mountEl.classList.add('uplot-mount');

        // ── uPlot live chart ─────────────────────────────────────────────────
        this._uplot = this._createUplot();

        // Overlays go on top of uPlot
        this._mountEl.appendChild(this.canvas);
        this._mountEl.appendChild(this._valLabel);
        this._mountEl.appendChild(this._tarLabel);

        // ── Responsive resize via ResizeObserver ─────────────────────────────
        this._ro = new ResizeObserver(() => this._onResize());
        this._ro.observe(this._mountEl);

        // RAF batching — avoids redundant uPlot redraws within the same frame
        this._rafPending = false;
    }

    // ── Private helpers ──────────────────────────────────────────────────────

    _cssColor() {
        const map = { cyan: '#00f2ff', magenta: '#ff4ddd', yellow: '#ffe44d' };
        return map[this.color] || this.color;
    }

    _createUplot() {
        const w = Math.max(100, this._mountEl.clientWidth  || 300);
        const h = Math.max(40,  this._mountEl.clientHeight || 100);
        const stroke = this._cssColor();

        const opts = {
            width:  w,
            height: h,
            padding: [4, 10, 0, 0],
            cursor: { show: true, drag: { x: true, y: false, setScale: true } },
            select: { show: true },
            legend: { show: false },
            series: [
                {},
                { stroke, width: 2 },
                { stroke: 'rgba(255,255,255,0.35)', width: 1.5, dash: [6, 4] },
                { stroke: '#00ff88', width: 1.8, spanGaps: false },
                { stroke: 'rgba(255,200,0,0.7)', width: 1.5, dash: [6, 4], spanGaps: false },
            ],
            axes: [
                {
                    stroke: 'rgba(255,255,255,0.3)',
                    grid:   { stroke: 'rgba(255,255,255,0.06)', width: 1 },
                    ticks:  { stroke: 'rgba(255,255,255,0.06)', width: 1, size: 3 },
                    font:   '9px "JetBrains Mono", monospace',
                    gap:    2,
                    size:   16,
                    values: (u, ticks) => {
                        const arr  = u.data[0];
                        const last = arr && arr.length ? arr[arr.length - 1] : 0;
                        return ticks.map(t => {
                            const d = t - last;
                            return d === 0 ? 'now' : d.toFixed(1) + 's';
                        });
                    },
                },
                {
                    stroke: 'rgba(255,255,255,0.3)',
                    grid:   { stroke: 'rgba(255,255,255,0.06)', width: 1 },
                    ticks:  { stroke: 'rgba(255,255,255,0.06)', width: 1, size: 3 },
                    font:   '9px "JetBrains Mono", monospace',
                    gap:    2,
                    size:   42,
                    values: (u, ticks) => ticks.map(v =>
                        (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1))
                    ),
                },
            ],
            scales: {
                x: { time: false },
                y: { range: [this.minVal, this.maxVal] },
            },
        };

        return new uPlot(opts, [[], [], [], [], []], this._mountEl);
    }

    _onResize() {
        const w = Math.max(100, this._mountEl.clientWidth  || 300);
        const h = Math.max(40,  this._mountEl.clientHeight || 100);
        if (this._uplot) this._uplot.setSize({ width: w, height: h });
        this.canvas.width  = w;
        this.canvas.height = h;
        if (this.tuningMode) this._drawTuning();
    }

    _setUplotData() {
        this._uplot.setData([this._ts, this._act, this._tar, this._kf, this._san]);
    }

    // ── Public: Live mode ────────────────────────────────────────────────────

    /** Push a new live sample.  Always call before addTarget/addEstimate/addSanity. */
    addData(val) {
        this._ts.push(performance.now() / 1000);
        this._act.push(val);
        // Pad companion arrays to same length (overwritten by the add* calls below)
        this._tar.push(this._tar.length ? this._tar[this._tar.length - 1] : 0);
        this._kf.push(NaN);
        this._san.push(NaN);

        if (this._ts.length > this._MAX) {
            this._ts.shift(); this._act.shift(); this._tar.shift();
            this._kf.shift(); this._san.shift();
        }

        if (!this.paused && !this.tuningMode) {
            this._pendingVal = val;   // consumed by the RAF render, not written here
            this._scheduleUplotRender();
        }
    }

    // Defers uPlot.setData to the next animation frame. Multiple addData() calls
    // within the same 16 ms frame collapse into a single render.
    _scheduleUplotRender() {
        if (this._rafPending) return;
        this._rafPending = true;
        requestAnimationFrame(() => {
            this._rafPending = false;
            if (!this.paused && !this.tuningMode) {
                this._setUplotData();
                const v = this._pendingVal;
                this._valLabel.textContent = (v === undefined || isNaN(v)) ? '--' : v.toFixed(1);
            }
        });
    }

    addTarget(val) {
        if (this._tar.length) this._tar[this._tar.length - 1] = val;
        if (!this.tuningMode) this._tarLabel.textContent = 'T:' + (isNaN(val) ? '--' : val.toFixed(1));
    }

    addEstimate(val) {
        if (this._kf.length) this._kf[this._kf.length - 1] = val;
    }

    addSanity(val) {
        if (this._san.length) this._san[this._san.length - 1] = val;
    }

    clearEstimate() {
        for (let i = 0; i < this._kf.length; i++) this._kf[i] = NaN;
        if (!this.paused && !this.tuningMode) this._setUplotData();
    }

    clearSanity() {
        for (let i = 0; i < this._san.length; i++) this._san[i] = NaN;
        if (!this.paused && !this.tuningMode) this._setUplotData();
    }

    /**
     * history getter/setter — API compat for `chartVel.history = []` in app.js.
     * Prefer clearLiveData() in new code.
     */
    get history() { return this._act; }
    set history(_) {
        this._ts = []; this._act = []; this._tar = []; this._kf = []; this._san = [];
        if (!this.tuningMode) {
            this._uplot.setData([[], [], [], [], []]);
            this._valLabel.textContent = '--';
            this._tarLabel.textContent = '';
        }
    }

    /** Wipe the live rolling buffer and blank the chart. */
    clearLiveData() {
        this.history = null;  // triggers setter above
    }

    setPreviewSine(amp, freq) {
        this.previewSine = (amp && freq > 0) ? { amp, freq } : null;
        if (this.tuningMode) this.draw();
    }
    clearPreviewSine() { this.previewSine = null; if (this.tuningMode) this.draw(); }

    // ── Public: Mode switch & draw ───────────────────────────────────────────

    setMode(tuning) {
        this.tuningMode = tuning;
        if (tuning) {
            if (this._uplot?.root) this._uplot.root.style.display = 'none';
            this._valLabel.style.display = 'none';
            this._tarLabel.style.display = 'none';
            this.canvas.style.display = '';
            this.canvas.width  = Math.max(1, this._mountEl.clientWidth  || 300);
            this.canvas.height = Math.max(1, this._mountEl.clientHeight || 100);
        } else {
            this.canvas.style.display = 'none';
            if (this._uplot?.root) this._uplot.root.style.display = '';
            this._valLabel.style.display = '';
            this._tarLabel.style.display = '';
            this._setUplotData();
        }
        this.draw();
    }

    draw() {
        if (this.paused) return;
        if (this.tuningMode) this._drawTuning();
        else this._setUplotData();
    }

    resize() { this._onResize(); }

    // ── Public: Tuning mode ──────────────────────────────────────────────────

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

    // ── Tuning canvas helpers ────────────────────────────────────────────────

    _effMin() {
        return (this.tuningMode && this.tuningMinVal !== undefined) ? this.tuningMinVal : this.minVal;
    }
    _effMax() {
        return (this.tuningMode && this.tuningMaxVal !== undefined) ? this.tuningMaxVal : this.maxVal;
    }
    _xform(v) {
        return (this.tuningMode && this.absInTuning) ? Math.abs(v) : v;
    }

    // ── Tuning canvas draw (identical to original) ───────────────────────────

    _drawTuning() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const minV = this._effMin();
        const maxV = this._effMax();
        const range = maxV - minV;
        ctx.clearRect(0, 0, width, height);

        let minT = 0;
        let maxT = 10;
        const lastT = (this.currentRun && this.currentRun.times.length)
            ? this.currentRun.times[this.currentRun.times.length - 1]
            : 0;
        const scrolling = this.scrollWindowSec && this.scrollWindowSec > 0;
        if (scrolling) {
            const win = this.scrollWindowSec;
            maxT = Math.max(win, lastT);
            minT = Math.max(0, maxT - win);
        } else {
            this.tuningRuns.forEach(r => { if (r.times.length) maxT = Math.max(maxT, r.times[r.times.length - 1] + 1); });
            if (this.currentRun && this.currentRun.times.length)
                maxT = Math.max(maxT, lastT + 1);
        }

        this._drawGrid(width, height, range);
        this._drawSinePreview(width, height, minV, maxV);

        ctx.fillStyle = 'rgba(255,255,255,0.4)';
        ctx.font = '9px "JetBrains Mono"';
        const span = maxT - minT;
        const step = span <= 6 ? 1 : (span <= 20 ? 2 : 5);
        for (let s = Math.ceil(minT / step) * step; s <= maxT; s += step) {
            const x = ((s - minT) / span) * width;
            ctx.fillText(s + 's', x + 2, height - 2);
        }

        const lastIdx = this.tuningRuns.length - 1;
        const alphaFor = (ri) => (ri === lastIdx) ? 0.55 : (ri === lastIdx - 1 ? 0.25 : 0.10);
        if (!scrolling) this.tuningRuns.forEach((run, ri) => {
            const alpha = alphaFor(ri);
            const col = this._hexToRgba(this._cssColor(), alpha);
            this._drawTuningLine(run.times, run.vals, maxT, col, false, 1.5, width, height, range, minV, maxV, minT);

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

            if (run.settleTime !== null && run.settleTime !== undefined) {
                const sx = ((run.settleTime - minT) / span) * width;
                ctx.beginPath();
                ctx.strokeStyle = `rgba(100,255,100,${alpha * 2})`;
                ctx.setLineDash([3, 4]);
                ctx.lineWidth = 1;
                ctx.moveTo(sx, 0); ctx.lineTo(sx, height);
                ctx.stroke();
                ctx.setLineDash([]);
            }
        });

        if (!scrolling) {
            const pinColors = { A: '#ffb000', B: '#39ff14' };
            ['A', 'B'].forEach(slot => {
                const run = this.pinnedRuns[slot];
                if (!run || !run.times || run.times.length < 2) return;
                this._drawTuningLine(run.times, run.vals, maxT, pinColors[slot], false, 2,
                    width, height, range, minV, maxV, minT);
                const sx = ((run.times[0] - minT) / span) * width;
                const sy = height - ((Math.min(Math.max(this._xform(run.vals[0]), minV), maxV) - minV) / range) * height;
                ctx.fillStyle = pinColors[slot];
                ctx.font = 'bold 11px monospace';
                ctx.fillText(slot, sx + 2, Math.max(11, sy - 3));
            });
        }

        if (this.currentRun) {
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
                ctx.fillStyle = '#ff4ddd';
                ctx.font = 'bold 10px "JetBrains Mono"';
                const lbl = 'TARGET ' + this.currentRun.target.toFixed(1);
                ctx.fillText(lbl, 8, ty - 4);
            }
            if (this.currentRun.times.length >= 2) {
                if (this.currentRun.vsets && this.currentRun.vsets.length >= 2) {
                    this._drawTuningLine(this.currentRun.times, this.currentRun.vsets, maxT,
                        'rgba(255,255,255,0.55)', true, 1.5,
                        width, height, range, minV, maxV, minT);
                }
                this._drawTuningLine(this.currentRun.times, this.currentRun.vals, maxT,
                    this._cssColor(), false, 2, width, height, range, minV, maxV, minT);
            }

            if (this.currentRun.vals.length > 0) {
                const lastV = this.currentRun.vals[this.currentRun.vals.length - 1];
                ctx.fillStyle = this._cssColor();
                ctx.font = 'bold 11px "JetBrains Mono"';
                const ltext = isNaN(lastV) ? '--' : lastV.toFixed(1);
                ctx.fillText(ltext, width - ctx.measureText(ltext).width - 5, 15);
            }
        }

        ctx.fillStyle = 'rgba(255,255,255,0.5)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.fillText(maxV, 5, 10);
        ctx.fillText(minV, 5, height - 14);

        ctx.fillStyle = 'rgba(255,200,0,0.08)';
        ctx.font = 'bold 28px "JetBrains Mono"';
        ctx.fillText('TUNING', 8, height / 2 + 10);
    }

    _drawTuningLine(times, vals, maxT, color, dashed, lw, width, height, range, minV, maxV, minT) {
        if (times.length < 2) return;
        if (minV === undefined) minV = this.minVal;
        if (maxV === undefined) maxV = this.maxVal;
        if (minT === undefined) minT = 0;
        const span = maxT - minT;
        if (span <= 0) return;
        const ctx = this.ctx;
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = lw;
        ctx.lineJoin = 'round';
        if (dashed) ctx.setLineDash([6, 4]); else ctx.setLineDash([]);
        let started = false;
        times.forEach((t, i) => {
            if (t < minT) return;
            const x = ((t - minT) / span) * width;
            let val = Math.min(Math.max(this._xform(vals[i]), minV), maxV);
            const y = height - ((val - minV) / range) * height;
            if (!started) { ctx.moveTo(x, y); started = true; }
            else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }

    _drawGrid(width, height, range) {
        const ctx = this.ctx;
        const minV = this._effMin();
        const maxV = this._effMax();
        const yRange = maxV - minV;

        ctx.lineWidth = 1;

        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
        for (let i = 0; i <= 4; i++) {
            const y = (height / 4) * i;
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
        }
        ctx.stroke();

        ctx.fillStyle = 'rgba(255,255,255,0.45)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.textBaseline = 'middle';
        for (let i = 0; i <= 4; i++) {
            const y = (height / 4) * i;
            const val = maxV - (i / 4) * yRange;
            const txt = Math.abs(val) >= 100 ? val.toFixed(0) : val.toFixed(1);
            const yT = i === 0 ? y + 7 : (i === 4 ? y - 7 : y);
            ctx.fillText(txt, 4, yT);
        }

        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255,255,255,0.3)';
        ctx.moveTo(1, 0); ctx.lineTo(1, height);
        ctx.stroke();

        const zeroY = height - ((0 - minV) / yRange) * height;
        if (zeroY >= 0 && zeroY <= height) {
            ctx.beginPath();
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.moveTo(0, zeroY); ctx.lineTo(width, zeroY);
            ctx.stroke();
            ctx.fillStyle = 'rgba(255,255,255,0.55)';
            ctx.fillText('0', 4, zeroY - 1);
        }

        ctx.fillStyle = 'rgba(255,255,255,0.35)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.textBaseline = 'alphabetic';
        const xLbl = 'time (s) →';
        const w = ctx.measureText(xLbl).width;
        ctx.fillText(xLbl, width - w - 4, height - 4);
    }

    _drawSinePreview(width, height, minV, maxV) {
        if (!this.previewSine) return;
        const { amp, freq } = this.previewSine;
        const range = maxV - minV;
        const ctx = this.ctx;
        const periods = 2;
        ctx.beginPath();
        ctx.strokeStyle = '#ff3131';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        for (let x = 0; x <= width; x++) {
            const phase = (x / width) * periods * 2 * Math.PI;
            const val = Math.min(Math.max(amp * Math.sin(phase), minV), maxV);
            const y = height - ((val - minV) / range) * height;
            if (x === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#ff3131';
        ctx.font = '10px "JetBrains Mono"';
        ctx.fillText(`PREVIEW  ±${amp} RPM @ ${freq} Hz`, 6, 14);
    }

    _hexToRgba(hex, alpha) {
        const c = hex.replace('#', '');
        if (c.length === 6) {
            const r = parseInt(c.slice(0, 2), 16);
            const g = parseInt(c.slice(2, 4), 16);
            const b = parseInt(c.slice(4, 6), 16);
            return `rgba(${r},${g},${b},${alpha})`;
        }
        return hex;
    }
}
