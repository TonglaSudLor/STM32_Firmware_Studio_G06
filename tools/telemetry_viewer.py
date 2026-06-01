import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM15'  # <--- Ensure this matches your COM port
BAUD_RATE = 115200

# Data Storage
class TelemetryData:
    def __init__(self):
        self.current_run = {"t": [], "pos": [], "target": []}
        self.prev_1 = {"t": [], "pos": []}
        self.prev_2 = {"t": [], "pos": []}
        self.history_settle = []
        self.history_overshoot = []
        self.run_count = 0
        self.is_collecting = False
        self.start_pos = 0.0
        self.target_val = 0.0
        self.dir_mult = 1.0

data = TelemetryData()

def read_serial():
    """Background thread to handle the START/DATA/END protocol"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.flushInput()
        print(f"Connected to {SERIAL_PORT}. Ready for Ghost Move...")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if "START" in line:
                    # START,target
                    parts = line.split(',')
                    if len(parts) >= 2:
                        data.target_val = float(parts[1])
                    
                    # Shift History
                    if len(data.current_run["pos"]) > 0:
                        data.prev_2 = data.prev_1.copy()
                        data.prev_1 = data.current_run.copy()
                    
                    data.current_run = {"t": [], "pos": [], "target": []}
                    data.is_collecting = True
                    data.start_pos = None # Will set on first DATA
                    print(f">>> Recording Run #{data.run_count + 1}...")

                elif "END" in line:
                    if data.is_collecting and len(data.current_run["t"]) > 0:
                        data.is_collecting = False
                        data.run_count += 1
                        
                        # Calculate Metrics
                        pos = np.array(data.current_run["pos"])
                        t = np.array(data.current_run["t"])
                        target = data.current_run["target"][-1]
                        
                        peak = np.max(pos)
                        overshoot = max(0.0, ((peak - target) / target) * 100.0) if target > 1.0 else 0.0
                        
                        # Settling Time (Last index outside 1.0 deg band)
                        idx_outside = np.where(np.abs(pos - target) > 1.0)[0]
                        settle_time = t[idx_outside[-1]] if len(idx_outside) > 0 else 0.0
                        
                        data.history_settle.append(settle_time)
                        data.history_overshoot.append(overshoot)
                        print(f"DONE. Run #{data.run_count}: Settling={settle_time:.2f}s, OS={overshoot:.1f}%")

                elif "DATA" in line:
                    # DATA,tick,pos,target
                    parts = line.split(',')
                    if len(parts) >= 4:
                        try:
                            tick = float(parts[1]) / 1000.0 # to seconds
                            p_raw = float(parts[2]) / 100.0
                            t_raw = float(parts[3]) / 100.0
                            
                            if data.start_pos is None:
                                data.start_pos = p_raw
                                data.dir_mult = 1.0 if (t_raw - p_raw) >= 0 else -1.0
                            
                            # Normalize: Start at 0, Flip if moving negative
                            norm_pos = (p_raw - data.start_pos) * data.dir_mult
                            norm_target = (t_raw - data.start_pos) * data.dir_mult
                            
                            data.current_run["t"].append(tick)
                            data.current_run["pos"].append(norm_pos)
                            data.current_run["target"].append(norm_target)
                        except ValueError: pass

    except Exception as e:
        print(f"Serial Error: {e}")

# Start background thread
threading.Thread(target=read_serial, daemon=True).start()

# --- Plotting Dashboard ---
plt.style.use('dark_background')
fig = plt.figure(figsize=(14, 9))
gs = fig.add_gridspec(2, 2)

ax_main = fig.add_subplot(gs[0, :])
ax_settle = fig.add_subplot(gs[1, 0])
ax_over = fig.add_subplot(gs[1, 1])

# Main Plot Objects
p_target, = ax_main.plot([], [], 'm--', alpha=0.6, label='Target', linewidth=1.5)
p_prev2, = ax_main.plot([], [], color='yellow', alpha=0.1, label='Prev Run -2', linewidth=1)
p_prev1, = ax_main.plot([], [], color='yellow', alpha=0.3, label='Prev Run -1', linewidth=1.5)
p_current, = ax_main.plot([], [], 'y', label='Current Run', linewidth=2.5)

ax_main.set_xlim(0, 10)
ax_main.set_ylim(0, 200)
ax_main.set_title('Ghost Mode PID Performance (Normalized)', fontweight='bold')
ax_main.set_ylabel('Relative Position (deg)')
ax_main.legend(loc='upper right')
ax_main.grid(True, alpha=0.1)

# History Plots
p_settle_hist, = ax_settle.plot([], [], 'c-o', linewidth=1.5, markersize=4)
ax_settle.set_title('Settling Time History')
ax_settle.set_ylabel('Time (s)')
ax_settle.set_xlabel('Run #')
ax_settle.grid(True, alpha=0.1)

p_over_hist, = ax_over.plot([], [], 'g-o', linewidth=1.5, markersize=4)
ax_over.set_title('Overshoot History')
ax_over.set_ylabel('Overshoot (%)')
ax_over.set_xlabel('Run #')
ax_over.grid(True, alpha=0.1)

def update(frame):
    # Update Main Plot
    if len(data.current_run["t"]) > 0:
        t_norm = np.array(data.current_run["t"]) - data.current_run["t"][0]
        p_current.set_data(t_norm, data.current_run["pos"])
        p_target.set_data([0, 10], [data.current_run["target"][-1], data.current_run["target"][-1]])
        ax_main.set_xlim(0, max(10, t_norm[-1]))
        ax_main.set_ylim(0, max(100, data.current_run["target"][-1] * 1.5))
    
    if len(data.prev_1["t"]) > 0:
        t_prev1 = np.array(data.prev_1["t"]) - data.prev_1["t"][0]
        p_prev1.set_data(t_prev1, data.prev_1["pos"])
    
    if len(data.prev_2["t"]) > 0:
        t_prev2 = np.array(data.prev_2["t"]) - data.prev_2["t"][0]
        p_prev2.set_data(t_prev2, data.prev_2["pos"])

    # Update History Plots
    if len(data.history_settle) > 0:
        runs = range(1, len(data.history_settle) + 1)
        p_settle_hist.set_data(runs, data.history_settle)
        p_over_hist.set_data(runs, data.history_overshoot)
        ax_settle.set_xlim(0.5, len(runs) + 0.5)
        ax_over.set_xlim(0.5, len(runs) + 0.5)
        ax_settle.set_ylim(0, max(5, max(data.history_settle) * 1.2))
        ax_over.set_ylim(0, max(10, max(data.history_overshoot) * 1.2))

    return p_current, p_target, p_prev1, p_prev2, p_settle_hist, p_over_hist

ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
