"""
Rig Control: laptop app for the test rig.

Control the props and gimbals, see live graphs, and save runs to CSV.
There's no RPM sensor: RPM, thrust and amps are estimated from throttle
using T-Motor's test table below.
Needs test_rig.ino on the Arduino.

Install once:
    pip install pyserial matplotlib numpy

Run:
    python rig_control.py

Keyboard controls (click on the app window first so it gets your key presses):

    Space or Esc    STOP. Props turn off right away and throttle goes to 0.
                    Use this any time something looks wrong.

    A               Arm the props. Until you arm, the props won't spin no matter
                    what the throttle says. Only works when throttle is at 0,
                    so the props can't jump to speed the moment you arm.

    Up / Down       Hold to raise or lower power smoothly (both props
                    together). Speed per mode is in MODES below. Hold Shift
                    too for slow, fine control.
                    Max is MAX_THROTTLE_PCT below.

    Left / Right    Hold to tilt both gimbals smoothly, TILT_RATE degrees per
                    second. Hold Shift too for slow, fine control.
                    If the gimbals are at different angles, both move by the
                    same amount (the gap between them stays).

    L               Level the gimbals (tilt back to 0).

    R               Start recording to a CSV file in the logs folder.
                    Press R again to stop and save.

The sliders in the window do the same thing as the arrow keys.

Power modes (pick one above the power slider):
    ESC us      The ESC pulse width, like RCBenchmark's "ESC" control.
                1000 us = off, 2000 us = full. Exact, no estimating.
    Throttle %  Same thing as a percent: 1000 us = 0 %, 2000 us = 100 %.
    Amps        Amps per motor. The app works out the throttle from
                MOTOR_CURVE. No current sensor, so this is an estimate.
    Put your own RCBenchmark numbers in MOTOR_CURVE to match your motors.

Gimbal sliders:
    "Both gimbals" moves both to the same angle.
    "Gimbal 1" and "Gimbal 2" move just that one.
    Same angle = both props tilt the same way. The Arduino flips gimbal 2
    for you, since it's mounted on the other side of the car.
To go to an exact value, type it in the box next to the slider and press
Enter (or Set). The Arduino glides there smoothly; it never jumps.

Graphs always run live (last PLOT_SECONDS). Data is only saved to CSV
while recording (R or "Start recording").

Gimbal zero (each gimbal set on its own):
    In the "Gimbal zero" box, type an angle for gimbal 1 or 2 and press Set
    (or Enter). The gimbal moves there, and that spot becomes its new 0.
    -/+ nudge it by ZERO_STEP. The Arduino saves the zeros, so they stay
    after unplugging. The tilt slider and keys still move both gimbals
    together, measured from each one's zero.
    Angles here are degrees from the servo's middle (1500 us).
    Keys like A and Space don't fire while you're typing in these boxes
    (Esc still STOPs).
"""

import csv
import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# Settings 

BAUD = 115200
SEND_EVERY_MS = 20                 # how often to send commands
MAX_THROTTLE_PCT = 40              # max throttle (raise later)
GIMBAL_LIMIT_DEG = 150             # max tilt (servo does 300 deg total = +-150)
PLOT_SECONDS = 30                  # seconds shown on the graphs
PLOT_EVERY_MS = 400                # graph redraw (slower = smoother controls)
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

# Motor: T-Motor V3115 900KV
# Numbers from T-Motor's test (10.5" prop, 6S battery), for one motor.
# Swap in your own numbers once you measure.
MOTOR_TEST_VOLTS = 22.2
MOTOR_CURVE = [
    # thr   RPM    thrust_g  amps
    (0,       0,      0,     0.0),
    (10,   1715,     55,     0.24),
    (20,   3428,    254,     1.27),
    (30,   5048,    593,     3.69),
    (40,   6509,   1009,     7.91),
    (50,   7815,   1475,    13.69),
    (60,   8932,   1942,    21.28),
    (70,   9957,   2427,    30.46),
    (80,  10778,   2853,    41.19),
    (90,  11398,   3216,    52.47),
    (100, 11873,   3496,    64.37),
]

# battery voltage (6S = 22.2, 4S = 14.8)
BATTERY_VOLTS = 22.2

# How fast the arrow keys move things while held (Shift = slow)
THROTTLE_RATE = 15                 # % per second
THROTTLE_RATE_SLOW = 3
TILT_RATE = 40                     # degrees per second
TILT_RATE_SLOW = 8
KEY_TICK_MS = 15
ZERO_STEP = 0.5                    # -/+ buttons in the Gimbal zero box (deg)


_THR, _RPM, _THRUST, _AMPS = (np.array(col, dtype=float) for col in zip(*MOTOR_CURVE))
_V_SCALE = BATTERY_VOLTS / MOTOR_TEST_VOLTS


def estimate_rpm(throttle_pct):
    return float(np.interp(throttle_pct, _THR, _RPM)) * _V_SCALE


def estimate_thrust_g(throttle_pct):
    return float(np.interp(throttle_pct, _THR, _THRUST)) * _V_SCALE ** 2


def estimate_amps(throttle_pct):
    return float(np.interp(throttle_pct, _THR, _AMPS)) * _V_SCALE ** 2


def throttle_for_amps(amps):
    """Throttle % that should draw this many amps per motor (from MOTOR_CURVE)."""
    return float(np.interp(amps / _V_SCALE ** 2, _AMPS, _THR))


MAX_AMPS = round(estimate_amps(MAX_THROTTLE_PCT), 1)   # amps at the throttle cap

# ESC pulse: 1000 us = 0 %, 2000 us = 100 % (same as test_rig.ino)
ESC_MIN_US, ESC_MAX_US = 1000, 2000


def pct_to_us(pct):
    return ESC_MIN_US + pct * (ESC_MAX_US - ESC_MIN_US) / 100.0


def us_to_pct(us):
    return (us - ESC_MIN_US) * 100.0 / (ESC_MAX_US - ESC_MIN_US)


# Power slider modes:
#   name: (title, unit, min, max, key speed per s, slow key speed, to throttle %, from throttle %)
MODES = {
    "us":       ("ESC pulse", "us", ESC_MIN_US, round(pct_to_us(MAX_THROTTLE_PCT)), 150, 30,
                 us_to_pct, pct_to_us),
    "throttle": ("Throttle", "%", 0, MAX_THROTTLE_PCT, 15, 3,
                 lambda v: v, lambda t: t),
    "amps":     ("Amps per motor", "A", 0, MAX_AMPS, 2, 0.4,
                 throttle_for_amps, estimate_amps),
}


# USB connection


class Link:
    """Talks to the Arduino."""

    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.lines = queue.Queue()
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError, TypeError):
                self.lines.put(None)       # connection lost
                return
            if raw:
                self.lines.put(raw.decode("ascii", errors="replace").strip())

    def send(self, text):
        try:
            self.ser.write((text + "\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError):
            return False

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


# App

FIELDS = ["time_s", "gimbal1_deg", "gimbal2_deg", "throttle1_pct", "throttle2_pct",
          "esc1_us", "esc2_us", "armed",
          "rpm1_est", "rpm2_est",
          "thrust1_g_est", "thrust2_g_est", "amps1_est", "amps2_est"]


class App:
    def __init__(self, root):
        self.root = root
        root.title("Rig Control")
        self.link = None
        self.armed = False
        self.t0_ms = None
        self.history = deque()             # graph data (last PLOT_SECONDS)
        self.latest = None                 # newest data, for the status panel
        self.rec_file = None
        self.rec_writer = None
        self.rec_rows = []                 # recorded data
        self.rec_start = None
        self.cmd_tilt = 0.0
        self.cmd_throttle = 0.0

        self._build_ui()
        self.refresh_ports()
        self._bind_keys()
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.root.after(SEND_EVERY_MS, self.tick_control)
        self.root.after(20, self.tick_read)
        self.plot_dirty = True
        self.layout_dirty = True
        self.root.after(PLOT_EVERY_MS, self.tick_plot)

    # Window

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=4)

        self.stop_btn = tk.Button(top, text="STOP  (Space)", bg="#c62828", fg="white",
                                  font=("Segoe UI", 11, "bold"), command=self.stop, width=14)
        self.stop_btn.pack(side="right", padx=4)
        self.arm_btn = tk.Button(top, text="ARM", font=("Segoe UI", 11, "bold"), command=self.arm, width=8)
        self.arm_btn.pack(side="right", padx=4)
        self.rec_btn = ttk.Button(top, text="Start recording", command=self.toggle_record)
        self.rec_btn.pack(side="right", padx=4)

        body = ttk.Frame(self.root, padding=6)
        body.pack(fill="both", expand=True)

        controls = ttk.Frame(body, width=260)
        controls.pack(side="left", fill="y", padx=(0, 8))

        # Power slider: throttle % or amps per motor (throttle_var holds the slider value)
        mode_row = ttk.Frame(controls)
        mode_row.pack(anchor="w")
        self.mode_var = tk.StringVar(value="us")
        self.prev_mode = "us"
        for value, text in (("us", "ESC us"), ("throttle", "Throttle %"), ("amps", "Amps")):
            ttk.Radiobutton(mode_row, text=text, value=value, variable=self.mode_var,
                            command=self.change_mode).pack(side="left", padx=(0, 8))
        self.power_title = ttk.Label(controls)
        self.power_title.pack(anchor="w")
        self.throttle_var = tk.DoubleVar(value=self.power_min())
        self.power_scale = ttk.Scale(controls, from_=self.power_min(), to=self.power_max(),
                                     variable=self.throttle_var, orient="horizontal", length=240)
        self.power_scale.pack(anchor="w")
        self.throttle_entry_var = self._value_box(controls, self.set_throttle_from_box)
        self.throttle_lbl = ttk.Label(controls, text="0 %", wraplength=240)
        self.throttle_lbl.pack(anchor="w", pady=(0, 10))
        self._update_power_title()

        # "Both" slider sets both gimbals. Gimbal 1 / 2 sliders set just one.
        ttk.Label(controls, text="Tilt: both gimbals (deg)").pack(anchor="w")
        self.tilt_var = tk.DoubleVar(value=0)
        ttk.Scale(controls, from_=-GIMBAL_LIMIT_DEG, to=GIMBAL_LIMIT_DEG, variable=self.tilt_var,
                  orient="horizontal", length=240,
                  command=lambda v: self.set_both(float(v))).pack(anchor="w")
        self.tilt_entry_var = self._value_box(controls, self.set_tilt_from_box)

        self.g_vars = [tk.DoubleVar(value=0), tk.DoubleVar(value=0)]
        self.g_entry_vars = []
        for i in range(2):
            ttk.Label(controls, text=f"Tilt: gimbal {i + 1} only (deg)").pack(anchor="w", pady=(6, 0))
            ttk.Scale(controls, from_=-GIMBAL_LIMIT_DEG, to=GIMBAL_LIMIT_DEG, variable=self.g_vars[i],
                      orient="horizontal", length=240).pack(anchor="w")
            self.g_entry_vars.append(self._value_box(controls, lambda n=i: self.set_one_from_box(n)))

        self.tilt_lbl = ttk.Label(controls, text="0 deg")
        self.tilt_lbl.pack(anchor="w", pady=(4, 0))
        ttk.Button(controls, text="Level gimbals", command=lambda: self.set_both(0)).pack(anchor="w", pady=(2, 10))

        zbox = ttk.LabelFrame(controls, text="Gimbal zero (deg from servo middle)", padding=4)
        zbox.pack(anchor="w", fill="x", pady=(0, 10))
        self.zero_vars = []
        for i in range(2):
            row = ttk.Frame(zbox)
            row.pack(anchor="w", pady=1)
            ttk.Label(row, text=f"Gimbal {i + 1}", width=8).pack(side="left")
            var = tk.StringVar(value="?")
            self.zero_vars.append(var)
            ent = ttk.Entry(row, textvariable=var, width=7)
            ent.pack(side="left", padx=2)
            ent.bind("<Return>", lambda e, n=i: self.set_zero(n))
            ttk.Button(row, text="-", width=2, command=lambda n=i: self.set_zero(n, -ZERO_STEP)).pack(side="left")
            ttk.Button(row, text="+", width=2, command=lambda n=i: self.set_zero(n, +ZERO_STEP)).pack(side="left")
            ttk.Button(row, text="Set", width=4, command=lambda n=i: self.set_zero(n)).pack(side="left", padx=2)

        ttk.Label(controls, font=("Consolas", 9), justify="left", text=(
            "Keys (click this window first)\n"
            "  Space/Esc   STOP\n"
            "  A           arm props\n"
            "  Up/Down     hold: throttle up/down\n"
            "  Left/Right  hold: tilt\n"
            "  + Shift     slow, fine control\n"
            "  L           level gimbals\n"
            "  R           record on/off")).pack(anchor="w")

        ttk.Separator(controls).pack(fill="x", pady=8)
        self.status_lbl = ttk.Label(controls, text="", justify="left", font=("Consolas", 10))
        self.status_lbl.pack(anchor="w")

        ttk.Separator(controls).pack(fill="x", pady=8)
        ttk.Label(controls, text="Messages").pack(anchor="w")
        self.log_box = tk.Text(controls, width=34, height=7, font=("Consolas", 9), state="disabled")
        self.log_box.pack(anchor="w", fill="y", expand=True)

        self.fig = Figure(figsize=(7.5, 6), dpi=90)
        self.ax_angle = self.fig.add_subplot(3, 1, 1)
        self.ax_rpm = self.fig.add_subplot(3, 1, 2, sharex=self.ax_angle)
        self.ax_xy = self.fig.add_subplot(3, 1, 3)

        self.ax_angle.set_ylabel("gimbal angle (deg)")
        self.ax_angle.set_ylim(-GIMBAL_LIMIT_DEG - 5, GIMBAL_LIMIT_DEG + 5)
        self.l_g1, = self.ax_angle.plot([], [], label="gimbal 1")
        self.l_g2, = self.ax_angle.plot([], [], label="gimbal 2", linestyle="--")
        self.ax_angle.legend(loc="upper left", fontsize=8)

        self.ax_rpm.set_ylabel("prop RPM (estimated)")
        self.ax_rpm.set_xlabel("time (s)")
        self.l_r1e, = self.ax_rpm.plot([], [], label="prop 1")
        self.l_r2e, = self.ax_rpm.plot([], [], label="prop 2", linestyle="--")
        self.ax_rpm.legend(loc="upper left", fontsize=8)

        self.ax_xy.set_xlabel("prop RPM (estimated)")
        self.ax_xy.set_ylabel("gimbal angle (deg)")
        self.sc1 = self.ax_xy.scatter([], [], s=8, label="gimbal 1 vs prop 1")
        self.sc2 = self.ax_xy.scatter([], [], s=8, marker="x", label="gimbal 2 vs prop 2")
        self.ax_xy.legend(loc="upper left", fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=body)
        self.canvas.get_tk_widget().pack(side="left", fill="both", expand=True)
        self.canvas.get_tk_widget().bind("<Configure>", lambda e: self._mark_layout(), add="+")

    def _value_box(self, parent, on_set):
        """Small 'type a value' box + Set button under a slider."""
        row = ttk.Frame(parent)
        row.pack(anchor="w", pady=(2, 0))
        ttk.Label(row, text="Go to").pack(side="left")
        var = tk.StringVar()
        ent = ttk.Entry(row, textvariable=var, width=7)
        ent.pack(side="left", padx=4)
        ent.bind("<Return>", lambda e: on_set())
        ttk.Button(row, text="Set", width=4, command=on_set).pack(side="left")
        return var

    def _read_box(self, var, lo, hi, name):
        try:
            v = float(var.get())
        except ValueError:
            messagebox.showwarning("Rig Control", f"{name} must be a number.")
            return None
        v = max(lo, min(hi, v))
        var.set(f"{v:g}")
        self.root.focus_set()              # so the keys work again
        return v

    def set_throttle_from_box(self):
        v = self._read_box(self.throttle_entry_var, self.power_min(), self.power_max(),
                           self.mode()[0])
        if v is not None:
            self.throttle_var.set(v)

    # Power modes: ESC us / Throttle % / Amps

    def mode(self, name=None):
        return MODES[name or self.mode_var.get()]

    def power_min(self):
        return self.mode()[2]

    def power_max(self):
        return self.mode()[3]

    def throttle_now(self):
        """Throttle % to send, from the slider (converted from the mode's units)."""
        t = self.mode()[6](self.throttle_var.get())
        return min(MAX_THROTTLE_PCT, max(0.0, t))

    def change_mode(self):
        # Keep the props at the same speed when switching
        pct = self.mode(self.prev_mode)[6](self.throttle_var.get())
        self.prev_mode = self.mode_var.get()
        v = self.mode()[7](pct)
        self.power_scale.configure(from_=self.power_min(), to=self.power_max())
        self.throttle_var.set(round(min(self.power_max(), max(self.power_min(), v)), 2))
        self.throttle_entry_var.set("")
        self._update_power_title()
        self.root.focus_set()

    def _update_power_title(self):
        title, unit, lo, hi = self.mode()[:4]
        self.power_title.configure(text=f"{title}, both props ({lo}-{hi} {unit})")

    def set_tilt_from_box(self):
        v = self._read_box(self.tilt_entry_var, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG, "Tilt")
        if v is not None:
            self.set_both(v)

    def set_one_from_box(self, i):
        v = self._read_box(self.g_entry_vars[i], -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG, f"Gimbal {i + 1} tilt")
        if v is not None:
            self.g_vars[i].set(v)

    def set_both(self, v):
        """Put both gimbals at the same angle."""
        v = min(GIMBAL_LIMIT_DEG, max(-GIMBAL_LIMIT_DEG, v))
        self.tilt_var.set(v)
        for g in self.g_vars:
            g.set(v)

    # Keyboard

    def _bind_keys(self):
        keys = {
            "<space>": self.stop,
            "<Escape>": self.stop,
            "<KeyPress-a>": self.arm,
            "<KeyPress-A>": self.arm,
            "<KeyPress-r>": self.toggle_record,
            "<KeyPress-R>": self.toggle_record,
            "<KeyPress-l>": lambda: self.set_both(0),
            "<KeyPress-L>": lambda: self.set_both(0),
        }
        for key, action in keys.items():
            if key == "<Escape>":
                self.root.bind(key, lambda e, act=action: act())
            else:
                self.root.bind(key, lambda e, act=action: None if self.typing(e) else act())

        # Arrow keys: move smoothly while held (see tick_keys)
        self.held = set()
        self.shift = False
        for key in ("Up", "Down", "Left", "Right"):
            for mod in ("", "Shift-"):
                for widget in (self.root, self.port_box):   # port_box: don't let the dropdown grab arrows
                    widget.bind(f"<{mod}KeyPress-{key}>", lambda e, k=key: self._arrow(e, k, True))
                    widget.bind(f"<{mod}KeyRelease-{key}>", lambda e, k=key: self._arrow(e, k, False))
        for key in ("Shift_L", "Shift_R"):
            self.root.bind(f"<KeyPress-{key}>", lambda e: setattr(self, "shift", True))
            self.root.bind(f"<KeyRelease-{key}>", lambda e: setattr(self, "shift", False))
        self.root.bind("<FocusOut>", lambda e: self.held.clear())
        self.port_box.bind("<<ComboboxSelected>>", lambda e: self.root.focus_set())
        self.last_key_tick = time.monotonic()
        self.root.after(KEY_TICK_MS, self.tick_keys)

    def _arrow(self, event, key, down):
        if self.typing(event):
            return None
        self.shift = bool(event.state & 0x0001)
        if down:
            self.held.add(key)
        else:
            self.held.discard(key)
        return "break"

    @staticmethod
    def typing(event):
        return isinstance(event.widget, (tk.Entry, ttk.Entry))

    def tick_keys(self):
        """While an arrow key is held, move throttle/tilt a little every tick."""
        now = time.monotonic()
        dt = min(0.3, now - self.last_key_tick)
        self.last_key_tick = now
        thr = ("Up" in self.held) - ("Down" in self.held)
        tilt = ("Right" in self.held) - ("Left" in self.held)
        if thr:
            rate = self.mode()[5] if self.shift else self.mode()[4]
            v = self.throttle_var.get() + thr * rate * dt
            self.throttle_var.set(min(self.power_max(), max(self.power_min(), v)))
        if tilt:
            rate = TILT_RATE_SLOW if self.shift else TILT_RATE
            step = tilt * rate * dt          # move both by the same amount
            for g in self.g_vars:
                g.set(min(GIMBAL_LIMIT_DEG, max(-GIMBAL_LIMIT_DEG, g.get() + step)))
            self.tilt_var.set(sum(g.get() for g in self.g_vars) / 2)
        self.root.after(KEY_TICK_MS, self.tick_keys)

    def log(self, text):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    # Connection

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        names = [f"{p.device} - {p.description}" for p in ports]
        self.port_box["values"] = names
        if names and not self.port_var.get():
            # Pick the Arduino if we can spot it
            pick = next((n for n in names if any(k in n for k in ("Arduino", "CH340", "USB"))), names[0])
            self.port_var.set(pick)

    def toggle_connect(self):
        if self.link:
            self.disconnect()
            return
        if not self.port_var.get():
            messagebox.showwarning("Rig Control", "Pick a port first.")
            return
        port = self.port_var.get().split(" - ")[0]
        try:
            self.link = Link(port)
        except serial.SerialException as e:
            messagebox.showerror("Rig Control", f"Couldn't open {port}.\n\n{e}\n\n"
                                 "Is the Serial Monitor (or another app) still using it?")
            return
        self.t0_ms = None
        self.history.clear()
        self.connect_btn.configure(text="Disconnect")
        self.log(f"connected to {port} (Arduino restarts, ~2 s)")
        self.root.focus_set()              # so the keys work right away

    def disconnect(self):
        if self.link:
            self.link.send("x")
            self.link.close()
            self.link = None
        self.armed = False
        self.connect_btn.configure(text="Connect")
        self.log("disconnected")

    # Commands

    def arm(self):
        if not self.link:
            self.log("connect first")
            return
        if self.cmd_throttle > 0:
            messagebox.showwarning("Rig Control", "Set the throttle to 0 before arming.")
            return
        self.link.send("a")

    def stop(self):
        self.throttle_var.set(self.power_min())
        self.cmd_throttle = 0
        if self.link:
            self.link.send("x")
        self.log("STOP")

    def set_zero(self, i, nudge=0.0):
        """Send gimbal i's zero to the Arduino (it saves it)."""
        if not self.link:
            self.log("connect first")
            return
        try:
            deg = float(self.zero_vars[i].get()) + nudge
        except ValueError:
            messagebox.showwarning("Rig Control", f"Gimbal {i + 1} zero must be a number.")
            return
        deg = round(max(-GIMBAL_LIMIT_DEG, min(GIMBAL_LIMIT_DEG, deg)), 1)
        self.zero_vars[i].set(f"{deg:g}")
        self.link.send(f"z {i + 1} {deg}")
        self.root.focus_set()              # so the keys work again

    def toggle_record(self):
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = None
            self.rec_btn.configure(text="Start recording")
            self.log(f"saved {len(self.rec_rows)} rows to\n  {self.rec_path}")
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        self.rec_path = os.path.join(LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.csv"))
        self.rec_file = open(self.rec_path, "w", newline="")
        self.rec_writer = csv.DictWriter(self.rec_file, fieldnames=FIELDS)
        self.rec_writer.writeheader()
        self.rec_rows = []
        self.rec_start = None
        self.rec_btn.configure(text="Stop recording")
        self.log("recording...")

    # Repeating jobs

    def tick_control(self):
        """Read the sliders (keys move them too), send to the Arduino."""
        self.cmd_throttle = round(self.throttle_now(), 1)
        g1, g2 = (round(g.get(), 1) for g in self.g_vars)
        self.cmd_tilt = (g1 + g2) / 2
        t = self.cmd_throttle
        self.throttle_lbl.configure(text=f"{pct_to_us(t):.0f} us = {t:.1f} %\n"
                                         f"est. per motor: ~{estimate_amps(t):.1f} A, "
                                         f"~{estimate_rpm(t):.0f} RPM, {estimate_thrust_g(t):.0f} g")
        self.tilt_lbl.configure(text=f"gimbal 1: {g1:+.1f} deg    gimbal 2: {g2:+.1f} deg")

        if self.link:
            t = self.cmd_throttle
            self.link.send(f"s {g1} {g2} {t} {t}")
        self.root.after(SEND_EVERY_MS, self.tick_control)

    def tick_read(self):
        """Read what the Arduino sent."""
        while self.link:
            try:
                line = self.link.lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                self.log("lost connection to the Arduino")
                self.disconnect()
                break
            if line.startswith("T,"):
                self.on_telemetry(line)
            elif line:
                if line.startswith("# zero:"):
                    self.on_zero(line)
                self.log(line.lstrip("# "))
        self.update_status()
        self.root.after(20, self.tick_read)

    def on_telemetry(self, line):
        parts = line.split(",")
        if len(parts) != 7:
            return
        try:
            ms = int(parts[1])
            g1, g2, t1, t2 = (float(x) for x in parts[2:6])
            armed = parts[6] == "1"
        except ValueError:
            return
        if self.t0_ms is None or ms < self.t0_ms:
            self.t0_ms = ms
        self.armed = armed
        row = {"time_s": (ms - self.t0_ms) / 1000.0, "gimbal1_deg": g1, "gimbal2_deg": g2,
               "throttle1_pct": t1, "throttle2_pct": t2,
               "esc1_us": round(pct_to_us(t1)), "esc2_us": round(pct_to_us(t2)), "armed": int(armed),
               "rpm1_est": round(estimate_rpm(t1)), "rpm2_est": round(estimate_rpm(t2)),
               "thrust1_g_est": round(estimate_thrust_g(t1)), "thrust2_g_est": round(estimate_thrust_g(t2)),
               "amps1_est": round(estimate_amps(t1), 2), "amps2_est": round(estimate_amps(t2), 2)}
        self.latest = row
        self.history.append(row)
        self.plot_dirty = True
        while self.history and row["time_s"] - self.history[0]["time_s"] > PLOT_SECONDS:
            self.history.popleft()

        # CSV only while recording
        if self.rec_file:
            if self.rec_start is None:
                self.rec_start = row["time_s"]
            rec = dict(row, time_s=round(row["time_s"] - self.rec_start, 3))
            self.rec_writer.writerow(rec)
            self.rec_rows.append(rec)

    def on_zero(self, line):
        parts = line.split(":", 1)[1].split()
        if len(parts) == 2:
            for var, val in zip(self.zero_vars, parts):
                var.set(f"{float(val):g}")

    def update_status(self):
        lines = [
            f"link:       {'connected' if self.link else 'not connected'}",
            f"props:      {'ARMED' if self.armed else 'disarmed'}",
            f"recording:  {'YES' if self.rec_file else 'no'}",
        ]
        if self.latest:
            r = self.latest
            lines += ["",
                      f"gimbal 1/2: {r['gimbal1_deg']:+.1f} / {r['gimbal2_deg']:+.1f} deg",
                      f"throttle:   {r['throttle1_pct']:.1f} / {r['throttle2_pct']:.1f} %",
                      f"RPM est:    {r['rpm1_est']} / {r['rpm2_est']}",
                      f"thrust est: {r['thrust1_g_est']} / {r['thrust2_g_est']} g",
                      f"amps est:   {r['amps1_est'] + r['amps2_est']:.1f} A total"]
        self.status_lbl.configure(text="\n".join(lines))
        self.arm_btn.configure(bg="#2e7d32" if self.armed else "#e0e0e0",
                               fg="white" if self.armed else "black",
                               text="ARMED" if self.armed else "ARM")

    def rpm_of(self, rows, i):
        return np.array([r[f"rpm{i}_est"] for r in rows], dtype=float)

    def _mark_layout(self):
        self.layout_dirty = True
        self.plot_dirty = True

    def tick_plot(self):
        # Redrawing is slow, so only do it when something changed
        if not self.plot_dirty:
            self.root.after(PLOT_EVERY_MS, self.tick_plot)
            return
        self.plot_dirty = False
        rows = list(self.history)
        if rows:
            t = np.array([r["time_s"] for r in rows])
            self.l_g1.set_data(t, [r["gimbal1_deg"] for r in rows])
            self.l_g2.set_data(t, [r["gimbal2_deg"] for r in rows])
            self.l_r1e.set_data(t, [r["rpm1_est"] for r in rows])
            self.l_r2e.set_data(t, [r["rpm2_est"] for r in rows])
            self.ax_angle.set_xlim(max(0, t[-1] - PLOT_SECONDS), max(PLOT_SECONDS, t[-1]))
            self.ax_rpm.relim()
            self.ax_rpm.autoscale_view(scalex=False)

        # Angle vs RPM: the recording if there is one, otherwise live
        xy_rows = self.rec_rows or rows
        if xy_rows:
            g1 = [r["gimbal1_deg"] for r in xy_rows]
            g2 = [r["gimbal2_deg"] for r in xy_rows]
            r1, r2 = self.rpm_of(xy_rows, 1), self.rpm_of(xy_rows, 2)
            self.sc1.set_offsets(np.column_stack([r1, g1]))
            self.sc2.set_offsets(np.column_stack([r2, g2]))
            self.ax_xy.set_xlim(0, max(1000, float(max(r1.max(), r2.max())) * 1.1))
            self.ax_xy.set_ylim(-GIMBAL_LIMIT_DEG - 5, GIMBAL_LIMIT_DEG + 5)
            src = "recording" if self.rec_rows else f"last {PLOT_SECONDS} s"
            self.ax_xy.set_title(f"gimbal angle vs RPM ({src})", fontsize=9)

        if self.layout_dirty:              # fit the graphs to the window (slow, so only on resize)
            self.layout_dirty = False
            self.fig.set_layout_engine("constrained")
            self.canvas.draw()
            self.fig.set_layout_engine("none")
        else:
            self.canvas.draw()
        self.root.after(PLOT_EVERY_MS, self.tick_plot)

    def on_close(self):
        if self.rec_file:
            self.toggle_record()
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
