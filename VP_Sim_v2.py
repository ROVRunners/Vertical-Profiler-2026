import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons, TextBox

# -----------------------------
# VP configuration database
# -----------------------------
VP_CONFIGS = {
    "LBCC 2026": {
        "vehicle": {
            "mass": 1.2,
            "rho": 1023.0,
            "neutral_volume_m3": 0.001173,
            "reference_area_m2": 0.0070,
            "drag_coefficient": 0.9,
            "buoyancy_engine_delta_m3": 19.6e-6,
        },
        "actuator": {
            "min_us": 500,
            "max_us": 2500,
            "neutral_us": 1500,
            "idle_us": 500,
            "direction": -1,
            "full_travel_time_s": 1.2,
        },
        "sensor": {
            "surface_pressure_mbar": 1013.25,
            "depth_noise_std_m": 0.002,
        },
        "pid": {
            "kp": 700.0,
            "ki": 0.0,
            "kd": 0.0,
            "interval_s": 0.10,
            "integral_min": -1.5,
            "integral_max": 1.5,
        },
        "mission": {
            "deep_m": 0.60,
            "shallow_m": 0.40,
            "surface_m": 0.02,
            "hold_s": 30.0,
            "tol_m": 0.05,
            "surface_tol_m": 0.05,
            "max_transit_s": 90.0,
            "max_hold_s": 60.0,
            "max_recover_s": 90.0,
            "self_recover": True,
        },
    },

    "Rays 2026": {
        "vehicle": {
            "mass": 1.2,
            "rho": 1023.0,
            "neutral_volume_m3": 0.001173,
            "reference_area_m2": 0.0070,
            "drag_coefficient": 0.9,
            "buoyancy_engine_delta_m3": 19.6e-6,
        },
        "actuator": {
            "min_us": 500,
            "max_us": 2500,
            "neutral_us": 1500,
            "idle_us": 500,
            "direction": -1,
            "full_travel_time_s": 1.2,
        },
        "sensor": {
            "surface_pressure_mbar": 1013.25,
            "depth_noise_std_m": 0.002,
        },
        "pid": {
            "kp": 700.0,
            "ki": 0.0,
            "kd": 0.0,
            "interval_s": 0.10,
            "integral_min": -1.5,
            "integral_max": 1.5,
        },
        "mission": {
            "deep_m": 0.60,
            "shallow_m": 0.40,
            "surface_m": 0.02,
            "hold_s": 30.0,
            "tol_m": 0.05,
            "surface_tol_m": 0.05,
            "max_transit_s": 90.0,
            "max_hold_s": 60.0,
            "max_recover_s": 90.0,
            "self_recover": True,
        },
    },

    "Rays 2024": {
        "vehicle": {
            "mass": 1.8,
            "rho": 997.0,
            "neutral_volume_m3": 0.001805,
            "reference_area_m2": 0.0080,
            "drag_coefficient": 0.8,
            "buoyancy_engine_delta_m3": 26.0e-6,
        },
        "actuator": {
            "min_us": 1000,
            "max_us": 1300,
            "neutral_us": 1150,
            "idle_us": 1000,
            "direction": -1,
            "full_travel_time_s": 1.5,
        },
        "sensor": {
            "surface_pressure_mbar": 1013.25,
            "depth_noise_std_m": 0.002,
        },
        "pid": {
            "kp": 90.0,
            "ki": 0.0,
            "kd": 0.0,
            "interval_s": 0.10,
            "integral_min": -1.5,
            "integral_max": 1.5,
        },
        "mission": {
            "deep_m": 2.5,
            "shallow_m": 0.0,
            "surface_m": 0.05,
            "hold_s": 30.0,
            "tol_m": 0.25,
            "surface_tol_m": 0.10,
            "max_transit_s": 120.0,
            "max_hold_s": 60.0,
            "max_recover_s": 120.0,
            "self_recover": False,
        },
    },
}

SELECTED_VP = "LBCC 2026"

# -----------------------------
# Simple single-target step
# For STEP 1, keep mission logic simple
# just drive to deep target in real time
# -----------------------------
class VPSim:
    IDLE = 0
    DESCEND_1 = 1
    HOLD_DEEP_1 = 2
    ASCEND_1 = 3
    HOLD_SHALLOW_1 = 4
    DESCEND_2 = 5
    HOLD_DEEP_2 = 6
    ASCEND_2 = 7
    HOLD_SHALLOW_2 = 8
    STATION_KEEP = 9
    RECOVER_SURFACE = 10
    DONE = 11

    STATE_NAMES = {
        IDLE: "IDLE",
        DESCEND_1: "DESCEND_1",
        HOLD_DEEP_1: "HOLD_DEEP_1",
        ASCEND_1: "ASCEND_1",
        HOLD_SHALLOW_1: "HOLD_SHALLOW_1",
        DESCEND_2: "DESCEND_2",
        HOLD_DEEP_2: "HOLD_DEEP_2",
        ASCEND_2: "ASCEND_2",
        HOLD_SHALLOW_2: "HOLD_SHALLOW_2",
        STATION_KEEP: "STATION_KEEP",
        RECOVER_SURFACE: "RECOVER_SURFACE",
        DONE: "DONE",
    }

    def __init__(self, cfg_name: str):
        self.cfg_name = cfg_name
        self.cfg = VP_CONFIGS[cfg_name]
        self.rng = np.random.default_rng(42)
        self.reset()

    def load_config(self, cfg_name: str):
        self.cfg_name = cfg_name
        self.cfg = VP_CONFIGS[cfg_name]
        self.reset()

    def apply_settings(
        self,
        deep_m,
        shallow_m,
        surface_m,
        hold_s,
        tol_m,
        surface_tol_m,
        kp,
        ki,
        kd,
    ):
        self.cfg["mission"]["deep_m"] = deep_m
        self.cfg["mission"]["shallow_m"] = shallow_m
        self.cfg["mission"]["surface_m"] = surface_m
        self.cfg["mission"]["hold_s"] = hold_s
        self.cfg["mission"]["tol_m"] = tol_m
        self.cfg["mission"]["surface_tol_m"] = surface_tol_m

        self.cfg["pid"]["kp"] = kp
        self.cfg["pid"]["ki"] = ki
        self.cfg["pid"]["kd"] = kd

        self.reset()

    def reset(self):
        cfg = self.cfg

        self.mass = cfg["vehicle"]["mass"]
        self.rho = cfg["vehicle"]["rho"]
        self.g = 9.81
        self.neutral_volume_m3 = cfg["vehicle"]["neutral_volume_m3"]
        self.reference_area_m2 = cfg["vehicle"]["reference_area_m2"]
        self.drag_coefficient = cfg["vehicle"]["drag_coefficient"]
        self.buoyancy_engine_delta_m3 = cfg["vehicle"]["buoyancy_engine_delta_m3"]

        self.weight_force = self.mass * self.g
        self.neutral_buoyancy_force = self.rho * self.g * self.neutral_volume_m3
        self.max_buoyancy_delta = self.rho * self.g * (self.buoyancy_engine_delta_m3 / 2.0)

        self.act_min = cfg["actuator"]["min_us"]
        self.act_max = cfg["actuator"]["max_us"]
        self.act_neutral = cfg["actuator"]["neutral_us"]
        self.act_idle = cfg["actuator"]["idle_us"]
        self.act_direction = cfg["actuator"]["direction"]

        self.surface_pressure_mbar = cfg["sensor"]["surface_pressure_mbar"]
        self.depth_noise_std_m = cfg["sensor"]["depth_noise_std_m"]

        self.kp = cfg["pid"]["kp"]
        self.ki = cfg["pid"]["ki"]
        self.kd = cfg["pid"]["kd"]
        self.pid_interval = cfg["pid"]["interval_s"]
        self.pid_integral_min = cfg["pid"]["integral_min"]
        self.pid_integral_max = cfg["pid"]["integral_max"]

        self.deep_m = cfg["mission"]["deep_m"]
        self.shallow_m = cfg["mission"]["shallow_m"]
        self.surface_m = cfg["mission"]["surface_m"]
        self.hold_s = cfg["mission"]["hold_s"]
        self.tol_m = cfg["mission"]["tol_m"]
        self.surface_tol_m = cfg["mission"]["surface_tol_m"]

        # defaults if not present in config yet
        self.max_transit_s = cfg["mission"].get("max_transit_s", 90.0)
        self.max_hold_s = cfg["mission"].get("max_hold_s", 60.0)
        self.max_recover_s = cfg["mission"].get("max_recover_s", 90.0)
        self.self_recover = cfg["mission"].get("self_recover", True)

        self.t = 0.0
        self.true_depth = 0.0
        self.true_velocity = 0.0
        self.true_accel = 0.0
        self.measured_depth = 0.0
        self.actuator_us = self.act_idle

        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_timer = 0.0

        self.running = False

        self.current_state = self.DESCEND_1
        self.state_entry_time = 0.0
        self.in_tolerance_start = None
        self.current_target_depth = self.deep_m
        self.current_target_tol = self.tol_m

        self.time_hist = []
        self.depth_hist = []
        self.measured_depth_hist = []
        self.target_hist = []
        self.velocity_hist = []
        self.actuator_hist = []
        self.state_hist = []
        self.hold_hist = []

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def actuator_us_to_command(self, us):
        half_range = (self.act_max - self.act_min) / 2.0
        centered = us - self.act_neutral
        cmd = centered / half_range if half_range > 0 else 0.0
        cmd = self.clamp(cmd, -1.0, 1.0)
        return self.act_direction * cmd

    def state_name(self):
        return self.STATE_NAMES[self.current_state]

    def state_has_target(self):
        return self.current_state in {
            self.DESCEND_1,
            self.HOLD_DEEP_1,
            self.ASCEND_1,
            self.HOLD_SHALLOW_1,
            self.DESCEND_2,
            self.HOLD_DEEP_2,
            self.ASCEND_2,
            self.HOLD_SHALLOW_2,
            self.STATION_KEEP,
            self.RECOVER_SURFACE,
        }

    def get_target_for_state(self, state):
        if state in {self.DESCEND_1, self.HOLD_DEEP_1, self.DESCEND_2, self.HOLD_DEEP_2}:
            return self.deep_m, self.tol_m
        if state in {self.ASCEND_1, self.HOLD_SHALLOW_1, self.ASCEND_2, self.HOLD_SHALLOW_2, self.STATION_KEEP}:
            return self.shallow_m, self.tol_m
        if state == self.RECOVER_SURFACE:
            return self.surface_m, self.surface_tol_m
        return 0.0, self.tol_m

    def enter_state(self, new_state):
        self.current_state = new_state
        self.state_entry_time = self.t
        self.in_tolerance_start = None
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_timer = 0.0
        self.current_target_depth, self.current_target_tol = self.get_target_for_state(new_state)

    def state_time(self):
        return self.t - self.state_entry_time

    def in_tolerance(self):
        return abs(self.measured_depth - self.current_target_depth) <= self.current_target_tol

    def hold_elapsed(self):
        if self.in_tolerance_start is None:
            return 0.0
        return self.t - self.in_tolerance_start

    def step(self, dt):
        if not self.running:
            return

        self.t += dt

        # sensor
        self.measured_depth = self.true_depth + self.rng.normal(0.0, self.depth_noise_std_m)

        # update tolerance timer
        if self.state_has_target() and self.current_state in {
            self.HOLD_DEEP_1, self.HOLD_SHALLOW_1, self.HOLD_DEEP_2, self.HOLD_SHALLOW_2
        }:
            if self.in_tolerance():
                if self.in_tolerance_start is None:
                    self.in_tolerance_start = self.t
            else:
                self.in_tolerance_start = None

        # PID
        self.current_target_depth, self.current_target_tol = self.get_target_for_state(self.current_state)

        if self.current_state in {self.IDLE, self.DONE}:
            self.actuator_us = self.act_idle
        else:
            self.pid_timer += dt
            if self.pid_timer >= self.pid_interval:
                self.pid_timer = 0.0

                error = self.current_target_depth - self.measured_depth
                self.pid_integral += error * self.pid_interval
                self.pid_integral = self.clamp(
                    self.pid_integral,
                    self.pid_integral_min,
                    self.pid_integral_max
                )

                derivative = (error - self.pid_prev_error) / self.pid_interval
                self.pid_prev_error = error

                output_us = (
                    self.kp * error
                    + self.ki * self.pid_integral
                    + self.kd * derivative
                )

                self.actuator_us = self.act_neutral + output_us
                self.actuator_us = self.clamp(self.actuator_us, self.act_min, self.act_max)

        # mission state machine
        st = self.current_state
        timed_out_transit = self.state_time() >= self.max_transit_s
        timed_out_hold = self.state_time() >= self.max_hold_s
        timed_out_recover = self.state_time() >= self.max_recover_s

        if st == self.DESCEND_1:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_DEEP_1)

        elif st == self.HOLD_DEEP_1:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                self.enter_state(self.ASCEND_1)

        elif st == self.ASCEND_1:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_SHALLOW_1)

        elif st == self.HOLD_SHALLOW_1:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                self.enter_state(self.DESCEND_2)

        elif st == self.DESCEND_2:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_DEEP_2)

        elif st == self.HOLD_DEEP_2:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                self.enter_state(self.ASCEND_2)

        elif st == self.ASCEND_2:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_SHALLOW_2)

        elif st == self.HOLD_SHALLOW_2:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                if self.self_recover:
                    self.enter_state(self.RECOVER_SURFACE)
                else:
                    self.enter_state(self.STATION_KEEP)

        elif st == self.STATION_KEEP:
            pass

        elif st == self.RECOVER_SURFACE:
            if self.in_tolerance() or timed_out_recover:
                self.enter_state(self.DONE)
                self.actuator_us = self.act_idle

        # physics
        u = self.actuator_us_to_command(self.actuator_us)

        buoyancy_force = self.neutral_buoyancy_force + u * self.max_buoyancy_delta
        weight_force = self.weight_force

        drag_force = (
            -0.5
            * self.rho
            * self.drag_coefficient
            * self.reference_area_m2
            * self.true_velocity
            * abs(self.true_velocity)
        )

        net_force = weight_force - buoyancy_force + drag_force
        self.true_accel = net_force / self.mass
        self.true_velocity += self.true_accel * dt
        self.true_depth += self.true_velocity * dt

        if self.true_depth < 0.0:
            self.true_depth = 0.0
            self.true_velocity = 0.0

        # log
        self.time_hist.append(self.t)
        self.depth_hist.append(self.true_depth)
        self.measured_depth_hist.append(self.measured_depth)
        self.target_hist.append(self.current_target_depth if self.state_has_target() else 0.0)
        self.velocity_hist.append(self.true_velocity)
        self.actuator_hist.append(self.actuator_us)
        self.state_hist.append(self.current_state)
        self.hold_hist.append(self.hold_elapsed())

        max_points = 1500
        if len(self.time_hist) > max_points:
            self.time_hist = self.time_hist[-max_points:]
            self.depth_hist = self.depth_hist[-max_points:]
            self.measured_depth_hist = self.measured_depth_hist[-max_points:]
            self.target_hist = self.target_hist[-max_points:]
            self.velocity_hist = self.velocity_hist[-max_points:]
            self.actuator_hist = self.actuator_hist[-max_points:]
            self.state_hist = self.state_hist[-max_points:]
            self.hold_hist = self.hold_hist[-max_points:]


sim = VPSim(SELECTED_VP)

# -----------------------------
# Figure layout
# -----------------------------
fig = plt.figure(figsize=(15, 8))

# Leave space on the left for controls and on the bottom for buttons
gs = fig.add_gridspec(
    2, 3,
    left=0.22, right=0.98, top=0.92, bottom=0.12,
    width_ratios=[1.2, 1.2, 0.9],
    height_ratios=[1, 1],
    wspace=0.30, hspace=0.30
)

ax_depth_plot = fig.add_subplot(gs[0, 0:2])
ax_act_plot = fig.add_subplot(gs[1, 0:2])
ax_depth_graphic = fig.add_subplot(gs[:, 2])

fig.suptitle(f"VP Simulator - Step 1 - {SELECTED_VP}", fontsize=14)

# Depth plot
line_true_depth, = ax_depth_plot.plot([], [], label="True Depth (m)")
line_meas_depth, = ax_depth_plot.plot([], [], label="Measured Depth (m)")
line_target_depth, = ax_depth_plot.plot([], [], "--", label="Target Depth (m)")
ax_depth_plot.set_xlabel("Time (s)")
ax_depth_plot.set_ylabel("Depth (m)")
ax_depth_plot.invert_yaxis()
ax_depth_plot.grid(True)
ax_depth_plot.legend()

# Actuator plot
line_actuator, = ax_act_plot.plot([], [], label="Actuator (us)")
ax_act_plot.axhline(sim.act_neutral, linestyle="--", label="Neutral")
ax_act_plot.set_xlabel("Time (s)")
ax_act_plot.set_ylabel("Actuator (us)")
ax_act_plot.grid(True)
ax_act_plot.legend()

# Depth graphic
ax_depth_graphic.set_title("Depth View")
ax_depth_graphic.set_xlim(0, 1)
ax_depth_graphic.set_ylim(3.0, 0.0)
ax_depth_graphic.set_xticks([])
ax_depth_graphic.set_ylabel("Depth (m)")
ax_depth_graphic.grid(True, axis="y")

# Draw water column
ax_depth_graphic.fill_between([0.25, 0.75], [0, 0], [3.0, 3.0], alpha=0.15)

current_marker, = ax_depth_graphic.plot([0.5], [0.0], marker="o", markersize=10, linestyle="None", label="VP")
target_marker, = ax_depth_graphic.plot([0.5], [sim.current_target_depth], marker="x", markersize=10, linestyle="None", label="Target")
tol_top = sim.current_target_depth - sim.current_target_tol
tol_bot = sim.current_target_depth + sim.current_target_tol
tol_band = ax_depth_graphic.fill_between([0.35, 0.65], [tol_top, tol_top], [tol_bot, tol_bot], alpha=0.25)
ax_depth_graphic.legend(loc="upper right")

# Status text
status_text = ax_depth_graphic.text(
    0.05, 0.98, "",
    transform=ax_depth_graphic.transAxes,
    va="top",
    ha="left",
    fontsize=10,
    family="monospace"
)

# VP selector
ax_radio = fig.add_axes([0.03, 0.62, 0.15, 0.22])
radio_vp = RadioButtons(
    ax_radio,
    labels=list(VP_CONFIGS.keys()),
    active=list(VP_CONFIGS.keys()).index(SELECTED_VP)
)
ax_radio.set_title("VP Select", fontsize=10)

# Buttons along bottom-left
ax_btn_start = fig.add_axes([0.03, 0.04, 0.10, 0.05])
ax_btn_pause = fig.add_axes([0.14, 0.04, 0.10, 0.05])
ax_btn_reset = fig.add_axes([0.25, 0.04, 0.10, 0.05])
ax_btn_apply = fig.add_axes([0.03, 0.085, 0.12, 0.04])

btn_start = Button(ax_btn_start, "Start")
btn_pause = Button(ax_btn_pause, "Pause")
btn_reset = Button(ax_btn_reset, "Reset")
btn_apply = Button(ax_btn_apply, "Apply")

# Mission card
fig.text(0.03, 0.57, "Mission Settings", fontsize=10, weight="bold")

ax_box_deep = fig.add_axes([0.03, 0.53, 0.12, 0.035])
ax_box_shallow = fig.add_axes([0.03, 0.485, 0.12, 0.035])
ax_box_surface = fig.add_axes([0.03, 0.44, 0.12, 0.035])
ax_box_hold = fig.add_axes([0.03, 0.395, 0.12, 0.035])
ax_box_tol = fig.add_axes([0.03, 0.35, 0.12, 0.035])
ax_box_stol = fig.add_axes([0.03, 0.305, 0.12, 0.035])

box_deep = TextBox(ax_box_deep, "Deep m ", initial=str(sim.cfg["mission"]["deep_m"]))
box_shallow = TextBox(ax_box_shallow, "Shallow ", initial=str(sim.cfg["mission"]["shallow_m"]))
box_surface = TextBox(ax_box_surface, "Surface ", initial=str(sim.cfg["mission"]["surface_m"]))
box_hold = TextBox(ax_box_hold, "Hold s ", initial=str(sim.cfg["mission"]["hold_s"]))
box_tol = TextBox(ax_box_tol, "Tol m ", initial=str(sim.cfg["mission"]["tol_m"]))
box_stol = TextBox(ax_box_stol, "Surf tol ", initial=str(sim.cfg["mission"]["surface_tol_m"]))

# PID card
fig.text(0.03, 0.255, "PID Settings", fontsize=10, weight="bold")

ax_box_kp = fig.add_axes([0.03, 0.215, 0.12, 0.035])
ax_box_ki = fig.add_axes([0.03, 0.17, 0.12, 0.035])
ax_box_kd = fig.add_axes([0.03, 0.125, 0.12, 0.035])

box_kp = TextBox(ax_box_kp, "Kp ", initial=str(sim.cfg["pid"]["kp"]))
box_ki = TextBox(ax_box_ki, "Ki ", initial=str(sim.cfg["pid"]["ki"]))
box_kd = TextBox(ax_box_kd, "Kd ", initial=str(sim.cfg["pid"]["kd"]))

def on_start(event):
    sim.running = True

def on_pause(event):
    sim.running = False

def refresh_textboxes():
    box_deep.set_val(str(sim.cfg["mission"]["deep_m"]))
    box_shallow.set_val(str(sim.cfg["mission"]["shallow_m"]))
    box_surface.set_val(str(sim.cfg["mission"]["surface_m"]))
    box_hold.set_val(str(sim.cfg["mission"]["hold_s"]))
    box_tol.set_val(str(sim.cfg["mission"]["tol_m"]))
    box_stol.set_val(str(sim.cfg["mission"]["surface_tol_m"]))

    box_kp.set_val(str(sim.cfg["pid"]["kp"]))
    box_ki.set_val(str(sim.cfg["pid"]["ki"]))
    box_kd.set_val(str(sim.cfg["pid"]["kd"]))

def on_reset(event):
    sim.reset()
    refresh_textboxes()

def on_select_vp(label):
    sim.load_config(label)
    refresh_textboxes()

def on_apply(event):
    try:
        deep_m = float(box_deep.text)
        shallow_m = float(box_shallow.text)
        surface_m = float(box_surface.text)
        hold_s = float(box_hold.text)
        tol_m = float(box_tol.text)
        surface_tol_m = float(box_stol.text)

        kp = float(box_kp.text)
        ki = float(box_ki.text)
        kd = float(box_kd.text)

        sim.apply_settings(
            deep_m=deep_m,
            shallow_m=shallow_m,
            surface_m=surface_m,
            hold_s=hold_s,
            tol_m=tol_m,
            surface_tol_m=surface_tol_m,
            kp=kp,
            ki=ki,
            kd=kd,
        )
    except ValueError:
        print("Invalid input in mission/PID fields")

btn_start.on_clicked(on_start)
btn_pause.on_clicked(on_pause)
btn_reset.on_clicked(on_reset)
btn_apply.on_clicked(on_apply)
radio_vp.on_clicked(on_select_vp)

# -----------------------------
# Animation update
# -----------------------------
def update(frame):
    # Run a few physics steps per visual frame so it feels smoother
    for _ in range(3):
        sim.step(0.05)

    # Update depth plot
    line_true_depth.set_data(sim.time_hist, sim.depth_hist)
    line_meas_depth.set_data(sim.time_hist, sim.measured_depth_hist)
    line_target_depth.set_data(sim.time_hist, sim.target_hist)

    if len(sim.time_hist) > 2:
        ax_depth_plot.set_xlim(max(0, sim.time_hist[0]), sim.time_hist[-1] + 1.0)

        max_depth = max(max(sim.depth_hist), max(sim.target_hist), 1.0)
        ax_depth_plot.set_ylim(max_depth + 0.2, -0.05)

    # Update actuator plot
    line_actuator.set_data(sim.time_hist, sim.actuator_hist)
    if len(sim.time_hist) > 2:
        ax_act_plot.set_xlim(max(0, sim.time_hist[0]), sim.time_hist[-1] + 1.0)
        ax_act_plot.set_ylim(sim.act_min - 50, sim.act_max + 50)

    # Update depth graphic
    current_marker.set_data([0.5], [sim.true_depth])
    target_marker.set_data([0.5], [sim.current_target_depth])

    max_graph_depth = max(3.0, sim.current_target_depth + 1.0, sim.true_depth + 0.5)
    ax_depth_graphic.set_ylim(max_graph_depth, 0.0)

    # redraw tolerance band
    global tol_band
    try:
        tol_band.remove()
    except Exception:
        pass

    top = sim.current_target_depth - sim.current_target_tol
    bot = sim.current_target_depth + sim.current_target_tol
    tol_band = ax_depth_graphic.fill_between(
        [0.35, 0.65],
        [top, top],
        [bot, bot],
        alpha=0.25
    )

    status_text.set_text(
        f"VP: {sim.cfg_name}\n"
        f"running: {sim.running}\n"
        f"state: {sim.state_name()}\n"
        f"t: {sim.t:6.1f} s\n"
        f"depth: {sim.true_depth:6.3f} m\n"
        f"meas: {sim.measured_depth:6.3f} m\n"
        f"vel: {sim.true_velocity:6.3f} m/s\n"
        f"target: {sim.current_target_depth:6.3f} m\n"
        f"hold: {sim.hold_elapsed():6.1f} s\n"
        f"act: {sim.actuator_us:6.1f} us"
    )

    fig.suptitle(f"VP Simulator - Step 1 - {sim.cfg_name}", fontsize=14)

    return (
        line_true_depth,
        line_meas_depth,
        line_target_depth,
        line_actuator,
        current_marker,
        target_marker,
        status_text,
    )

ani = FuncAnimation(fig, update, interval=50, blit=False)
plt.show()
