from controller import Robot
import math
import random


# ──────────────────────────────────────────────────────────────────────────────
# PID CONTROLLER  (based on course material "Theorie regelaars" — unchanged)
# ──────────────────────────────────────────────────────────────────────────────
class PID_Controller:
    """
    PID controller per course material.
      Kp  = proportional gain
      Ki  = integral gain (0.0 = disabled)
      Kd  = derivative gain (0.0 = disabled)
      SP  = setpoint (desired value)
      LMN_HLM / LMN_LLM = high / low limit of controller output
    """
    def __init__(self, iKp, iKi, iKd, iSP, iLMN_HLM, iLMN_LLM):
        self.Kp       = iKp
        self.Ki       = iKi
        self.Kd       = iKd
        self.SP       = iSP
        self.prev_ER  = 0.0
        self.integral = 0.0
        self.LMN_HLM  = iLMN_HLM
        self.LMN_LLM  = iLMN_LLM

    def compute(self, iPV, iTimestep):
        ER = self.SP - iPV
        P_out = self.Kp * ER
        self.integral += ER * iTimestep
        I_out = self.Ki * self.integral
        derivative = (ER - self.prev_ER) / iTimestep if iTimestep > 0 else 0.0
        D_out = self.Kd * derivative
        oLMN = max(self.LMN_LLM, min(self.LMN_HLM, P_out + I_out + D_out))
        self.prev_ER = ER
        return oLMN

    def reset(self, new_SP=None):
        self.integral = 0.0
        self.prev_ER  = 0.0
        if new_SP is not None:
            self.SP = new_SP


# ──────────────────────────────────────────────────────────────────────────────
# HELPER FUNCTIONS
# ──────────────────────────────────────────────────────────────────────────────
def clamp(value, lo, hi):
    return max(lo, min(value, hi))

def angle_error(target, current):
    """
    Shortest angular distance [°] in [-180, 180].
    Positive = target is CW from current.
        err > 0 → CW  → set_motors(+, -)
        err < 0 → CCW → set_motors(-, +)
    """
    err = target - current
    while err >  180.0: err -= 360.0
    while err < -180.0: err += 360.0
    return err


# ──────────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ──────────────────────────────────────────────────────────────────────────────
MAX_SPEED        = 6.28
BATTERY_MAX      = 10_000.0
BATTERY_LOW_PCT  = 0.20          # return threshold: 20% remaining
ROAMING_DURATION = 1200.0         # 20 minutes per cleaning phase (safety backup)

# LiDAR sectors (256 rays, 180° FOV; index 0=LEFT, 128=FRONT, 255=RIGHT)
_L0,  _L1  =   0,  64
_FL0, _FL1 =  64, 112
_F0,  _F1  = 112, 144
_FR0, _FR1 = 144, 192
_R0,  _R1  = 192, 256

OBSTACLE_DIST     = 0.35   # stop/turn threshold for FRONT sectors
FAR_OBSTACLE_DIST = 0.28
SLOW_DIST         = 0.65

# Carpet threshold values (lookupTable: 0→1000, 0.1→0; floor baseline ≈ 650)
CARPET_FULL   = 730
CARPET_EDGE   = 670

# Carpet GPS bounds (rug: translation -1 1, size 2.5×1.8)
# Used for sensor-validation of carpet detection and MOPPING skip
CARPET_X_MIN  = -2.25
CARPET_X_MAX  =  0.30
CARPET_Y_MIN  =  0.05
CARPET_Y_MAX  =  1.95
CARPET_SAFE_X =  1.20   # safe target X (east of carpet) during carpet escape

# Docking geometrie
CHARGER_X           = 2.00
CHARGER_Y           = 2.85
PREDOCK_X           = 2.00
PREDOCK_Y           = 1.80
DOCK_ARM_Y          = 2.55
DOCK_CONFIRM_Y      = 2.70
DOCK_STALL_RADIUS_Y = 2.45

# Timing
ALIGNING_THRESHOLD = 6.0
ALIGNING_TIMEOUT   = 10.0
DOCKING_TIMEOUT    = 45.0
CHARGING_MAX_TIME  = 90.0

# Boustrophedon navigation (ClearView™ LiDAR style — "clean in neat rows")
# Source: irobot.com — "navigates with ClearView™ LiDAR, wall-to-wall"
BOUS_Y_MIN      = -2.10
BOUS_Y_MAX      =  1.80
BOUS_X_MIN      = -2.35
BOUS_X_MAX      =  2.35
BOUS_STRIP_STEP =  0.35
BOUS_WP_RADIUS  =  0.22

# If robot makes no progress for >WP_FRUSTRATION_TIME s → waypoint skipped.
WP_FRUSTRATION_TIME = 8.0

SPEED_RAMP      = 5.0    # acceleration ramp: MAX_SPEED/s
LOOK_AHEAD_DIST = 0.55   # m — start look-ahead to next waypoint within this distance

# Stuck detection: position-based (inspired by encoder counting, see classmate analysis)
# If robot moves less than STUCK_DIST m in >STUCK_TIMEOUT s → ESCAPE spin
STUCK_TIMEOUT   = 10.0   # seconds without STUCK_DIST movement → stuck
STUCK_DIST      = 0.05   # m minimum expected displacement in STUCK_TIMEOUT s

# Frustration timer threshold: only activate when robot is near waypoint
# (far away: obstacle avoidance is expected — no false positive frustration)
WP_NEAR_THRESH  = 1.5    # m — only frustration-tracking within this distance

# Passive LiDAR wall mapping (incremental, no dedicated scan phase)
# The robot collects LiDAR wall points DURING normal driving (VACUUMING).
# After the first full VACUUMING pass, room contours are computed and
# boustrophedon waypoints regenerated for the next cycle.
SCAN_MARGIN   = 0.30   # safety margin relative to detected wall points (m)

# Persistent carpet grid map (sensor-based recognition, MOPPING avoidance)
# Cell size 0.25m. Each CARPET_FULL hit adds cell(s) to carpet_map.
# VACUUMING builds the map; MOPPING uses it for WP exclusion and GPS monitoring.
CARPET_CELL_SIZE = 0.25   # m per grid cell for carpet map


# ──────────────────────────────────────────────────────────────────────────────
# CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
class RoombaController:

    CLEANING_STATES = ("VACUUMING", "MOPPING")

    def __init__(self, robot, timestep):
        self.robot    = robot
        self.timestep = timestep
        self.dt       = timestep / 1000.0

        # State machine
        self.state      = "UNDOCKING"
        self.prev_state = "VACUUMING"   # return-to state after ESCAPE

        # FEAT-1: Mission cycle
        self.mission_phase      = "VACUUMING"  # always start with vacuuming
        self.roaming_start_time = 0.0

        # UNDOCKING
        self.undock_phase = "REVERSE"
        self.undock_timer = 0.0

        # ESCAPE
        self.escape_reverse_end = 0.0
        self.escape_end_time    = 0.0
        self.turn_direction     = 1
        self.evasion_count      = 0
        self.last_evasion_time  = 0.0

        # Carpet escape (GPS-guided, MOPPING only)
        self.in_carpet_escape     = False
        self.carpet_escape_count  = 0
        # Escape direction: (tx, ty) target.
        # South (pos[0], Y_MIN-0.60) if robot is south of carpet center;
        # East (CARPET_SAFE_X, pos[1]) if robot is deep inside carpet.
        self.carpet_escape_target = (CARPET_SAFE_X, 0.0)

        # Boustrophedon (ClearView™ LiDAR navigation — straight rows, wall-to-wall)
        self.bous_waypoints = self._generate_waypoints()
        self.bous_wp_idx    = 0

        self.wp_prev_dist     = float('inf')
        self.wp_progress_time = 0.0   # timestamp of last meaningful progress

        # Persistente tapijt-gridkaart: set van (ix, iy) cel-indices bevestigd door sensor.
        # Filled during VACUUMING (sensor may detect carpet); used in
        # MOPPING for WP exclusion and proactive GPS monitoring.
        self.carpet_map = set()

        # Passive LiDAR wall mapping: wall points collected during normal driving.
        # After first full VACUUMING → room contours recomputed for next cycle.
        self.scan_wall_pts       = []    # wall points (wx, wy) in world coordinates
        self.room_bounds_computed = False  # True after first VACUUMING cycle
        self.last_scan_collect_t  = 0.0   # timestamp of last wall point collection

        # Persistent map knowledge: tracked cleaned/processed waypoints per phase.
        # Survives charging cycles so the robot RESUMES instead of restarting.
        self.wp_done = {"VACUUMING": set(), "MOPPING": set()}

        # Missievoltooiingsteller: telt het aantal volledig afgeronde reinigingsfases.
        # At 2 (VACUUMING + MOPPING fully done) robot transitions to FINISHED.
        self.phases_completed = 0

        # WP skip counter per phase (reset on phase start, logged in summary)
        self.wp_skipped_count = 0

        # Exploration factor: cautious start at new phase, ramp to full speed.
        # 0.5 = half speed (exploration), 1.0 = full drive speed (systematic)
        self.exploration_factor = 1.0   # start full at first run (UNDOCKING sets this)
        self.fresh_phase        = True   # True = first start of a new phase

        self.cleaning_speed = 0.0     # current ramped drive speed (rad/s)

        # Stuck detection (position-based)
        self.stuck_check_pos  = None
        self.stuck_check_time = 0.0

        # RETURNING timer (adaptive tolerance) + intermediate via-point
        self.returning_start_time = 0.0
        self.returning_via        = None

        # PID heading control (per course material, unchanged)
        self.pid_bearing = PID_Controller(
            iKp=0.045, iKi=0.0, iKd=0.001,
            iSP=0.0,
            iLMN_HLM= 0.45 * MAX_SPEED,
            iLMN_LLM=-0.45 * MAX_SPEED
        )

        # Docking
        self.aligning_start_time = 0.0
        self.docking_start_time  = 0.0
        self.dock_last_y         = 0.0
        self.dock_stall_timer    = 0.0

        # Charging
        self.charging_start_time = 0.0
        self.last_charge_time    = None

        # Telemetry
        self.last_logged_pct = 110
        self.last_carpet_log = -99.0
        self.last_escape_log_t  = -99.0   # rate-limit ESCAPE log spam
        self.escape_log_count   = 0        # consecutive escapes since last log
        self.last_carpet_log_t  = -99.0   # rate-limit carpet-escape log
        self.carpet_log_count   = 0        # carpet escapes since last log

        # Watchdog
        self.prev_cycle_time = 0.0

        self._setup_devices()
        self._set_leds("UNDOCKING")
        print(f"[SYSTEM] Controller v5.12 | {BATTERY_MAX:.0f} J battery | "
              f"return at {BATTERY_LOW_PCT*100:.0f}% | 20 min per phase")
        print(f"[MISSION] Starting phase 1: {self.mission_phase} | "
              f"{len(self._generate_waypoints())} waypoints | "
              f"WP radius: {BOUS_WP_RADIUS}m")

    # ── devices ───────────────────────────────────────────────────────────────
    def _setup_devices(self):
        self.motor_left  = self.robot.getDevice('motor_left')
        self.motor_right = self.robot.getDevice('motor_right')
        for m in (self.motor_left, self.motor_right):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        self.lidar = self.robot.getDevice('clearview_lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.robot.batterySensorEnable(self.timestep)

        self.bumper = self.robot.getDevice('bumper')
        self.bumper.enable(self.timestep)

        self.wall_sensor = self.robot.getDevice('wall_sensor')
        self.wall_sensor.enable(self.timestep)

        self.carpet_sensor = self.robot.getDevice('carpet_sensor')
        self.carpet_sensor.enable(self.timestep)

        self.cliff_sensors = []
        for name in ('cliff_sensor_left', 'cliff_sensor_right',
                     'cliff_sensor_front_left', 'cliff_sensor_front_right'):
            s = self.robot.getDevice(name)
            s.enable(self.timestep)
            self.cliff_sensors.append(s)

        self.gps     = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        self.status_led  = self.robot.getDevice('status_led')
        self.battery_led = self.robot.getDevice('battery_led')

    # ── actuators ────────────────────────────────────────────────────────────
    def set_motors(self, left, right):
        self.motor_left.setVelocity(clamp(left,  -MAX_SPEED, MAX_SPEED))
        self.motor_right.setVelocity(clamp(right, -MAX_SPEED, MAX_SPEED))

    def _set_leds(self, state):
        """
        LED-sturing per state (conform system requirements):
          VACUUMING : status on         (actively vacuuming)
          MOPPING   : both on           (mopping mode visible)
          ESCAPE     : beide uit         (noodstop)
          RETURNING  : beide aan         (terugkeer naar lader)
          ALIGNING   : status aan        (uitlijnen)
          DOCKING    : status aan        (insturen lader)
          CHARGING   : beide uit         (stilstaand laden)
          UNDOCKING  : status aan        (losschieten lader)
          FINISHED  : both on           (mission successfully completed)
        """
        leds = {
            "VACUUMING": (1, 0),
            "MOPPING":    (1, 1),
            "ESCAPE":     (0, 0),
            "RETURNING":  (1, 1),
            "ALIGNING":   (1, 0),
            "DOCKING":    (1, 0),
            "CHARGING":   (0, 0),
            "UNDOCKING":  (1, 0),
            "FINISHED":   (1, 1),   # both LEDs on: mission fully completed
        }
        s, b = leds.get(state, (1, 0))
        self.status_led.set(s)
        self.battery_led.set(b)

    # ── bearing ───────────────────────────────────────────────────────────────
    def get_bearing(self):
        val = self.compass.getValues()
        if not val or math.isnan(val[0]):
            return 0.0
        deg = math.degrees(math.atan2(val[1], val[0]))
        return deg + 360.0 if deg < 0.0 else deg

    # ── lidar ─────────────────────────────────────────────────────────────────
    def _lidar_min(self, start, end):
        sector = [r for r in self.ranges[start:end] if 0.02 < r != float('inf')]
        return min(sector) if sector else float('inf')

    def _read_lidar_sectors(self):
        if not self.ranges:
            self.d_far_left = self.d_front_left = self.d_front = \
                self.d_front_right = self.d_far_right = float('inf')
            return
        self.d_far_left    = self._lidar_min(_L0,  _L1)
        self.d_front_left  = self._lidar_min(_FL0, _FL1)
        self.d_front       = self._lidar_min(_F0,  _F1)
        self.d_front_right = self._lidar_min(_FR0, _FR1)
        self.d_far_right   = self._lidar_min(_R0,  _R1)

    # ── boustrophedon waypoint-generatie ──────────────────────────────────────
    def _generate_waypoints(self):
        return self._generate_boustrophedon(
            BOUS_X_MIN, BOUS_X_MAX, BOUS_Y_MIN, BOUS_Y_MAX
        )

    def _generate_mopping_waypoints(self):
        """
        Generate MOPPING-specific waypoints that avoid the carpet.
        """
        cy_min = self._min_carpet_y()
        cy_max = self._max_carpet_y()
        cx_east = self._carpet_x_east()
        buffer = 0.20

        cy_lo = cy_min - buffer   # lower Y-bound of carpet zone (with buffer)
        cy_hi = cy_max + buffer   # upper Y-bound of carpet zone (with buffer)

        pts = []
        y = BOUS_Y_MIN
        going_east = True

        while y <= BOUS_Y_MAX + 0.01:
            yr = round(y, 2)

            # Charger corner exclusion: same as in _generate_boustrophedon
            if yr > CHARGER_Y - 0.80:
                x_east = min(BOUS_X_MAX, CHARGER_X - 0.50)
            else:
                x_east = BOUS_X_MAX

            # Determine west bound: in carpet Y-zone → east part only, else full width
            in_carpet_zone = cy_lo <= yr <= cy_hi
            x_west = cx_east if in_carpet_zone else BOUS_X_MIN

            if going_east:
                pts.append((x_west, yr))
                pts.append((x_east, yr))
            else:
                pts.append((x_east, yr))
                pts.append((x_west, yr))

            y += BOUS_STRIP_STEP
            going_east = not going_east

        n_full = sum(1 for (_, yy) in pts[::2] if not (cy_lo <= yy <= cy_hi))
        n_east = sum(1 for (_, yy) in pts[::2] if (cy_lo <= yy <= cy_hi))
        print(f"[MOPPING] {len(pts)} waypoints: {n_full} full-width + "
              f"{n_east} east-only strips (carpet zone {cy_lo:.2f}..{cy_hi:.2f}m)")
        return pts

    def _generate_boustrophedon(self, x_min, x_max, y_min, y_max):
        """
        Generate a uniform zigzag grid pattern within the given room bounds.

        Based on the ClearView™ LiDAR navigation principle of the real Roomba® 205:
        'maximizes floor-cleaning coverage wall-to-wall, cleans in neat rows'
        (source: irobot.com)

        Called with hardcoded BOUS_* values as fallback at initialization,
        and with dynamically measured bounds after SCANNING state so the controller
        works correctly regardless of room size.

        Obstacles are reactively avoided (LiDAR + frustration timeout) — no
        hardcoded furniture zones needed.
        """
        pts = []
        y = y_min
        going_east = True
        while y <= y_max + 0.01:
            yr = round(y, 2)
            # Charger corner exclusion: the east end of strips near the
            # charger (Y > CHARGER_Y - 0.80 = 2.05m) is clamped to
            # CHARGER_X - 0.50 = 1.50m to avoid the charger arm.
            # This prevents WPs from being generated in the blocked northeast corner.
            if yr > CHARGER_Y - 0.80:
                x_east = min(x_max, CHARGER_X - 0.50)
            else:
                x_east = x_max
            if going_east:
                pts.append((x_min, yr))
                pts.append((x_east, yr))
            else:
                pts.append((x_east, yr))
                pts.append((x_min, yr))
            y += BOUS_STRIP_STEP
            going_east = not going_east
        return pts

    def _compute_room_bounds(self, keep_wp_done=False):
        """
        Compute room bounds from collected LiDAR wall points (SCANNING state).

        Method: 5th/95th percentile filtering of X and Y coordinates.
        This removes outliers from furniture or floor irregularities and gives
        a reliable estimate of the actual wall positions.

        After computation, bous_waypoints are regenerated and wp_done reset,
        so the robot covers the full room in strips.
        """
        if len(self.scan_wall_pts) < 50:
            print(f"[PASSIVE SCAN] Too few points ({len(self.scan_wall_pts)}) "
                  f"→ keeping default room dimensions")
            return

        xs = sorted(p[0] for p in self.scan_wall_pts)
        ys = sorted(p[1] for p in self.scan_wall_pts)
        n  = len(xs)

        # 5th/95th percentile: robust against outliers from furniture
        p5  = max(0, n // 20)
        p95 = min(n - 1, n - n // 20)

        raw_x_min = xs[p5]
        raw_x_max = xs[p95]
        raw_y_min = ys[p5]
        raw_y_max = ys[p95]

        # Add safety margin (SCAN_MARGIN) so robot does not get too close to wall
        new_x_min = raw_x_min + SCAN_MARGIN
        new_x_max = raw_x_max - SCAN_MARGIN
        new_y_min = raw_y_min + SCAN_MARGIN
        new_y_max = raw_y_max - SCAN_MARGIN

        # Guard minimum room size (prevents degeneration from bad scan)
        if new_x_max - new_x_min < 1.0 or new_y_max - new_y_min < 1.0:
            print(f"[PASSIVE SCAN] Unreasonable room bounds "
                  f"X=[{new_x_min:.2f},{new_x_max:.2f}], "
                  f"Y=[{new_y_min:.2f},{new_y_max:.2f}] → keeping defaults")
            return

        print(f"[SCANNING] Room discovered: "
              f"X=[{new_x_min:.2f}, {new_x_max:.2f}], "
              f"Y=[{new_y_min:.2f}, {new_y_max:.2f}] "
              f"({n} points, margin={SCAN_MARGIN}m)")

        old_count = len(self.bous_waypoints)
        self.bous_waypoints = self._generate_boustrophedon(
            new_x_min, new_x_max, new_y_min, new_y_max
        )
        if not keep_wp_done:
            self.bous_wp_idx = 0
            self.wp_done = {"VACUUMING": set(), "MOPPING": set()}
        print(f"[PASSIVE SCAN] Waypoints updated: "
              f"{old_count} → {len(self.bous_waypoints)} WP "
              f"({'wp_done kept' if keep_wp_done else 'fresh start'})")

    # ── sensors ───────────────────────────────────────────────────────────────
    def _read_sensors(self):
        self.battery    = self.robot.batterySensorGetValue()
        self.wall_val   = self.wall_sensor.getValue()
        self.carpet_val = self.carpet_sensor.getValue()
        self.ranges     = self.lidar.getRangeImage()
        self.bumper_hit = self.bumper.getValue()
        self.cliff_hit  = any(s.getValue() < 100.0 for s in self.cliff_sensors)
        self.pos        = self.gps.getValues()
        self.bearing    = self.get_bearing()
        self._read_lidar_sectors()

        pct = int((self.battery / BATTERY_MAX) * 100) if self.battery >= 0 else 0
        _active = self.state in ("VACUUMING", "MOPPING", "RETURNING", "ESCAPE", "UNDOCKING")
        if (_active and pct != self.last_logged_pct
                and pct in (50, 20) and 0 <= pct <= 100):
            print(f"[BATTERY] {pct}%")
            self.last_logged_pct = pct
        if pct <= 20 and self.state not in ("CHARGING", "DOCKING"):
            self.battery_led.set(1)

        # ── Persistent carpet grid map: update ONLY during VACUUMING ──
        if self.state == "VACUUMING" and self.carpet_val > CARPET_FULL:
            bearing_rad = math.radians(self.bearing)
            sx = self.pos[0] + 0.16 * math.sin(bearing_rad)
            sy = self.pos[1] + 0.16 * math.cos(bearing_rad)
            cx = int(sx / CARPET_CELL_SIZE)
            cy = int(sy / CARPET_CELL_SIZE)
            cell = (cx, cy)
            if cell not in self.carpet_map:
                self.carpet_map.add(cell)
                pass  # carpet map updated silently

    # ── watchdog ──────────────────────────────────────────────────────────────
    def _watchdog(self, t):
        elapsed = t - self.prev_cycle_time
        if elapsed > 0.150 and self.prev_cycle_time > 0:
            print(f"[WATCHDOG] Cycle time exceeded: {elapsed:.3f}s")
        self.prev_cycle_time = t

    # ── emergency stop ────────────────────────────────────────────────────────
    def _check_emergencies(self, t):
        """Bumper or cliff → ESCAPE. Active in cleaning states and RETURNING."""
        if self.state not in (*self.CLEANING_STATES, "RETURNING"):
            return

        # Raised to 15% (was 8%): logs show bumper hits during RETURNING
        # cause ESCAPE loops that fully drain remaining battery.
        # Robot drives directly to charger; collision risk is less than guaranteed
        # energy loss from an endless ESCAPE spiral.
        if self.state == "RETURNING" and self.battery < 0.15 * BATTERY_MAX:
            return

        trigger = ""
        if self.bumper_hit > 0.0:
            trigger = "BUMPER"
        elif self.cliff_hit:
            trigger = "CLIFF"
        if not trigger:
            return

        battery_low = self.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up     = (t - self.roaming_start_time) >= ROAMING_DURATION
        self.prev_state = "RETURNING" if (battery_low or time_up) else self.state

        self.in_carpet_escape = False

        # Rate-limit escape logging (suppress repeated identical escapes)
        self.escape_log_count += 1
        if t - self.last_escape_log_t > 30.0:
            count_str = f" ×{self.escape_log_count}" if self.escape_log_count > 1 else ""
            print(f"[ESCAPE] {trigger}{count_str} in {self.state}")
            self.last_escape_log_t = t
            self.escape_log_count  = 0
        self.state = "ESCAPE"
        self._set_leds("ESCAPE")

        if t - self.last_evasion_time < 6.0:
            self.evasion_count += 1
        else:
            self.evasion_count = 1
        self.last_evasion_time = t

        if self.evasion_count >= 3:
            print("[WATCHDOG] Corner situation → 180° rotation.")
            self.escape_reverse_end = t + 1.0
            self.escape_end_time    = self.escape_reverse_end + 2.2
            self.turn_direction     = 1
            self.evasion_count      = 0
        else:
            self.escape_reverse_end = t + 1.0
            self.escape_end_time    = self.escape_reverse_end + random.uniform(0.8, 1.6)
            if self.wall_val > 200.0:
                self.turn_direction = -1
            elif self.d_far_right < self.d_far_left:
                self.turn_direction = -1
            else:
                self.turn_direction = 1

    # ── carpet escape (MOPPING only) ──────────────────────────────────────────
    def _on_carpet_gps(self):
        return (CARPET_X_MIN <= self.pos[0] <= CARPET_X_MAX and
                CARPET_Y_MIN <= self.pos[1] <= CARPET_Y_MAX)

    def _wp_in_carpet_zone(self, wx, wy, buffer=0.20):
        """
        True if waypoint (wx, wy) falls within the carpet Y-band + buffer.

        buffer = 0.20m: safety margin south/north of carpet Y bounds.
        """
        if not self.carpet_map:
            return CARPET_Y_MIN - buffer <= wy <= CARPET_Y_MAX + buffer

        cy_min = self._min_carpet_y()
        cy_max = self._max_carpet_y()
        return (cy_min - buffer) <= wy <= (cy_max + buffer)

    def _min_carpet_y(self):
        """Lowest Y-coordinate of mapped carpet cells (in meters)."""
        if not self.carpet_map:
            return CARPET_Y_MIN
        return min(cy * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def _max_carpet_y(self):
        """Highest Y-coordinate of mapped carpet cells (in meters)."""
        if not self.carpet_map:
            return CARPET_Y_MAX
        return max(cy * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def _carpet_x_east(self):
        """
        Safe start-X for east-only MOPPING strips.
        """
        if not self.carpet_map:
            return CARPET_X_MAX + 0.30          # = 0.60m as fallback
        max_cx = max(cx for (cx, _) in self.carpet_map)
        return (max_cx + 1) * CARPET_CELL_SIZE + 0.20  # cell upper bound + buffer

    def _handle_carpet_escape(self, t):
        """
        Sensor-driven carpet avoidance for MOPPING.
        Vuurt als carpet_sensor boven drempel of GPS meldt robot op tapijt.
        Altijd SOUTH ontsnappen: robot rijdt naar Y = carpet_y_min - 0.50.
        """
        carpet_y_min = self._min_carpet_y()

        gps_on   = self._on_carpet_gps()
        gps_near = (-2.55 <= self.pos[0] <= 0.55 and
                    CARPET_Y_MIN - 0.20 <= self.pos[1] <= CARPET_Y_MAX + 0.25)
        trigger  = (self.carpet_val > CARPET_FULL or
                    (self.carpet_val > CARPET_EDGE and gps_near))

        if not self.in_carpet_escape and (trigger or gps_on):
            self.in_carpet_escape    = True
            self.carpet_escape_count += 1
            target_y = carpet_y_min - 0.50
            self.carpet_escape_target = (self.pos[0], target_y)
            self.last_carpet_log = t  # carpet avoidance handled silently

        if not self.in_carpet_escape:
            return False

        # Free condition: sensor clear + GPS confirms outside carpet
        sensor_clear = self.carpet_val < (CARPET_EDGE - 30)
        if sensor_clear and not gps_on:
            self.in_carpet_escape = False
            return False

        # Navigate to escape target (south)
        tx, ty = self.carpet_escape_target
        dx = tx - self.pos[0]
        dy = ty - self.pos[1]
        tb = math.degrees(math.atan2(dx, dy))
        if tb < 0.0:
            tb += 360.0
        err  = angle_error(tb, self.bearing)
        turn = clamp(0.05 * err, -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)
        self.set_motors(0.45 * MAX_SPEED + turn, 0.45 * MAX_SPEED - turn)
        return True

    # ── shared cleaning logic ─────────────────────────────────────────────────
    def _execute_cleaning(self, t, allow_carpet):
        """
        Shared drive logic for VACUUMING and MOPPING.
        allow_carpet=True  → carpet allowed (VACUUMING)
        allow_carpet=False → avoid carpet (MOPPING)
        """
        # ── Priority 0: Phase fully done → return immediately ──────────
        # If all waypoints in the current phase are processed, the robot
        # does not need to wait for the battery to drain. Go directly to
        # RETURNING so CHARGING can handle the phase switch.
        
        phase_key = "VACUUMING" if allow_carpet else "MOPPING"
        if len(self.wp_done[phase_key]) >= len(self.bous_waypoints):
            _skip_str = f" | {self.wp_skipped_count} WPs skipped" if self.wp_skipped_count else ""
            print(f"[MISSION] {phase_key} complete "
                  f"({len(self.wp_done[phase_key])}/{len(self.bous_waypoints)} WP{_skip_str})")
            self.pid_bearing.reset()
            self.in_carpet_escape     = False
            self.returning_start_time = 0.0
            self.cleaning_speed       = 0.0
            self.state = "RETURNING"
            self._set_leds("RETURNING")
            return

        # ── Priority 1: Check battery and time limit ─────────────────────
        battery_low = self.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up     = (t - self.roaming_start_time) >= ROAMING_DURATION

        if battery_low or time_up:
            reason = "time limit 20 min" if time_up else f"low battery ({BATTERY_LOW_PCT*100:.0f}%)"
            print(f"[STATE CHANGE] {self.state} → RETURNING ({reason})")
            self.pid_bearing.reset()
            self.in_carpet_escape     = False
            self.returning_start_time = 0.0
            self.cleaning_speed       = 0.0
            self.state = "RETURNING"
            self._set_leds("RETURNING")
            return

        # ── Priority 2: Carpet avoidance (MOPPING only) ──────────────────
        if not allow_carpet:
            if self._handle_carpet_escape(t):
                return

        # ── Priority 3: LiDAR obstacle avoidance ─────────────────────────
        obs_front       = self.d_front       < OBSTACLE_DIST
        obs_front_left  = self.d_front_left  < OBSTACLE_DIST
        obs_front_right = self.d_front_right < OBSTACLE_DIST
        obs_far_left    = self.d_far_left    < FAR_OBSTACLE_DIST
        obs_far_right   = self.d_far_right   < FAR_OBSTACLE_DIST
        slow_front      = self.d_front       < SLOW_DIST

        any_obstacle = (obs_front or obs_front_left or obs_front_right
                        or obs_far_left or obs_far_right)

        if any_obstacle:
            # free driving robot restarts smoothly instead of abruptly.
            self.cleaning_speed = max(0.0,
                                      self.cleaning_speed - 0.15 * MAX_SPEED)

        if obs_front:
            # Obstacle directly ahead → turn toward most open side
            if self.d_far_left >= self.d_far_right:
                self.set_motors(-0.20 * MAX_SPEED, 0.55 * MAX_SPEED)
            else:
                self.set_motors(0.55 * MAX_SPEED, -0.20 * MAX_SPEED)

        elif obs_front_left and obs_front_right:
            # If there is space ahead but both front-side sectors are blocked
            # → creep forward slowly instead of turning.
            if self.d_front > 0.50:
                self.set_motors(0.25 * MAX_SPEED, 0.25 * MAX_SPEED)
            else:
                if self.d_far_left >= self.d_far_right:
                    self.set_motors(-0.20 * MAX_SPEED, 0.55 * MAX_SPEED)
                else:
                    self.set_motors(0.55 * MAX_SPEED, -0.20 * MAX_SPEED)

        elif obs_front_left and not obs_front_right:
            self.set_motors(0.58 * MAX_SPEED, 0.12 * MAX_SPEED)

        elif obs_front_right and not obs_front_left:
            self.set_motors(0.12 * MAX_SPEED, 0.58 * MAX_SPEED)

        elif obs_far_left and not obs_far_right:
            self.set_motors(0.52 * MAX_SPEED, 0.30 * MAX_SPEED)

        elif obs_far_right and not obs_far_left:
            self.set_motors(0.30 * MAX_SPEED, 0.52 * MAX_SPEED)

        elif slow_front:
            base = 0.35 * MAX_SPEED
            if self.d_front_left >= self.d_front_right:
                self.set_motors(base * 0.6, base)
            else:
                self.set_motors(base, base * 0.6)

        else:
            # ── Prioriteit 4: Boustrophedon GPS-navigatie ─────────────────
            # "Cleans in neat rows, wall-to-wall" — irobot.com / ClearView™ LiDAR

            wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
            dx_wp  = wp_x - self.pos[0]
            dy_wp  = wp_y - self.pos[1]
            dist_wp = math.hypot(dx_wp, dy_wp)

            if dist_wp < WP_NEAR_THRESH:
                if self.wp_progress_time == 0.0:
                    self.wp_progress_time = t
                    self.wp_prev_dist     = dist_wp

                if dist_wp < self.wp_prev_dist - 0.04:
                    # Meaningful progress (>4cm closer to waypoint)
                    self.wp_progress_time = t
                    self.wp_prev_dist     = dist_wp
                elif (t - self.wp_progress_time) > WP_FRUSTRATION_TIME:
                    # Near but stuck: obstacle blocking waypoint (silent, counted)
                    old_wp    = self.bous_wp_idx
                    phase_key = "VACUUMING" if allow_carpet else "MOPPING"
                    self.wp_done[phase_key].add(old_wp)
                    self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                    self.wp_skipped_count += 1
                    self.wp_progress_time = t
                    self.wp_prev_dist     = float('inf')
                    wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                    dx_wp  = wp_x - self.pos[0]
                    dy_wp  = wp_y - self.pos[1]
                    dist_wp = math.hypot(dx_wp, dy_wp)
            else:
                # Far from waypoint: reset frustration timer and navigate normally
                self.wp_progress_time = 0.0
                self.wp_prev_dist     = dist_wp

            # ── Waypoint reached → mark as done, go to next ──────────────
            if dist_wp < BOUS_WP_RADIUS:
                phase_key = "VACUUMING" if allow_carpet else "MOPPING"
                self.wp_done[phase_key].add(self.bous_wp_idx)
                self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                self.wp_progress_time = t
                self.wp_prev_dist     = float('inf')
                # Log each new strip (every 2 WPs = 1 strip); skip if phase just finished
                if self.bous_wp_idx % 2 == 0 and self.bous_wp_idx != 0:
                    next_wp = self.bous_waypoints[self.bous_wp_idx]
                    strip_n = self.bous_wp_idx // 2
                    total_s = len(self.bous_waypoints) // 2
                    n_done  = len(self.wp_done[phase_key])
                    skipped_str = f" | {self.wp_skipped_count} skipped" if self.wp_skipped_count else ""
                    print(f"[NAV] Strip {strip_n}/{total_s} → Y={next_wp[1]:.2f} "
                          f"[{n_done}/{len(self.bous_waypoints)} WP{skipped_str}]")
                wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                dx_wp  = wp_x - self.pos[0]
                dy_wp  = wp_y - self.pos[1]
                dist_wp = math.hypot(dx_wp, dy_wp)

            # ── Heading calculation to waypoint ──────────────────────────
            target_deg = math.degrees(math.atan2(dx_wp, dy_wp))
            if target_deg < 0.0:
                target_deg += 360.0
            if dist_wp < LOOK_AHEAD_DIST:
                next_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                nwx, nwy = self.bous_waypoints[next_idx]
                ndx = nwx - self.pos[0]
                ndy = nwy - self.pos[1]
                next_deg = math.degrees(math.atan2(ndx, ndy))
                if next_deg < 0.0:
                    next_deg += 360.0
                # blend: 0.0 at LOOK_AHEAD_DIST, max 0.50 at waypoint
                blend = clamp(1.0 - dist_wp / LOOK_AHEAD_DIST, 0.0, 0.50)
                diff  = angle_error(next_deg, target_deg)
                target_deg = (target_deg + blend * diff) % 360.0

            bearing_err = angle_error(target_deg, self.bearing)
            turn = clamp(0.040 * bearing_err, -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)

            # Wall sensor P-correction (sensor used for wall-distance correction)
            # Detects right wall for gentle heading correction
            wall_corr = 0.0
            if self.wall_val > 80.0:
                wall_err  = 380.0 - self.wall_val
                wall_corr = clamp(0.0008 * wall_err,
                                  -0.06 * MAX_SPEED, 0.06 * MAX_SPEED)

            total_turn = clamp(turn + wall_corr,
                               -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)

            if self.exploration_factor < 1.0:
                self.exploration_factor = min(
                    1.0,
                    self.exploration_factor + (1.0 - self.exploration_factor) * 0.025 * self.dt
                )
            target_speed = 0.55 * MAX_SPEED * self.exploration_factor
            ramp_delta   = SPEED_RAMP * self.dt
            self.cleaning_speed = clamp(
                self.cleaning_speed + clamp(
                    target_speed - self.cleaning_speed,
                    -ramp_delta, ramp_delta
                ),
                0.0, MAX_SPEED
            )

            self.set_motors(self.cleaning_speed + total_turn,
                            self.cleaning_speed - total_turn)

            # ── Passive wall mapping (VACUUMING only, max 1×/5s) ────────
            # Collect LiDAR wall points during normal driving without a dedicated
            # scan phase.
            if allow_carpet and not self.room_bounds_computed:
                if t - self.last_scan_collect_t >= 5.0 and self.ranges:
                    self.last_scan_collect_t = t
                    bearing_rad = math.radians(self.bearing)
                    sensor_x = self.pos[0] + 0.15 * math.sin(bearing_rad)
                    sensor_y = self.pos[1] + 0.15 * math.cos(bearing_rad)
                    n_rays = len(self.ranges)
                    for i, r in enumerate(self.ranges):
                        if 0.10 < r < 5.0:
                            offset_deg     = -90.0 + (180.0 / max(n_rays - 1, 1)) * i
                            wb_rad         = bearing_rad + math.radians(offset_deg)
                            self.scan_wall_pts.append((
                                sensor_x + r * math.sin(wb_rad),
                                sensor_y + r * math.cos(wb_rad)
                            ))

    # ── stuck detection ───────────────────────────────────────────────────────
    def _check_stuck(self, t):
        """
        Position-based stuck detection (inspired by classmate: encoder counting).
        If robot moves less than STUCK_DIST m in >STUCK_TIMEOUT s during
        a cleaning state → ESCAPE spin to break the deadlock.
        """
        if self.state not in self.CLEANING_STATES:
            # Outside cleaning states: reset tracker so it starts fresh
            self.stuck_check_pos  = None
            self.stuck_check_time = 0.0
            return

        if self.stuck_check_pos is None:
            self.stuck_check_pos  = (self.pos[0], self.pos[1])
            self.stuck_check_time = t
            return

        moved = math.hypot(self.pos[0] - self.stuck_check_pos[0],
                           self.pos[1] - self.stuck_check_pos[1])
        if moved > STUCK_DIST:
            # Sufficient movement → reset timer
            self.stuck_check_pos  = (self.pos[0], self.pos[1])
            self.stuck_check_time = t
        elif t - self.stuck_check_time > STUCK_TIMEOUT:
            print(f"[STUCK] No {STUCK_DIST}m movement in {STUCK_TIMEOUT:.0f}s "
                  f"(pos={self.pos[0]:.2f},{self.pos[1]:.2f}) → ESCAPE spin")
            self.prev_state         = self.state
            self.state              = "ESCAPE"
            self.escape_reverse_end = t + 0.3
            self.escape_end_time    = t + 0.3 + 1.5
            self.turn_direction     = 1 if random.random() > 0.5 else -1
            self.cleaning_speed     = 0.0
            self.wp_progress_time   = 0.0
            self.wp_prev_dist       = float('inf')
            self.stuck_check_pos    = None
            self.stuck_check_time   = 0.0
            self._set_leds("ESCAPE")

    # ── state machine ─────────────────────────────────────────────────────────
    def _execute_state_machine(self, t):

        # ── UNDOCKING ─────────────────────────────────────────────────────────
        if self.state == "UNDOCKING":
            if self.undock_timer == 0.0:
                self.undock_timer = t

            if self.undock_phase == "REVERSE":
                self.set_motors(-0.45 * MAX_SPEED, -0.45 * MAX_SPEED)
                if self.pos[1] < 2.1 or (t - self.undock_timer > 4.0):
                    self.undock_phase = "TURN"
                    self.undock_timer = t

            elif self.undock_phase == "TURN":
                err = angle_error(180.0, self.bearing)
                if abs(err) < 2.5 or (t - self.undock_timer > 5.0):
                    # Directly to cleaning phase — no dedicated scan phase.
                    # Mapping happens incrementally during driving (passive scan).
                    self.roaming_start_time   = t
                    self.returning_start_time = 0.0
                    self.returning_via        = None
                    self.in_carpet_escape     = False
                    self.cleaning_speed   = 0.0
                    self.wp_progress_time = 0.0
                    self.wp_prev_dist     = float('inf')

                    # Persistent navigation: resume at first UNCLEANED waypoint.
                    # This means the robot does NOT restart after charging,
                    # but continues from where it left off (like real Roomba).
                    phase_done = self.wp_done[self.mission_phase]
                    n_wps      = len(self.bous_waypoints)
                    found_next = False
                    for i in range(n_wps):
                        idx = (self.bous_wp_idx + i) % n_wps
                        if idx not in phase_done:
                            self.bous_wp_idx = idx
                            found_next = True
                            break
                    if not found_next:
                        # All waypoints already done (can occur after reset)
                        self.bous_wp_idx = 0

                    # Exploration factor: more cautious for new phase, confident for resume
                    if self.fresh_phase:
                        self.exploration_factor = 0.50
                    else:
                        self.exploration_factor = 0.85
                    self.fresh_phase = False

                    self.wp_skipped_count = 0
                    self.state = self.mission_phase
                    self._set_leds(self.mission_phase)
                    print(f"[STATE CHANGE] UNDOCKING → {self.mission_phase} "
                          f"(t={t:.0f}s | WP#{self.bous_wp_idx}/{len(self.bous_waypoints)})")
                else:
                    turn_sp = clamp(0.04 * abs(err),
                                    0.08 * MAX_SPEED, 0.40 * MAX_SPEED)
                    if err > 0:
                        self.set_motors(turn_sp, -turn_sp)
                    else:
                        self.set_motors(-turn_sp, turn_sp)

        # ── VACUUMING ────────────────────────────────────────────────────────
        elif self.state == "VACUUMING":
            # allow_carpet=True: robot may drive over carpet (vacuuming)
            self._execute_cleaning(t, allow_carpet=True)

        # ── MOPPING ───────────────────────────────────────────────────────────
        elif self.state == "MOPPING":
            # allow_carpet=False: avoid carpet (mop must not touch carpet)
            self._execute_cleaning(t, allow_carpet=False)

        # ── ESCAPE ────────────────────────────────────────────────────────────
        elif self.state == "ESCAPE":
            if t < self.escape_reverse_end:
                self.set_motors(-0.50 * MAX_SPEED, -0.50 * MAX_SPEED)
            elif t < self.escape_end_time:
                td = self.turn_direction
                self.set_motors(td * 0.45 * MAX_SPEED, -td * 0.45 * MAX_SPEED)
            else:
                self.cleaning_speed   = 0.0
                self.wp_progress_time = 0.0
                self.wp_prev_dist     = float('inf')
                self.state = self.prev_state
                self._set_leds(self.prev_state)

        # ── RETURNING ─────────────────────────────────────────────────────────
        elif self.state == "RETURNING":
            if self.returning_start_time == 0.0:
                self.returning_start_time = t
                if self.pos[1] < 0.30:
                    via_x = PREDOCK_X - 1.50                      # 0.50m west van predock
                    via_y = max(0.30, PREDOCK_Y - 1.30)           # 1.30m south van predock
                    self.returning_via = (via_x, via_y)
                else:
                    self.returning_via = None

            # Determine current navigation target (via-point or predock)
            if self.returning_via is not None:
                tx, ty = self.returning_via
                if math.hypot(tx - self.pos[0], ty - self.pos[1]) < 0.35:
                    self.returning_via = None
                    tx, ty = PREDOCK_X, PREDOCK_Y
            else:
                tx, ty = PREDOCK_X, PREDOCK_Y

            dx = tx - self.pos[0]
            dy = ty - self.pos[1]

            # Tolerance check: only for final predock destination
            if self.returning_via is None:
                ret_elapsed = t - self.returning_start_time
                if ret_elapsed > 120.0:
                    tol_x, tol_y = 0.15, 0.40   # emergency mode
                elif ret_elapsed > 60.0:
                    tol_x, tol_y = 0.09, 0.32   # relaxed
                else:
                    tol_x, tol_y = 0.06, 0.25   # nominal (strict for arm clearance)

                dx_abs = abs(PREDOCK_X - self.pos[0])
                dy_abs = abs(PREDOCK_Y - self.pos[1])
                if dx_abs < tol_x and dy_abs < tol_y:
                    print(f"[STATE CHANGE] RETURNING → ALIGNING "
                          f"(X={self.pos[0]:.3f}, Y={self.pos[1]:.3f}, "
                          f"ΔX={dx_abs:.3f}m, t_ret={ret_elapsed:.0f}s)")
                    self.aligning_start_time  = t
                    self.returning_start_time = 0.0
                    self.returning_via        = None
                    self.pid_bearing.reset(new_SP=0.0)
                    self.state = "ALIGNING"
                    self._set_leds("ALIGNING")
                    return

            target_deg = math.degrees(math.atan2(dx, dy))
            if target_deg < 0.0:
                target_deg += 360.0
            err  = angle_error(target_deg, self.bearing)
            turn = clamp(0.045 * err, -0.40 * MAX_SPEED, 0.40 * MAX_SPEED)

            obs_front       = self.d_front       < OBSTACLE_DIST
            obs_front_left  = self.d_front_left  < OBSTACLE_DIST
            obs_front_right = self.d_front_right < OBSTACLE_DIST
            obs_far_left    = self.d_far_left    < OBSTACLE_DIST
            obs_far_right   = self.d_far_right   < OBSTACLE_DIST
            base = 0.40 * MAX_SPEED

            if obs_front or (obs_front_left and obs_front_right):
                if self.d_far_left >= self.d_far_right:
                    self.set_motors(-0.20 * MAX_SPEED, 0.42 * MAX_SPEED)
                else:
                    self.set_motors(0.42 * MAX_SPEED, -0.20 * MAX_SPEED)
            elif obs_front_left:
                self.set_motors(0.45 * MAX_SPEED, 0.10 * MAX_SPEED)
            elif obs_front_right:
                self.set_motors(0.10 * MAX_SPEED, 0.45 * MAX_SPEED)
            elif obs_far_left and not obs_far_right:
                self.set_motors(base + turn + 0.08 * MAX_SPEED,
                                base - turn - 0.08 * MAX_SPEED)
            elif obs_far_right and not obs_far_left:
                self.set_motors(base + turn - 0.08 * MAX_SPEED,
                                base - turn + 0.08 * MAX_SPEED)
            else:
                self.set_motors(base + turn, base - turn)

        # ── ALIGNING ──────────────────────────────────────────────────────────
        elif self.state == "ALIGNING":
            err       = angle_error(0.0, self.bearing)
            aligned   = abs(err) < ALIGNING_THRESHOLD
            timed_out = (t - self.aligning_start_time) > ALIGNING_TIMEOUT

            if aligned or timed_out:
                if timed_out and not aligned:
                    print(f"[ALIGNING] Timeout → DOCKING (err={err:.1f}°)")
                else:
                    print(f"[STATE CHANGE] ALIGNING → DOCKING (err={err:.1f}°)")
                self.aligning_start_time = 0.0
                self.docking_start_time  = t
                self.dock_last_y         = 0.0
                self.dock_stall_timer    = t
                self.state = "DOCKING"
                self._set_leds("DOCKING")
            else:
                # PID heading control (per course material).
                # iPV = -err so ER = SP - (-err) = err → positive output when err > 0.
                lmn     = self.pid_bearing.compute(-err, self.dt)
                turn_sp = clamp(abs(lmn), 0.02 * MAX_SPEED, 0.45 * MAX_SPEED)
                if err > 0:
                    self.set_motors(turn_sp, -turn_sp)
                else:
                    self.set_motors(-turn_sp, turn_sp)

        # ── DOCKING ───────────────────────────────────────────────────────────
        elif self.state == "DOCKING":
            if self.dock_last_y == 0.0:
                self.dock_last_y      = self.pos[1]
                self.dock_stall_timer = t
            if self.pos[1] > self.dock_last_y + 0.02:
                self.dock_last_y      = self.pos[1]
                self.dock_stall_timer = t

            bumper_in_dock = self.bumper_hit > 0.0 and self.pos[1] > DOCK_ARM_Y
            gps_in_dock    = self.pos[1] > DOCK_CONFIRM_Y
            stalled        = (t - self.dock_stall_timer) > 8.0

            def _enter_charging():
                trigger = "bumper" if (self.bumper_hit > 0 and self.pos[1] > DOCK_ARM_Y) \
                          else f"GPS/stall Y={self.pos[1]:.2f}"
                print(f"[STATE CHANGE] DOCKING → CHARGING "
                      f"({trigger}, distance={abs(CHARGER_Y - self.pos[1]):.2f}m)")
                self.last_charge_time    = self.robot.getTime()
                self.charging_start_time = 0.0
                self.docking_start_time  = 0.0
                self.dock_last_y         = 0.0
                self.state = "CHARGING"
                self._set_leds("CHARGING")
                self.set_motors(0.0, 0.0)

            if bumper_in_dock or gps_in_dock:
                _enter_charging()
            elif stalled:
                if self.pos[1] > DOCK_STALL_RADIUS_Y:
                    _enter_charging()
                else:
                    print(f"[DOCKING] Stall outside radius (Y={self.pos[1]:.2f}) → ALIGNING")
                    self.docking_start_time  = 0.0
                    self.dock_last_y         = 0.0
                    self.aligning_start_time = t
                    self.state = "ALIGNING"
                    self._set_leds("ALIGNING")
                    self.set_motors(0.0, 0.0)
            elif (t - self.docking_start_time) > DOCKING_TIMEOUT:
                print(f"[DOCKING] Timeout → ALIGNING (Y={self.pos[1]:.2f})")
                self.docking_start_time  = 0.0
                self.dock_last_y         = 0.0
                self.aligning_start_time = t
                self.state = "ALIGNING"
                self._set_leds("ALIGNING")
                self.set_motors(0.0, 0.0)
            else:
                # x_err > 0 → robot left of charger → steer right (CW: +L,-R)
                # x_err < 0 → robot right of charger → steer left (CCW: -L,+R)
                x_err       = CHARGER_X - self.pos[0]
                bearing_err = angle_error(0.0, self.bearing)
                x_turn      = clamp(4.0 * x_err,
                                    -0.20 * MAX_SPEED, 0.20 * MAX_SPEED)
                b_turn      = clamp(0.02 * bearing_err,
                                    -0.08 * MAX_SPEED, 0.08 * MAX_SPEED)
                turn        = clamp(x_turn + b_turn,
                                    -0.20 * MAX_SPEED, 0.20 * MAX_SPEED)
                self.set_motors(0.12 * MAX_SPEED + turn, 0.12 * MAX_SPEED - turn)

        # ── CHARGING ──────────────────────────────────────────────────────────
        elif self.state == "CHARGING":
            if self.charging_start_time == 0.0:
                self.charging_start_time = t
                pct = int((self.battery / BATTERY_MAX) * 100)

            if self.pos[1] < DOCK_STALL_RADIUS_Y:
                print(f"[CHARGING] Outside charging zone (Y={self.pos[1]:.2f}) → DOCKING")
                self.docking_start_time  = t
                self.dock_last_y         = 0.0
                self.dock_stall_timer    = t
                self.charging_start_time = 0.0
                self.state = "DOCKING"
                self._set_leds("DOCKING")
                return

            self.set_motors(0.0, 0.0)
            pct = int((self.battery / BATTERY_MAX) * 100) if self.battery >= 0 else 0

            def _start_next_phase():
                """Start UNDOCKING or go to FINISHED if both phases are complete."""
                n_total   = len(self.bous_waypoints)
                phase_key = self.mission_phase
                n_done    = len(self.wp_done[phase_key])
                t_now     = self.robot.getTime()
                dur       = t_now - self.last_charge_time if self.last_charge_time else 0
                print(f"[BATTERY] Charged: {pct}%")
                self.last_logged_pct = pct

                if n_done >= n_total:
                    # Current phase fully completed
                    self.phases_completed += 1
                    old_phase = self.mission_phase

                    if self.phases_completed >= 2:
                        # Both phases (VACUUMING + MOPPING) fully cleaned → FINISHED
                        print(f"[STATE CHANGE] CHARGING → FINISHED")
                        print(f"[MISSION] Complete! VACUUMING + MOPPING done "
                              f"| t={t_now:.0f}s | battery: {pct}%")
                        self.last_charge_time    = t_now
                        self.charging_start_time = 0.0
                        self.state = "FINISHED"
                        self._set_leds("FINISHED")
                        self.set_motors(0.0, 0.0)
                        return

                    # First phase done → switch to the other phase
                    self.mission_phase = ("MOPPING" if old_phase == "VACUUMING"
                                          else "VACUUMING")
                    self.wp_done[self.mission_phase].clear()   # clean start new phase
                    self.fresh_phase = True
                    self.bous_wp_idx = 0

                    # Generate phase-specific waypoints on phase switch.
                    if self.mission_phase == "MOPPING":
                        self.bous_waypoints = self._generate_mopping_waypoints()
                    else:
                        self.bous_waypoints = self._generate_waypoints()

                    print(f"[STATE CHANGE] CHARGING → UNDOCKING "
                          f"| {old_phase} done → next: {self.mission_phase} "
                          f"| battery: {pct}% | [{self.phases_completed}/2 phases]")
                else:
                    # Phase not yet done → resume same phase (waypoints unchanged)
                    remaining = n_total - n_done
                    self.fresh_phase = False
                    print(f"[STATE CHANGE] CHARGING → UNDOCKING "
                          f"| {phase_key}: {n_done}/{n_total} WP done, "
                          f"{remaining} remaining | battery: {pct}%")

                self.last_charge_time    = t_now
                self.charging_start_time = 0.0
                self.state               = "UNDOCKING"
                self.undock_phase        = "REVERSE"
                self.undock_timer        = 0.0
                self.in_carpet_escape    = False
                self._set_leds("UNDOCKING")

            if pct >= 95:
                _start_next_phase()
            elif (t - self.charging_start_time) > CHARGING_MAX_TIME:
                print(f"[CHARGING] Timeout {CHARGING_MAX_TIME:.0f}s (battery: {pct}%)")
                _start_next_phase()

        # ── FINISHED ─────────────────────────────────────────────────────────
        elif self.state == "FINISHED":
            # Mission fully completed: VACUUMING + MOPPING both done.
            self.set_motors(0.0, 0.0)

    # ── main loop ─────────────────────────────────────────────────────────────
    def run_step(self):
        t = self.robot.getTime()
        self._watchdog(t)
        self._read_sensors()
        self._check_emergencies(t)
        self._check_stuck(t)
        self._execute_state_machine(t)


# ──────────────────────────────────────────────────────────────────────────────
# WEBOTS ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────
robot    = Robot()
timestep = int(robot.getBasicTimeStep())
roomba   = RoombaController(robot, timestep)

while robot.step(timestep) != -1:
    roomba.run_step()
