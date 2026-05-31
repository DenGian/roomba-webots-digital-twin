from controller import Robot
import math
import random

# ──────────────────────────────────────────────────────────────────────────────
# PID CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
class PID_Controller:
    """Standard PID controller. Ki=0 means P+D only."""
    def __init__(self, iKp, iKi, iKd, iSP, iLMN_HLM, iLMN_LLM):
        self.Kp = iKp
        self.Ki = iKi
        self.Kd = iKd
        self.SP = iSP
        self.prev_ER = 0.0
        self.integral = 0.0
        self.LMN_HLM = iLMN_HLM
        self.LMN_LLM = iLMN_LLM

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
        self.prev_ER = 0.0
        if new_SP is not None:
            self.SP = new_SP

# ──────────────────────────────────────────────────────────────────────────────
# HELPER FUNCTIONS
# ──────────────────────────────────────────────────────────────────────────────
def clamp(value, lo, hi):
    return max(lo, min(value, hi))

def angle_error(target, current):
    """Shortest signed angle from current to target in [-180, 180]."""
    err = target - current
    while err >  180.0: err -= 360.0
    while err < -180.0: err += 360.0
    return err

# ──────────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ──────────────────────────────────────────────────────────────────────────────
MAX_SPEED = 6.28
BATTERY_MAX = 10_000.0
BATTERY_LOW_PCT = 0.20  # return to charger below 20%
ROAMING_DURATION = 1200.0  # max 20 minutes per cleaning phase

# LiDAR sector index ranges (256 rays, 180° FOV)
_L0,  _L1 =   0,  64
_FL0, _FL1 =  64, 112
_F0,  _F1 = 112, 144
_FR0, _FR1 = 144, 192
_R0,  _R1 = 192, 256

OBSTACLE_DIST = 0.35
FAR_OBSTACLE_DIST = 0.28
SLOW_DIST = 0.65

# Carpet sensor thresholds
CARPET_FULL = 730
CARPET_EDGE = 670

# Known carpet area bounds (world coordinates)
CARPET_X_MIN = -2.25
CARPET_X_MAX =  0.30
CARPET_Y_MIN =  0.05
CARPET_Y_MAX =  1.95
CARPET_SAFE_X =  1.20

# Docking geometry
CHARGER_X = 2.00
CHARGER_Y = 2.85
PREDOCK_X = 2.00
PREDOCK_Y = 1.80
DOCK_ARM_Y = 2.55
DOCK_CONFIRM_Y = 2.70
DOCK_STALL_RADIUS_Y = 2.45

# Timeouts
ALIGNING_THRESHOLD = 6.0
ALIGNING_TIMEOUT = 10.0
DOCKING_TIMEOUT = 45.0
CHARGING_MAX_TIME = 90.0

# Boustrophedon (zigzag) navigation bounds and parameters
BOUS_Y_MIN = -2.10
BOUS_Y_MAX =  1.80
BOUS_X_MIN = -2.35
BOUS_X_MAX =  2.35
BOUS_STRIP_STEP =  0.35
BOUS_WP_RADIUS =  0.22

WP_FRUSTRATION_TIME = 8.0

SPEED_RAMP = 5.0
LOOK_AHEAD_DIST = 0.55

STUCK_TIMEOUT = 10.0
STUCK_DIST = 0.05

WP_NEAR_THRESH = 1.5

SCAN_MARGIN = 0.30  # safety margin relative to detected wall points (m)

CARPET_CELL_SIZE = 0.25  # grid cell size for carpet map

# ──────────────────────────────────────────────────────────────────────────────
# CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
class RoombaController:

    CLEANING_STATES = ("VACUUMING", "MOPPING")

    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        self.dt = timestep / 1000.0

        self.state = "UNDOCKING"
        self.prev_state = "VACUUMING"

        self.mission_phase = "VACUUMING"
        self.roaming_start_time = 0.0

        self.undock_phase = "REVERSE"
        self.undock_timer = 0.0

        # ESCAPE
        self.escape_reverse_end = 0.0
        self.escape_end_time = 0.0
        self.turn_direction = 1
        self.evasion_count = 0
        self.last_evasion_time = 0.0

        # Carpet escape (MOPPING only)
        self.in_carpet_escape = False
        self.carpet_escape_count = 0
        self.carpet_escape_target = (CARPET_SAFE_X, 0.0)

        # Boustrophedon navigation
        self.bous_waypoints = self._generate_waypoints()
        self.bous_wp_idx = 0

        self.wp_prev_dist = float('inf')
        self.wp_progress_time = 0.0

        # Carpet grid map, built during VACUUMING, used during MOPPING
        self.carpet_map = set()

        self.scan_wall_pts = []
        self.room_bounds_computed = False
        self.last_scan_collect_t = 0.0

        # Waypoints completed per phase — persists across charging cycles
        self.wp_done = {"VACUUMING": set(), "MOPPING": set()}

        self.phases_completed = 0
        self.wp_skipped_count = 0

        self.exploration_factor = 1.0
        self.fresh_phase = True
        self.cleaning_speed = 0.0

        self.stuck_check_pos = None
        self.stuck_check_time = 0.0

        self.returning_start_time = 0.0
        self.returning_via = None

        # PID for heading alignment during docking
        self.pid_bearing = PID_Controller(
            iKp=0.045, iKi=0.0, iKd=0.001,
            iSP=0.0,
            iLMN_HLM= 0.45 * MAX_SPEED,
            iLMN_LLM=-0.45 * MAX_SPEED
        )

        self.aligning_start_time = 0.0
        self.docking_start_time = 0.0
        self.dock_last_y = 0.0
        self.dock_stall_timer = 0.0

        self.charging_start_time = 0.0
        self.last_charge_time = None

        self.last_logged_pct = 110
        self.last_carpet_log = -99.0
        self.last_escape_log_t = -99.0
        self.escape_log_count = 0
        self.last_carpet_log_t = -99.0
        self.carpet_log_count = 0
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
        self.motor_left = self.robot.getDevice('motor_left')
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

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        self.status_led = self.robot.getDevice('status_led')
        self.battery_led = self.robot.getDevice('battery_led')

    # ── actuators ────────────────────────────────────────────────────────────
    def set_motors(self, left, right):
        self.motor_left.setVelocity(clamp(left,  -MAX_SPEED, MAX_SPEED))
        self.motor_right.setVelocity(clamp(right, -MAX_SPEED, MAX_SPEED))

    def _set_leds(self, state):
        """LED pattern per state: (status_led, battery_led)."""
        leds = {
            "VACUUMING": (1, 0),
            "MOPPING": (1, 1),
            "ESCAPE": (0, 0),
            "RETURNING": (1, 1),
            "ALIGNING": (1, 0),
            "DOCKING": (1, 0),
            "CHARGING": (0, 0),
            "UNDOCKING": (1, 0),
            "FINISHED": (1, 1),
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
        self.d_far_left = self._lidar_min(_L0,  _L1)
        self.d_front_left = self._lidar_min(_FL0, _FL1)
        self.d_front = self._lidar_min(_F0,  _F1)
        self.d_front_right = self._lidar_min(_FR0, _FR1)
        self.d_far_right = self._lidar_min(_R0,  _R1)

    # ── waypoint generation ───────────────────────────────────────────────────
    def _generate_waypoints(self):
        return self._generate_boustrophedon(
            BOUS_X_MIN, BOUS_X_MAX, BOUS_Y_MIN, BOUS_Y_MAX
        )

    def _generate_mopping_waypoints(self):
        """Waypoints for mopping: full-width strips outside carpet, east-only inside."""
        cy_min = self._min_carpet_y()
        cy_max = self._max_carpet_y()
        cx_east = self._carpet_x_east()
        buffer = 0.20

        cy_lo = cy_min - buffer
        cy_hi = cy_max + buffer

        pts = []
        y = BOUS_Y_MIN
        going_east = True

        while y <= BOUS_Y_MAX + 0.01:
            yr = round(y, 2)

            # Limit east end near charger
            if yr > CHARGER_Y - 0.80:
                x_east = min(BOUS_X_MAX, CHARGER_X - 0.50)
            else:
                x_east = BOUS_X_MAX

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

        pts = []
        y = y_min
        going_east = True
        while y <= y_max + 0.01:
            yr = round(y, 2)
            # Limit east end of strips near the charger to avoid the charger arm
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

        if len(self.scan_wall_pts) < 50:
            print(f"[PASSIVE SCAN] Too few points ({len(self.scan_wall_pts)}) "
                  f"→ keeping default room dimensions")
            return

        xs = sorted(p[0] for p in self.scan_wall_pts)
        ys = sorted(p[1] for p in self.scan_wall_pts)
        n = len(xs)

        # Use 5th/95th percentile to filter wall point outliers
        p5 = max(0, n // 20)
        p95 = min(n - 1, n - n // 20)

        raw_x_min = xs[p5]
        raw_x_max = xs[p95]
        raw_y_min = ys[p5]
        raw_y_max = ys[p95]

        new_x_min = raw_x_min + SCAN_MARGIN
        new_x_max = raw_x_max - SCAN_MARGIN
        new_y_min = raw_y_min + SCAN_MARGIN
        new_y_max = raw_y_max - SCAN_MARGIN

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
        self.battery = self.robot.batterySensorGetValue()
        self.wall_val = self.wall_sensor.getValue()
        self.carpet_val = self.carpet_sensor.getValue()
        self.ranges = self.lidar.getRangeImage()
        self.bumper_hit = self.bumper.getValue()
        self.cliff_hit = any(s.getValue() < 100.0 for s in self.cliff_sensors)
        self.pos = self.gps.getValues()
        self.bearing = self.get_bearing()
        self._read_lidar_sectors()

        pct = int((self.battery / BATTERY_MAX) * 100) if self.battery >= 0 else 0
        _active = self.state in ("VACUUMING", "MOPPING", "RETURNING", "ESCAPE", "UNDOCKING")
        if (_active and pct != self.last_logged_pct
                and pct in (50, 20) and 0 <= pct <= 100):
            print(f"[BATTERY] {pct}%")
            self.last_logged_pct = pct
        if pct <= 20 and self.state not in ("CHARGING", "DOCKING"):
            self.battery_led.set(1)

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

        if self.state not in (*self.CLEANING_STATES, "RETURNING"):
            return

        # Skip escape when battery is critically low during return
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
        time_up = (t - self.roaming_start_time) >= ROAMING_DURATION
        self.prev_state = "RETURNING" if (battery_low or time_up) else self.state

        self.in_carpet_escape = False

        self.escape_log_count += 1
        if t - self.last_escape_log_t > 30.0:
            count_str = f" ×{self.escape_log_count}" if self.escape_log_count > 1 else ""
            print(f"[ESCAPE] {trigger}{count_str} in {self.state}")
            self.last_escape_log_t = t
            self.escape_log_count = 0
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
            self.escape_end_time = self.escape_reverse_end + 2.2
            self.turn_direction = 1
            self.evasion_count = 0
        else:
            self.escape_reverse_end = t + 1.0
            self.escape_end_time = self.escape_reverse_end + random.uniform(0.8, 1.6)
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

        if not self.carpet_map:
            return CARPET_Y_MIN - buffer <= wy <= CARPET_Y_MAX + buffer

        cy_min = self._min_carpet_y()
        cy_max = self._max_carpet_y()
        return (cy_min - buffer) <= wy <= (cy_max + buffer)

    def _min_carpet_y(self):

        if not self.carpet_map:
            return CARPET_Y_MIN
        return min(cy * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def _max_carpet_y(self):

        if not self.carpet_map:
            return CARPET_Y_MAX
        return max(cy * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def _carpet_x_east(self):
        """
        Safe start-X for east-only MOPPING strips.
        """
        if not self.carpet_map:
            return CARPET_X_MAX + 0.30
        max_cx = max(cx for (cx, _) in self.carpet_map)
        return (max_cx + 1) * CARPET_CELL_SIZE + 0.20

    def _handle_carpet_escape(self, t):

        carpet_y_min = self._min_carpet_y()

        gps_on = self._on_carpet_gps()
        gps_near = (-2.55 <= self.pos[0] <= 0.55 and
                    CARPET_Y_MIN - 0.20 <= self.pos[1] <= CARPET_Y_MAX + 0.25)
        trigger = (self.carpet_val > CARPET_FULL or
                    (self.carpet_val > CARPET_EDGE and gps_near))

        if not self.in_carpet_escape and (trigger or gps_on):
            self.in_carpet_escape = True
            self.carpet_escape_count += 1
            target_y = carpet_y_min - 0.50
            self.carpet_escape_target = (self.pos[0], target_y)
            self.last_carpet_log = t  # carpet avoidance handled silently

        if not self.in_carpet_escape:
            return False

        sensor_clear = self.carpet_val < (CARPET_EDGE - 30)
        if sensor_clear and not gps_on:
            self.in_carpet_escape = False
            return False

        tx, ty = self.carpet_escape_target
        dx = tx - self.pos[0]
        dy = ty - self.pos[1]
        tb = math.degrees(math.atan2(dx, dy))
        if tb < 0.0:
            tb += 360.0
        err = angle_error(tb, self.bearing)
        turn = clamp(0.05 * err, -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)
        self.set_motors(0.45 * MAX_SPEED + turn, 0.45 * MAX_SPEED - turn)
        return True

    # ── shared cleaning logic ─────────────────────────────────────────────────
    def _execute_cleaning(self, t, allow_carpet):

        # Return early if all waypoints are done
        phase_key = "VACUUMING" if allow_carpet else "MOPPING"
        if len(self.wp_done[phase_key]) >= len(self.bous_waypoints):
            _skip_str = f" | {self.wp_skipped_count} WPs skipped" if self.wp_skipped_count else ""
            print(f"[MISSION] {phase_key} complete "
                  f"({len(self.wp_done[phase_key])}/{len(self.bous_waypoints)} WP{_skip_str})")
            self.pid_bearing.reset()
            self.in_carpet_escape = False
            self.returning_start_time = 0.0
            self.cleaning_speed = 0.0
            self.state = "RETURNING"
            self._set_leds("RETURNING")
            return

        # Return to charger if battery is low or time limit reached
        battery_low = self.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up = (t - self.roaming_start_time) >= ROAMING_DURATION

        if battery_low or time_up:
            reason = "time limit 20 min" if time_up else f"low battery ({BATTERY_LOW_PCT*100:.0f}%)"
            print(f"[STATE CHANGE] {self.state} → RETURNING ({reason})")
            self.pid_bearing.reset()
            self.in_carpet_escape = False
            self.returning_start_time = 0.0
            self.cleaning_speed = 0.0
            self.state = "RETURNING"
            self._set_leds("RETURNING")
            return

        if not allow_carpet:
            if self._handle_carpet_escape(t):
                return

        # LiDAR obstacle avoidance
        obs_front = self.d_front < OBSTACLE_DIST
        obs_front_left = self.d_front_left < OBSTACLE_DIST
        obs_front_right = self.d_front_right < OBSTACLE_DIST
        obs_far_left = self.d_far_left < FAR_OBSTACLE_DIST
        obs_far_right = self.d_far_right < FAR_OBSTACLE_DIST
        slow_front = self.d_front < SLOW_DIST

        any_obstacle = (obs_front or obs_front_left or obs_front_right
                        or obs_far_left or obs_far_right)

        if any_obstacle:
            self.cleaning_speed = max(0.0,
                                      self.cleaning_speed - 0.15 * MAX_SPEED)

        if obs_front:
            if self.d_far_left >= self.d_far_right:
                self.set_motors(-0.20 * MAX_SPEED, 0.55 * MAX_SPEED)
            else:
                self.set_motors(0.55 * MAX_SPEED, -0.20 * MAX_SPEED)

        elif obs_front_left and obs_front_right:
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
            # Boustrophedon GPS navigation
            wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
            dx_wp = wp_x - self.pos[0]
            dy_wp = wp_y - self.pos[1]
            dist_wp = math.hypot(dx_wp, dy_wp)

            if dist_wp < WP_NEAR_THRESH:
                if self.wp_progress_time == 0.0:
                    self.wp_progress_time = t
                    self.wp_prev_dist = dist_wp

                if dist_wp < self.wp_prev_dist - 0.04:
                    self.wp_progress_time = t
                    self.wp_prev_dist = dist_wp
                elif (t - self.wp_progress_time) > WP_FRUSTRATION_TIME:
                    old_wp = self.bous_wp_idx
                    phase_key = "VACUUMING" if allow_carpet else "MOPPING"
                    self.wp_done[phase_key].add(old_wp)
                    self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                    self.wp_skipped_count += 1
                    self.wp_progress_time = t
                    self.wp_prev_dist = float('inf')
                    wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                    dx_wp = wp_x - self.pos[0]
                    dy_wp = wp_y - self.pos[1]
                    dist_wp = math.hypot(dx_wp, dy_wp)
            else:
                # Far from waypoint: reset frustration timer and navigate normally
                self.wp_progress_time = 0.0
                self.wp_prev_dist = dist_wp

            if dist_wp < BOUS_WP_RADIUS:
                phase_key = "VACUUMING" if allow_carpet else "MOPPING"
                self.wp_done[phase_key].add(self.bous_wp_idx)
                self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                self.wp_progress_time = t
                self.wp_prev_dist = float('inf')

                if self.bous_wp_idx % 2 == 0 and self.bous_wp_idx != 0:
                    next_wp = self.bous_waypoints[self.bous_wp_idx]
                    strip_n = self.bous_wp_idx // 2
                    total_s = len(self.bous_waypoints) // 2
                    n_done = len(self.wp_done[phase_key])
                    skipped_str = f" | {self.wp_skipped_count} skipped" if self.wp_skipped_count else ""
                    print(f"[NAV] Strip {strip_n}/{total_s} → Y={next_wp[1]:.2f} "
                          f"[{n_done}/{len(self.bous_waypoints)} WP{skipped_str}]")
                wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                dx_wp = wp_x - self.pos[0]
                dy_wp = wp_y - self.pos[1]
                dist_wp = math.hypot(dx_wp, dy_wp)

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

                blend = clamp(1.0 - dist_wp / LOOK_AHEAD_DIST, 0.0, 0.50)
                diff = angle_error(next_deg, target_deg)
                target_deg = (target_deg + blend * diff) % 360.0

            bearing_err = angle_error(target_deg, self.bearing)
            turn = clamp(0.040 * bearing_err, -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)

            # Wall sensor correction
            wall_corr = 0.0
            if self.wall_val > 80.0:
                wall_err = 380.0 - self.wall_val
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
            ramp_delta = SPEED_RAMP * self.dt
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
                            offset_deg = -90.0 + (180.0 / max(n_rays - 1, 1)) * i
                            wb_rad = bearing_rad + math.radians(offset_deg)
                            self.scan_wall_pts.append((
                                sensor_x + r * math.sin(wb_rad),
                                sensor_y + r * math.cos(wb_rad)
                            ))

    # ── stuck detection ───────────────────────────────────────────────────────
    def _check_stuck(self, t):

        if self.state not in self.CLEANING_STATES:

            self.stuck_check_pos = None
            self.stuck_check_time = 0.0
            return

        if self.stuck_check_pos is None:
            self.stuck_check_pos = (self.pos[0], self.pos[1])
            self.stuck_check_time = t
            return

        moved = math.hypot(self.pos[0] - self.stuck_check_pos[0],
                           self.pos[1] - self.stuck_check_pos[1])
        if moved > STUCK_DIST:

            self.stuck_check_pos = (self.pos[0], self.pos[1])
            self.stuck_check_time = t
        elif t - self.stuck_check_time > STUCK_TIMEOUT:
            print(f"[STUCK] No {STUCK_DIST}m movement in {STUCK_TIMEOUT:.0f}s "
                  f"(pos={self.pos[0]:.2f},{self.pos[1]:.2f}) → ESCAPE spin")
            self.prev_state = self.state
            self.state = "ESCAPE"
            self.escape_reverse_end = t + 0.3
            self.escape_end_time = t + 0.3 + 1.5
            self.turn_direction = 1 if random.random() > 0.5 else -1
            self.cleaning_speed = 0.0
            self.wp_progress_time = 0.0
            self.wp_prev_dist = float('inf')
            self.stuck_check_pos = None
            self.stuck_check_time = 0.0
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

                    self.roaming_start_time = t
                    self.returning_start_time = 0.0
                    self.returning_via = None
                    self.in_carpet_escape = False
                    self.cleaning_speed = 0.0
                    self.wp_progress_time = 0.0
                    self.wp_prev_dist = float('inf')

                    # Resume from first unvisited waypoint
                    phase_done = self.wp_done[self.mission_phase]
                    n_wps = len(self.bous_waypoints)
                    found_next = False
                    for i in range(n_wps):
                        idx = (self.bous_wp_idx + i) % n_wps
                        if idx not in phase_done:
                            self.bous_wp_idx = idx
                            found_next = True
                            break
                    if not found_next:
                        self.bous_wp_idx = 0

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

            self._execute_cleaning(t, allow_carpet=True)

        # ── MOPPING ───────────────────────────────────────────────────────────
        elif self.state == "MOPPING":

            self._execute_cleaning(t, allow_carpet=False)

        # ── ESCAPE ────────────────────────────────────────────────────────────
        elif self.state == "ESCAPE":
            if t < self.escape_reverse_end:
                self.set_motors(-0.50 * MAX_SPEED, -0.50 * MAX_SPEED)
            elif t < self.escape_end_time:
                td = self.turn_direction
                self.set_motors(td * 0.45 * MAX_SPEED, -td * 0.45 * MAX_SPEED)
            else:
                self.cleaning_speed = 0.0
                self.wp_progress_time = 0.0
                self.wp_prev_dist = float('inf')
                self.state = self.prev_state
                self._set_leds(self.prev_state)

        # ── RETURNING ─────────────────────────────────────────────────────────
        elif self.state == "RETURNING":
            if self.returning_start_time == 0.0:
                self.returning_start_time = t
                if self.pos[1] < 0.30:
                    via_x = PREDOCK_X - 1.50
                    via_y = max(0.30, PREDOCK_Y - 1.30)
                    self.returning_via = (via_x, via_y)
                else:
                    self.returning_via = None

            if self.returning_via is not None:
                tx, ty = self.returning_via
                if math.hypot(tx - self.pos[0], ty - self.pos[1]) < 0.35:
                    self.returning_via = None
                    tx, ty = PREDOCK_X, PREDOCK_Y
            else:
                tx, ty = PREDOCK_X, PREDOCK_Y

            dx = tx - self.pos[0]
            dy = ty - self.pos[1]

            if self.returning_via is None:
                ret_elapsed = t - self.returning_start_time
                if ret_elapsed > 120.0:
                    tol_x, tol_y = 0.15, 0.40
                elif ret_elapsed > 60.0:
                    tol_x, tol_y = 0.09, 0.32
                else:
                    tol_x, tol_y = 0.06, 0.25

                dx_abs = abs(PREDOCK_X - self.pos[0])
                dy_abs = abs(PREDOCK_Y - self.pos[1])
                if dx_abs < tol_x and dy_abs < tol_y:
                    print(f"[STATE CHANGE] RETURNING → ALIGNING "
                          f"(X={self.pos[0]:.3f}, Y={self.pos[1]:.3f}, "
                          f"ΔX={dx_abs:.3f}m, t_ret={ret_elapsed:.0f}s)")
                    self.aligning_start_time = t
                    self.returning_start_time = 0.0
                    self.returning_via = None
                    self.pid_bearing.reset(new_SP=0.0)
                    self.state = "ALIGNING"
                    self._set_leds("ALIGNING")
                    return

            target_deg = math.degrees(math.atan2(dx, dy))
            if target_deg < 0.0:
                target_deg += 360.0
            err = angle_error(target_deg, self.bearing)
            turn = clamp(0.045 * err, -0.40 * MAX_SPEED, 0.40 * MAX_SPEED)

            obs_front = self.d_front       < OBSTACLE_DIST
            obs_front_left = self.d_front_left  < OBSTACLE_DIST
            obs_front_right = self.d_front_right < OBSTACLE_DIST
            obs_far_left = self.d_far_left    < OBSTACLE_DIST
            obs_far_right = self.d_far_right   < OBSTACLE_DIST
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
            err = angle_error(0.0, self.bearing)
            aligned = abs(err) < ALIGNING_THRESHOLD
            timed_out = (t - self.aligning_start_time) > ALIGNING_TIMEOUT

            if aligned or timed_out:
                if timed_out and not aligned:
                    print(f"[ALIGNING] Timeout → DOCKING (err={err:.1f}°)")
                else:
                    print(f"[STATE CHANGE] ALIGNING → DOCKING (err={err:.1f}°)")
                self.aligning_start_time = 0.0
                self.docking_start_time = t
                self.dock_last_y = 0.0
                self.dock_stall_timer = t
                self.state = "DOCKING"
                self._set_leds("DOCKING")
            else:
                # PID: pass -err so that positive error → positive output → CW turn
                lmn = self.pid_bearing.compute(-err, self.dt)
                turn_sp = clamp(abs(lmn), 0.02 * MAX_SPEED, 0.45 * MAX_SPEED)
                if err > 0:
                    self.set_motors(turn_sp, -turn_sp)
                else:
                    self.set_motors(-turn_sp, turn_sp)

        # ── DOCKING ───────────────────────────────────────────────────────────
        elif self.state == "DOCKING":
            if self.dock_last_y == 0.0:
                self.dock_last_y = self.pos[1]
                self.dock_stall_timer = t
            if self.pos[1] > self.dock_last_y + 0.02:
                self.dock_last_y = self.pos[1]
                self.dock_stall_timer = t

            bumper_in_dock = self.bumper_hit > 0.0 and self.pos[1] > DOCK_ARM_Y
            gps_in_dock = self.pos[1] > DOCK_CONFIRM_Y
            stalled = (t - self.dock_stall_timer) > 8.0

            def _enter_charging():
                trigger = "bumper" if (self.bumper_hit > 0 and self.pos[1] > DOCK_ARM_Y) \
                          else f"GPS/stall Y={self.pos[1]:.2f}"
                print(f"[STATE CHANGE] DOCKING → CHARGING "
                      f"({trigger}, distance={abs(CHARGER_Y - self.pos[1]):.2f}m)")
                self.last_charge_time = self.robot.getTime()
                self.charging_start_time = 0.0
                self.docking_start_time = 0.0
                self.dock_last_y = 0.0
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
                    self.docking_start_time = 0.0
                    self.dock_last_y = 0.0
                    self.aligning_start_time = t
                    self.state = "ALIGNING"
                    self._set_leds("ALIGNING")
                    self.set_motors(0.0, 0.0)
            elif (t - self.docking_start_time) > DOCKING_TIMEOUT:
                print(f"[DOCKING] Timeout → ALIGNING (Y={self.pos[1]:.2f})")
                self.docking_start_time = 0.0
                self.dock_last_y = 0.0
                self.aligning_start_time = t
                self.state = "ALIGNING"
                self._set_leds("ALIGNING")
                self.set_motors(0.0, 0.0)
            else:

                x_err = CHARGER_X - self.pos[0]
                bearing_err = angle_error(0.0, self.bearing)
                x_turn = clamp(4.0 * x_err,
                                    -0.20 * MAX_SPEED, 0.20 * MAX_SPEED)
                b_turn = clamp(0.02 * bearing_err,
                                    -0.08 * MAX_SPEED, 0.08 * MAX_SPEED)
                turn = clamp(x_turn + b_turn,
                                    -0.20 * MAX_SPEED, 0.20 * MAX_SPEED)
                self.set_motors(0.12 * MAX_SPEED + turn, 0.12 * MAX_SPEED - turn)

        # ── CHARGING ──────────────────────────────────────────────────────────
        elif self.state == "CHARGING":
            if self.charging_start_time == 0.0:
                self.charging_start_time = t
                pct = int((self.battery / BATTERY_MAX) * 100)

            if self.pos[1] < DOCK_STALL_RADIUS_Y:
                print(f"[CHARGING] Outside charging zone (Y={self.pos[1]:.2f}) → DOCKING")
                self.docking_start_time = t
                self.dock_last_y = 0.0
                self.dock_stall_timer = t
                self.charging_start_time = 0.0
                self.state = "DOCKING"
                self._set_leds("DOCKING")
                return

            self.set_motors(0.0, 0.0)
            pct = int((self.battery / BATTERY_MAX) * 100) if self.battery >= 0 else 0

            def _start_next_phase():
                n_total = len(self.bous_waypoints)
                phase_key = self.mission_phase
                n_done = len(self.wp_done[phase_key])
                t_now = self.robot.getTime()
                dur = t_now - self.last_charge_time if self.last_charge_time else 0
                print(f"[BATTERY] Charged: {pct}%")
                self.last_logged_pct = pct

                if n_done >= n_total:
                    self.phases_completed += 1
                    old_phase = self.mission_phase

                    if self.phases_completed >= 2:
                        print(f"[STATE CHANGE] CHARGING → FINISHED")
                        print(f"[MISSION] Complete! VACUUMING + MOPPING done "
                              f"| t={t_now:.0f}s | battery: {pct}%")
                        self.last_charge_time = t_now
                        self.charging_start_time = 0.0
                        self.state = "FINISHED"
                        self._set_leds("FINISHED")
                        self.set_motors(0.0, 0.0)
                        return

                    self.mission_phase = ("MOPPING" if old_phase == "VACUUMING"
                                          else "VACUUMING")
                    self.wp_done[self.mission_phase].clear()
                    self.fresh_phase = True
                    self.bous_wp_idx = 0

                    if self.mission_phase == "MOPPING":
                        self.bous_waypoints = self._generate_mopping_waypoints()
                    else:
                        self.bous_waypoints = self._generate_waypoints()

                    print(f"[STATE CHANGE] CHARGING → UNDOCKING "
                          f"| {old_phase} done → next: {self.mission_phase} "
                          f"| battery: {pct}% | [{self.phases_completed}/2 phases]")
                else:
                    remaining = n_total - n_done
                    self.fresh_phase = False
                    print(f"[STATE CHANGE] CHARGING → UNDOCKING "
                          f"| {phase_key}: {n_done}/{n_total} WP done, "
                          f"{remaining} remaining | battery: {pct}%")

                self.last_charge_time = t_now
                self.charging_start_time = 0.0
                self.state = "UNDOCKING"
                self.undock_phase = "REVERSE"
                self.undock_timer = 0.0
                self.in_carpet_escape = False
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
robot = Robot()
timestep = int(robot.getBasicTimeStep())
roomba = RoombaController(robot, timestep)

while robot.step(timestep) != -1:
    roomba.run_step()
