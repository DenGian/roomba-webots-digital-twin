from controller import Robot
import math
import random


# ──────────────────────────────────────────────────────────────────────────────
# PID-REGELAAR  (conform design specification "Theorie regelaars" — ongewijzigd)
# ──────────────────────────────────────────────────────────────────────────────
class PID_Controller:
    """
    PID controller conform de design project.
      Kp  = proportionele versterkingsfactor
      Ki  = integrerende versterkingsfactor (0.0 = uitgeschakeld)
      Kd  = differentiërende versterkingsfactor (0.0 = uitgeschakeld)
      SP  = setpoint (gewenste waarde)
      LMN_HLM / LMN_LLM = hoge / lage limiet van de regeluitgang
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
# HULPFUNCTIES
# ──────────────────────────────────────────────────────────────────────────────
def clamp(value, lo, hi):
    return max(lo, min(value, hi))

def angle_error(target, current):
    """
    Kortste hoekafstand [°] in [-180, 180].
    Positief = target is CW van current.
        err > 0 → CW  → set_motors(+, -)
        err < 0 → CCW → set_motors(-, +)
    """
    err = target - current
    while err >  180.0: err -= 360.0
    while err < -180.0: err += 360.0
    return err


# ──────────────────────────────────────────────────────────────────────────────
# CONSTANTEN
# ──────────────────────────────────────────────────────────────────────────────
MAX_SPEED        = 6.28
BATTERY_MAX      = 10_000.0
BATTERY_LOW_PCT  = 0.20          # terugkeerdrempel: 20% resterend
ROAMING_DURATION = 1200.0         # 20 minuten per reinigingsfase (veiligheidsback-up)

# Lidar sectoren (256 rays, 180° FOV; index 0=LINKS, 128=VOOR, 255=RECHTS)
_L0,  _L1  =   0,  64
_FL0, _FL1 =  64, 112
_F0,  _F1  = 112, 144
_FR0, _FR1 = 144, 192
_R0,  _R1  = 192, 256

OBSTACLE_DIST     = 0.35   # stop/draai drempel voor VOOR-sectoren
FAR_OBSTACLE_DIST = 0.28   # FIX-4A: zachtere zij-reactie (was 0.35 → grove bochten)
SLOW_DIST         = 0.65

# Tapijt drempelwaarden (lookupTable: 0→1000, 0.1→0; vloer-baseline ≈ 650)
CARPET_FULL   = 730
CARPET_EDGE   = 670

# Tapijt GPS-grenzen (rug: translation -1 1, size 2.5×1.8)
# Gebruikt voor sensor-validatie van tapijt-detectie (FIX-2) en DWEILEN-skip
CARPET_X_MIN  = -2.25
CARPET_X_MAX  =  0.30
CARPET_Y_MIN  =  0.05
CARPET_Y_MAX  =  1.95
CARPET_SAFE_X =  1.20   # veilig doel-X (oost van tapijt) bij tapijt-ontsnapping

# Docking geometrie
CHARGER_X           = 2.00
CHARGER_Y           = 2.85
PREDOCK_X           = 2.00
PREDOCK_Y           = 1.80
DOCK_ARM_Y          = 2.55
DOCK_CONFIRM_Y      = 2.70
DOCK_STALL_RADIUS_Y = 2.45

# Timing
ALIGNING_THRESHOLD = 6.0    # BUG-16 FIX: was 4.0° → timeout bij 4.6°
ALIGNING_TIMEOUT   = 10.0
DOCKING_TIMEOUT    = 45.0   # verlengd voor betere betrouwbaarheid (was 30.0)
CHARGING_MAX_TIME  = 90.0

# Boustrophedon navigatie (ClearView™ LiDAR stijl — "schoon in rechte banen")
# Bronvermelding: irobot.com — "navigeert met ClearView™ LiDAR, wand-tot-wand"
BOUS_Y_MIN      = -2.10   # FIX-v5.11: was -2.40 → 30cm wand-marge (minder zuidwand-hits)
BOUS_Y_MAX      =  1.80   # FIX-v5.10: was 2.20 → vermijdt area nabij lader-arm
BOUS_X_MIN      = -2.35   # FIX-v5.12: dichter bij wanden voor betere wand-tot-wand dekking
BOUS_X_MAX      =  2.35   # FIX-v5.12: dichter bij wanden voor betere wand-tot-wand dekking
BOUS_STRIP_STEP =  0.35
BOUS_WP_RADIUS  =  0.22   # FIX-v5.12: terug naar 0.22m (robot keert dichter bij wand)

# FIX-7A: Dynamische obstakeldetectie via frustration timeout
# Vervangt alle hardcoded meubelconstanten (TABLE_*, SOFA_*).
# Als robot >WP_FRUSTRATION_TIME sec geen vooruitgang maakt → waypoint overgeslagen.
WP_FRUSTRATION_TIME = 8.0   # FIX-v5.11: 5.0 → 8.0s (meer tijd voor echte obstakelomzeiling)

# FIX-7C: Vloeiender rijgedrag
SPEED_RAMP      = 5.0    # versnellingsramp: MAX_SPEED/s
LOOK_AHEAD_DIST = 0.55   # m — begin look-ahead naar volgend waypoint binnen deze afstand

# Stuck-detectie: positie-gebaseerd (geïnspireerd op encodertelling, zie classmate-analyse)
# Als robot >STUCK_TIMEOUT sec minder dan STUCK_DIST m beweegt → ESCAPE-spin
STUCK_TIMEOUT   = 10.0   # seconden zonder STUCK_DIST beweging → stuck
STUCK_DIST      = 0.05   # m minimale verwachte verplaatsing in STUCK_TIMEOUT sec

# Frustration-timer drempel: pas activeren als robot dicht bij waypoint is
# (ver weg: obstakelomzeiling is verwacht — geen vals positief frustration)
WP_NEAR_THRESH  = 1.5    # m — alleen frustration-tracking binnen deze afstand

# Passieve LiDAR-wandkartering (inkrementeel, geen toegewijde scanfase)
# De robot verzamelt LiDAR-wandpunten TIJDENS het normale rijden (STOFZUIGEN).
# Na de eerste volledige STOFZUIGEN-pas worden kamercontouren berekend en
# boustrophedon-waypoints hergegenereerd voor de volgende cyclus.
# Zo gedraagt de robot zich zoals de echte Roomba® 205 met ClearView™ LiDAR:
# de kaart wordt incrementeel opgebouwd terwijl de robot schoonmaakt. (irobot.com)
SCAN_MARGIN   = 0.30   # veiligheidsmarge t.o.v. gedetecteerde wandpunten (m)

# Persistente tapijt-gridkaart (sensor-gebaseerde herkenning, DWEILEN-vermijding)
# Cel-grootte 0.25m. Elke CARPET_FULL-treffer voegt cel(len) toe aan carpet_map.
# STOFZUIGEN bouwt de kaart; DWEILEN gebruikt hem voor WP-uitsluiting en GPS-bewaking.
CARPET_CELL_SIZE = 0.25   # m per gridcel voor tapijt-kaart


# ──────────────────────────────────────────────────────────────────────────────
# CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
class RoombaController:

    CLEANING_STATES = ("STOFZUIGEN", "DWEILEN")

    def __init__(self, robot, timestep):
        self.robot    = robot
        self.timestep = timestep
        self.dt       = timestep / 1000.0

        # State machine
        self.state      = "UNDOCKING"
        self.prev_state = "STOFZUIGEN"   # terugkeer-state na ESCAPE

        # FEAT-1: Missiecyclus
        self.mission_phase      = "STOFZUIGEN"  # start altijd met stofzuigen
        self.roaming_start_time = 0.0            # BUG-15 FIX: relatieve timer

        # UNDOCKING
        self.undock_phase = "REVERSE"
        self.undock_timer = 0.0

        # ESCAPE
        self.escape_reverse_end = 0.0
        self.escape_end_time    = 0.0
        self.turn_direction     = 1
        self.evasion_count      = 0
        self.last_evasion_time  = 0.0

        # Tapijt-ontsnapping (GPS-gestuurd, DWEILEN-only)
        self.in_carpet_escape     = False
        self.carpet_escape_count  = 0
        # Ontsnappingsrichting: (tx, ty) doel.
        # Zuid (pos[0], Y_MIN-0.60) als robot ten zuiden van tapijt-middelpunt;
        # Oost (CARPET_SAFE_X, pos[1]) als robot diep in het tapijt zit.
        self.carpet_escape_target = (CARPET_SAFE_X, 0.0)

        # Boustrophedon (ClearView™ LiDAR navigatie — rechte banen, wand-tot-wand)
        self.bous_waypoints = self._generate_waypoints()
        self.bous_wp_idx    = 0

        # FIX-7A: Waypoint frustration tracking (vervangt hardcoded meubelzones)
        self.wp_prev_dist     = float('inf')
        self.wp_progress_time = 0.0   # tijdstip van laatste betekenisvolle vooruitgang

        # Persistente tapijt-gridkaart: set van (ix, iy) cel-indices bevestigd door sensor.
        # Wordt gevuld tijdens STOFZUIGEN (sensor mag tapijt detecteren); gebruikt in
        # DWEILEN voor WP-uitsluiting en proactieve GPS-bewaking.
        self.carpet_map = set()

        # Passieve LiDAR-wandkartering: wandpunten verzameld tijdens normaal rijden.
        # Na eerste volledige STOFZUIGEN → kamercontouren herberekend voor volgende cyclus.
        self.scan_wall_pts       = []    # wandpunten (wx, wy) in wereldcoördinaten
        self.room_bounds_computed = False  # True na eerste STOFZUIGEN-cyclus
        self.last_scan_collect_t  = 0.0   # tijdstip van laatste wandpunt-collectie

        # Persistente kaartkennis: bijgehouden gereinigde/verwerkte waypoints per fase.
        # Overleeft laadcycli zodat de robot na het opladen VERDERGAAT i.p.v. opnieuw begint.
        self.wp_done = {"STOFZUIGEN": set(), "DWEILEN": set()}

        # Verkenningsfactor: voorzichtig starten bij een nieuwe fase, opbouwen naar vol vermogen.
        # 0.5 = halve snelheid (verkenning), 1.0 = volledige rijsnelheid (systematisch)
        self.exploration_factor = 1.0   # start vol bij eerste run (UNDOCKING zet dit goed)
        self.fresh_phase        = True   # True = eerste start van een nieuwe fase

        # FIX-7C: Speed ramping voor vloeiender rijgedrag
        self.cleaning_speed = 0.0     # huidige gerampte rijsnelheid (rad/s)

        # Stuck-detectie (positie-gebaseerd)
        self.stuck_check_pos  = None
        self.stuck_check_time = 0.0

        # RETURNING timer (adaptieve tolerantie) + tussentijds routepunt (FIX-6C)
        self.returning_start_time = 0.0
        self.returning_via        = None

        # PID koersregeling (conform design project, ongewijzigd)
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

        # Telemetrie
        self.last_logged_pct = 110
        self.last_carpet_log = -99.0

        # Watchdog
        self.prev_cycle_time = 0.0

        self._setup_devices()
        self._set_leds("UNDOCKING")
        print(f"[SYSTEEM] Controller v5.12 gestart. State: {self.state}")
        print(f"[SYSTEEM] Batterij max: {BATTERY_MAX:.0f} J | "
              f"Terugkeerdrempel: {BATTERY_LOW_PCT*100:.0f}% | "
              f"Roaming: {ROAMING_DURATION:.0f}s per fase")
        print(f"[SYSTEEM] Navigatie: dynamisch (geen hardcoded meubelzones) | "
              f"WP-radius: {BOUS_WP_RADIUS}m | Frustration: {WP_FRUSTRATION_TIME}s")
        print(f"[MISSIE]  Fase 1: {self.mission_phase}")

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

    # ── actuatoren ────────────────────────────────────────────────────────────
    def set_motors(self, left, right):
        self.motor_left.setVelocity(clamp(left,  -MAX_SPEED, MAX_SPEED))
        self.motor_right.setVelocity(clamp(right, -MAX_SPEED, MAX_SPEED))

    def _set_leds(self, state):
        """
        LED-sturing per state (conform system requirements):
          STOFZUIGEN : status aan        (actief stofzuigen)
          DWEILEN    : beide aan         (dweilmodus zichtbaar)
          ESCAPE     : beide uit         (noodstop)
          RETURNING  : beide aan         (terugkeer naar lader)
          ALIGNING   : status aan        (uitlijnen)
          DOCKING    : status aan        (insturen lader)
          CHARGING   : beide uit         (stilstaand laden)
          UNDOCKING  : status aan        (losschieten lader)
        """
        leds = {
            "STOFZUIGEN": (1, 0),
            "DWEILEN":    (1, 1),
            "ESCAPE":     (0, 0),
            "RETURNING":  (1, 1),
            "ALIGNING":   (1, 0),
            "DOCKING":    (1, 0),
            "CHARGING":   (0, 0),
            "UNDOCKING":  (1, 0),
            "SCANNING":   (1, 0),   # LiDAR-kamerafmeting (knippert status-LED)
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
        """
        Fallback: genereer waypoints met vaste (hardcoded) kamerafmetingen.
        Wordt aangeroepen bij initialisatie. Na een succesvolle SCANNING-state
        worden de waypoints hergegenereerd via _generate_boustrophedon() met
        dynamisch ontdekte kamergrenzen.
        """
        return self._generate_boustrophedon(
            BOUS_X_MIN, BOUS_X_MAX, BOUS_Y_MIN, BOUS_Y_MAX
        )

    def _generate_dweilen_waypoints(self):
        """
        Genereer DWEILEN-specifieke waypoints die het tapijt vermijden.

        Strategie:
          • Stroken BUITEN de tapijt-Y-zone (Y < carpet_y_min - buffer):
            volledige breedte BOUS_X_MIN → BOUS_X_MAX (robot kan vrij).
          • Stroken IN de tapijt-Y-zone (carpet_y_min - buffer ≤ Y ≤ carpet_y_max + buffer):
            alleen het OOST-deel: carpet_x_east → BOUS_X_MAX.
            Zo wordt de vloer OOST van het tapijt toch gemopped zonder het tapijt te betreden.

        FIX-v5.12: Vervangt de Y-only WP-skip in _execute_cleaning (v5.11).
        De v5.11-aanpak sloeg ALLE WPs in de tapijt-Y-band over — ook die op
        X=2.20 (oost van tapijt) — waardoor de vloer rechts van het tapijt nooit
        gedweild werd. Door separate WPs te genereren met beperkte X-reeks wordt
        die zone wél bereikt terwijl het tapijt zelf nooit betreden wordt.

        Fallback als carpet_map nog leeg is: gebruik de hardcoded CARPET_*-grenzen.
        """
        cy_min = self._min_carpet_y()
        cy_max = self._max_carpet_y()
        cx_east = self._carpet_x_east()
        buffer = 0.20

        cy_lo = cy_min - buffer   # onderste Y-grens tapijt-zone (met buffer)
        cy_hi = cy_max + buffer   # bovenste Y-grens tapijt-zone (met buffer)

        pts = []
        y = BOUS_Y_MIN
        going_east = True

        while y <= BOUS_Y_MAX + 0.01:
            yr = round(y, 2)

            # Lader-hoekuitsluiting: zelfde als in _generate_boustrophedon
            if yr > CHARGER_Y - 0.80:
                x_east = min(BOUS_X_MAX, CHARGER_X - 0.50)
            else:
                x_east = BOUS_X_MAX

            # Bepaal westgrens: in tapijt-Y-zone → alleen oost-deel, anders vol breedte
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
        print(f"[DWEILEN-WP] {len(pts)} waypoints gegenereerd: "
              f"{n_full} volbreedte-stroken, {n_east} oost-stroken "
              f"(X≥{cx_east:.2f}m, tapijt-Y-zone {cy_lo:.2f}..{cy_hi:.2f})")
        return pts

    def _generate_boustrophedon(self, x_min, x_max, y_min, y_max):
        """
        Genereer een uniform zigzag-rasterpatroon binnen de opgegeven kamergrenzen.

        Gebaseerd op ClearView™ LiDAR navigatieprincipe van de echte Roomba® 205:
        'maximizes floor-cleaning coverage wall-to-wall, cleans in neat rows'
        (bron: irobot.com)

        Aangeroepen met hardcoded BOUS_* waarden als fallback bij initialisatie,
        en met dynamisch gemeten grenzen na de SCANNING-state zodat de controller
        correct werkt ongeacht de kamergrootte.

        Obstakels worden reactief omzeild (LiDAR + frustration timeout) — geen
        hardcoded meubelzones nodig.
        """
        pts = []
        y = y_min
        going_east = True
        while y <= y_max + 0.01:
            yr = round(y, 2)
            # Lader-hoekuitsluiting: het oost-uiteinde van stroken dicht bij de
            # lader (Y > CHARGER_Y - 0.80 = 2.05m) wordt teruggebracht tot
            # CHARGER_X - 0.50 = 1.50m om de lader-arm te vermijden.
            # Zo worden WPs in de geblokkeerde noordoost-hoek nooit gegenereerd.
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
        Bereken kamergrenzen uit de verzamelde LiDAR-wandpunten (SCANNING-state).

        Methode: 5e/95e percentielfiltering van X- en Y-coördinaten.
        Dit verwijdert uitschieters door meubels of vloer-oneffenheden en geeft
        een betrouwbare schatting van de echte wandposities.

        Na berekening worden bous_waypoints hergegenereerd en wp_done gereset,
        zodat de robot de volledige kamer in banen afdekt.
        """
        if len(self.scan_wall_pts) < 50:
            print(f"[PASSIEVE SCAN] Te weinig punten ({len(self.scan_wall_pts)}) "
                  f"→ standaard kamerafmetingen behouden")
            return

        xs = sorted(p[0] for p in self.scan_wall_pts)
        ys = sorted(p[1] for p in self.scan_wall_pts)
        n  = len(xs)

        # 5e/95e percentiel: robuust tegen uitschieters door meubels
        p5  = max(0, n // 20)
        p95 = min(n - 1, n - n // 20)

        raw_x_min = xs[p5]
        raw_x_max = xs[p95]
        raw_y_min = ys[p5]
        raw_y_max = ys[p95]

        # Voeg veiligheidsmarge toe (SCAN_MARGIN) zodat robot niet te dicht bij wand komt
        new_x_min = raw_x_min + SCAN_MARGIN
        new_x_max = raw_x_max - SCAN_MARGIN
        new_y_min = raw_y_min + SCAN_MARGIN
        new_y_max = raw_y_max - SCAN_MARGIN

        # Minimale kamergrootte bewaken (voorkomt degeneratie bij slechte scan)
        if new_x_max - new_x_min < 1.0 or new_y_max - new_y_min < 1.0:
            print(f"[PASSIEVE SCAN] Onredelijke kamergrenzen "
                  f"X=[{new_x_min:.2f},{new_x_max:.2f}], "
                  f"Y=[{new_y_min:.2f},{new_y_max:.2f}] → standaard behouden")
            return

        print(f"[SCANNING] Kamer ontdekt: "
              f"X=[{new_x_min:.2f}, {new_x_max:.2f}], "
              f"Y=[{new_y_min:.2f}, {new_y_max:.2f}] "
              f"({n} punten, marge={SCAN_MARGIN}m)")

        old_count = len(self.bous_waypoints)
        self.bous_waypoints = self._generate_boustrophedon(
            new_x_min, new_x_max, new_y_min, new_y_max
        )
        if not keep_wp_done:
            # Volledig opnieuw beginnen (eerste scan vanuit SCANNING-state)
            self.bous_wp_idx = 0
            self.wp_done = {"STOFZUIGEN": set(), "DWEILEN": set()}
        # Als keep_wp_done=True: huidige wp_done blijft behouden; nieuwe waypoints
        # worden gebruikt vanaf de volgende fase (DWEILEN). wp_idx wordt bijgewerkt
        # in UNDOCKING TURN via de "eerste ongereinigde WP"-zoeklogica.
        print(f"[PASSIEVE SCAN] Waypoints bijgewerkt: "
              f"{old_count} → {len(self.bous_waypoints)} WP "
              f"({'wp_done behouden' if keep_wp_done else 'verse start'})")

    # ── sensoren ──────────────────────────────────────────────────────────────
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
        if pct % 10 == 0 and pct != self.last_logged_pct and 0 <= pct <= 100:
            print(f"[BATTERIJ] {pct}% ({self.battery:.0f} J)")
            self.last_logged_pct = pct
        if pct <= 20 and self.state not in ("CHARGING", "DOCKING"):
            self.battery_led.set(1)

        # ── Persistente tapijt-gridkaart: update ALLEEN tijdens STOFZUIGEN ──
        # FIX-v5.10 Bug1: Kaart alleen bijwerken in STOFZUIGEN — niet in DOCKING/
        # CHARGING (anders markeert de lader-dock als tapijt door valse sensorreading).
        # Exact één cel markeren (geen 3×3 buurt) — de 0.40m buffer in _wp_in_carpet_zone
        # biedt al voldoende veiligheidsmarge. Dit voorkomt vals-positieve cellen op cy=-1
        # (Y=-0.25m) die de GPS-bewaking over de hele kamer lieten vuren.
        # Log-spam beperkt: alleen elke 10e nieuwe cel loggen.
        if self.state == "STOFZUIGEN" and self.carpet_val > CARPET_FULL:
            bearing_rad = math.radians(self.bearing)
            sx = self.pos[0] + 0.16 * math.sin(bearing_rad)
            sy = self.pos[1] + 0.16 * math.cos(bearing_rad)
            cx = int(sx / CARPET_CELL_SIZE)
            cy = int(sy / CARPET_CELL_SIZE)
            cell = (cx, cy)
            if cell not in self.carpet_map:
                self.carpet_map.add(cell)
                if len(self.carpet_map) % 10 == 1:   # log 1e, 11e, 21e, ... cel
                    print(f"[TAPIJT-KAART] Cel ({cx},{cy}) "
                          f"bij sensor=({sx:.2f},{sy:.2f}), "
                          f"totaal {len(self.carpet_map)} cellen")

    # ── watchdog ──────────────────────────────────────────────────────────────
    def _watchdog(self, t):
        elapsed = t - self.prev_cycle_time
        if elapsed > 0.150 and self.prev_cycle_time > 0:
            print(f"[WATCHDOG] Cyclustijd overschreden: {elapsed:.3f}s")
        self.prev_cycle_time = t

    # ── noodstop ──────────────────────────────────────────────────────────────
    def _check_emergencies(self, t):
        """Bumper of afgrond → ESCAPE. Actief in reinigingsstates en RETURNING."""
        if self.state not in (*self.CLEANING_STATES, "RETURNING"):
            return

        # FIX-3: bij lage batterij tijdens RETURNING geen ESCAPE meer.
        # Verhoogd naar 15% (was 8%): logs tonen dat bumper-hits tijdens RETURNING
        # ESCAPE-lussen veroorzaken die de resterende batterij volledig uitputten.
        # Robot rijdt direct naar charger; botsingsschade weegt niet op tegen zeker
        # energieverlies door eindeloze ESCAPE-spiraal.
        if self.state == "RETURNING" and self.battery < 0.15 * BATTERY_MAX:
            return

        trigger = ""
        if self.bumper_hit > 0.0:
            trigger = "BUMPER"
        elif self.cliff_hit:
            trigger = "AFGROND"
        if not trigger:
            return

        battery_low = self.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up     = (t - self.roaming_start_time) >= ROAMING_DURATION
        self.prev_state = "RETURNING" if (battery_low or time_up) else self.state

        self.in_carpet_escape = False

        print(f"[NOODSTOP] {trigger} in '{self.state}' → ESCAPE "
              f"(daarna '{self.prev_state}')")
        self.state = "ESCAPE"
        self._set_leds("ESCAPE")

        if t - self.last_evasion_time < 6.0:
            self.evasion_count += 1
        else:
            self.evasion_count = 1
        self.last_evasion_time = t

        if self.evasion_count >= 3:
            print("[WATCHDOG] Hoeksituatie → 180°-rotatie.")
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

    # ── tapijt-ontsnapping (alleen DWEILEN) ───────────────────────────────────
    def _on_carpet_gps(self):
        return (CARPET_X_MIN <= self.pos[0] <= CARPET_X_MAX and
                CARPET_Y_MIN <= self.pos[1] <= CARPET_Y_MAX)

    def _wp_in_carpet_zone(self, wx, wy, buffer=0.20):
        """
        True als waypoint (wx, wy) binnen de tapijt-Y-band + buffer valt.

        FIX-v5.11: Alleen Y-as wordt gecontroleerd (niet meer 2D gridcel-afstand).
        Reden: de oude 2D-check sloeg WPs aan X=-2.2 (ver van tapijt in X) NIET over,
        terwijl de GPS-bewaking wél vuurde op die Y-posities → eindeloze oscillatie.
        Een Y-only check is consistent met de GPS-bewaking en de fysieke realiteit:
        het tapijt loopt over (bijna) de volledige breedte van de kamer.

        buffer = 0.20m: veiligheidsmarge ten zuiden/noorden van tapijt-Y-grenzen.
        """
        if not self.carpet_map:
            return CARPET_Y_MIN - buffer <= wy <= CARPET_Y_MAX + buffer

        cy_min = self._min_carpet_y()
        cy_max = self._max_carpet_y()
        return (cy_min - buffer) <= wy <= (cy_max + buffer)

    def _min_carpet_y(self):
        """Laagste Y-coördinaat van gekarteeerde tapijt-cellen (in meters)."""
        if not self.carpet_map:
            return CARPET_Y_MIN
        return min(cy * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def _max_carpet_y(self):
        """Hoogste Y-coördinaat van gekarteeerde tapijt-cellen (in meters)."""
        if not self.carpet_map:
            return CARPET_Y_MAX
        return max(cy * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def _carpet_x_east(self):
        """
        Veilige start-X voor oost-enkel DWEILEN-stroken.
        = bovengrens van meest oostelijke tapijt-gridcel + 0.20m veiligheidsbuffer.
        Fallback als carpet_map leeg is: CARPET_X_MAX + 0.30m.
        """
        if not self.carpet_map:
            return CARPET_X_MAX + 0.30          # = 0.60m als fallback
        max_cx = max(cx for (cx, _) in self.carpet_map)
        return (max_cx + 1) * CARPET_CELL_SIZE + 0.20  # bovengrens cel + buffer

    def _handle_carpet_escape(self, t):
        """
        Sensor-gestuurde tapijtvermijding voor DWEILEN.
        Vuurt als carpet_sensor boven drempel of GPS meldt robot op tapijt.
        Altijd SOUTH ontsnappen: robot rijdt naar Y = carpet_y_min - 0.50.

        FIX-v5.11: Altijd SOUTH ontsnappen (verwijderd: Oost-optie).
        In DWEILEN benadert de robot het tapijt altijd vanuit het zuiden
        (alle tapijt-WPs zijn geskipt via Y-band check). SOUTH is dus altijd
        de juiste richting. De Oost-ontsnapping veroorzaakte lussen omdat
        het doel halverwege het tapijt lag.

        FIX-v5.11: Geen safe_south meer in de Vrij-conditie.
        De Y-band skip zorgt dat de robot na de ontsnapping niet meer terug-
        navigeert naar een WP in de tapijt-Y-zone. sensor_clear + not gps_on
        is voldoende.
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
            # Altijd SOUTH: zet robot duidelijk ten zuiden van tapijt-rand
            target_y = carpet_y_min - 0.50
            self.carpet_escape_target = (self.pos[0], target_y)
            print(f"[TAPIJT] Ontsnapping #{self.carpet_escape_count} "
                  f"(sensor={self.carpet_val:.0f}, "
                  f"GPS={self.pos[0]:.2f},{self.pos[1]:.2f}) → SOUTH "
                  f"(doel Y={target_y:.2f}, tapijt_min_Y={carpet_y_min:.2f})")
            self.last_carpet_log = t

        if not self.in_carpet_escape:
            return False

        # Vrij-conditie: sensor helder + GPS bevestigt buiten tapijt
        sensor_clear = self.carpet_val < (CARPET_EDGE - 30)
        if sensor_clear and not gps_on:
            self.in_carpet_escape = False
            print(f"[TAPIJT] Vrij "
                  f"(sensor={self.carpet_val:.0f}, "
                  f"GPS={self.pos[0]:.2f},{self.pos[1]:.2f})")
            return False

        # Navigeer naar ontsnappingsdoel (zuiden)
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

    # ── gedeelde reinigingslogica ─────────────────────────────────────────────
    def _execute_cleaning(self, t, allow_carpet):
        """
        Gedeelde rijlogica voor STOFZUIGEN en DWEILEN.
        allow_carpet=True  → tapijt mag (STOFZUIGEN)
        allow_carpet=False → tapijt vermijden (DWEILEN)

        Navigatieprioriteiten:
          1. Terugkeer bij lage batterij of tijdslimiet
          2. Tapijtvermijding via carpet-sensor + GPS (DWEILEN only)
          3. LiDAR obstakel-ontwijking (reactieve laag, altijd actief)
          4. Boustrophedon GPS-navigatie (ClearView™ LiDAR stijl, wand-tot-wand)
             + FIX-7A frustration timeout (dynamisch, geen hardcoded meubels)
             + FIX-7C speed ramping + look-ahead (vloeiender rijgedrag)
        """
        # ── Prioriteit 0: Fase volledig afgerond → meteen terugkeren ─────
        # Als alle waypoints in de huidige fase verwerkt zijn, hoeft de
        # robot niet te wachten tot de batterij leeg is. Ga direct naar
        # RETURNING zodat CHARGING de fasewisseling kan afhandelen.
        #
        # FIX-v5.10 Bug3: Passieve scan hergeneratie UITGESCHAKELD.
        # _compute_room_bounds() schoot te ver uit (±2.60m i.p.v. ±2.40m)
        # door schuin invallende LiDAR-stralen op wandkanten. Dit veroorzaakte
        # 30 i.p.v. 28 waypoints en navigatie naar Y=±2.60 (nabij wanden/lader),
        # wat extra bumper-hits en batterijverlies veroorzaakte.
        # De BOUS_*-constanten bieden al correcte kamergrenzen voor deze ruimte.
        phase_key = "STOFZUIGEN" if allow_carpet else "DWEILEN"
        if len(self.wp_done[phase_key]) >= len(self.bous_waypoints):
            print(f"[MISSIE] {phase_key} volledig gereinigd "
                  f"({len(self.wp_done[phase_key])}/{len(self.bous_waypoints)} WP) "
                  f"→ RETURNING voor fasewisseling")
            self.pid_bearing.reset()
            self.in_carpet_escape     = False
            self.returning_start_time = 0.0
            self.cleaning_speed       = 0.0
            self.state = "RETURNING"
            self._set_leds("RETURNING")
            return

        # ── Prioriteit 1: Controleer batterij en tijdslimiet ──────────────
        battery_low = self.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up     = (t - self.roaming_start_time) >= ROAMING_DURATION

        if battery_low or time_up:
            reason = "tijdslimiet 5 min" if time_up else f"lage batterij ({BATTERY_LOW_PCT*100:.0f}%)"
            print(f"[STATE CHANGE] {self.state} → RETURNING ({reason})")
            self.pid_bearing.reset()
            self.in_carpet_escape     = False
            self.returning_start_time = 0.0   # FIX-5: herstart RETURNING-timer
            self.cleaning_speed       = 0.0   # FIX-7C: reset speed ramp
            self.state = "RETURNING"
            self._set_leds("RETURNING")
            return

        # ── Prioriteit 2: Tapijtvermijding (DWEILEN only) ─────────────────
        # FIX-v5.11: GPS-proactieve bewaking VERWIJDERD.
        # De bewaking vuurde te breed: elke Y boven de drempel (inclusief de
        # laderzone Y=2.22 en de hele zuidhelft van de kamer) werd als "gevaarlijk"
        # beschouwd, waardoor de robot eindeloos rondjes draaide.
        # De Y-band WP-skip (Priority 4) garandeert nu dat de robot NOOIT een WP
        # in de tapijt-Y-zone navigeert. De sensor-escape in _handle_carpet_escape
        # is de enige veiligheidslaag die nog nodig is — en die is al voldoende.
        if not allow_carpet:
            if self._handle_carpet_escape(t):
                return

        # ── Prioriteit 3: LiDAR obstakel-ontwijking ───────────────────────
        obs_front       = self.d_front       < OBSTACLE_DIST
        obs_front_left  = self.d_front_left  < OBSTACLE_DIST
        obs_front_right = self.d_front_right < OBSTACLE_DIST
        # FIX-4A: FAR_OBSTACLE_DIST (0.28m) voor zij-sectoren → zachter bijsturen
        obs_far_left    = self.d_far_left    < FAR_OBSTACLE_DIST
        obs_far_right   = self.d_far_right   < FAR_OBSTACLE_DIST
        slow_front      = self.d_front       < SLOW_DIST

        any_obstacle = (obs_front or obs_front_left or obs_front_right
                        or obs_far_left or obs_far_right)

        if any_obstacle:
            # FIX-7C: verlaag gerampte snelheid bij obstakel; bij terugkeer naar
            # vrije rijden start robot vloeiend opnieuw in plaats van abrupt.
            self.cleaning_speed = max(0.0,
                                      self.cleaning_speed - 0.15 * MAX_SPEED)

        if obs_front:
            # Rechtstreeks obstakel voor → draai naar meest open kant
            if self.d_far_left >= self.d_far_right:
                self.set_motors(-0.20 * MAX_SPEED, 0.55 * MAX_SPEED)
            else:
                self.set_motors(0.55 * MAX_SPEED, -0.20 * MAX_SPEED)

        elif obs_front_left and obs_front_right:
            # FIX-4B: smal-doorgang detectie (bijv. tussen stoelpoten)
            # Als er ruimte vóór de robot is maar beide voor-zij-sectoren geblokkeerd
            # → langzaam rechtdoor kruipen i.p.v. draaien.
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
            # Geen hardcoded meubelzones; dynamisch via LiDAR + frustration timeout.
            #
            # FIX-v5.12: DWEILEN-WP-skip lus VERWIJDERD.
            # De v5.11-lus sloeg alle WPs in de tapijt-Y-band over (Y≈0.05..1.95),
            # ook die op X=2.20m (oost van tapijt), waardoor de vloer rechts van
            # het tapijt nooit gedweild werd.
            # v5.12 genereert aparte DWEILEN-waypoints via _generate_dweilen_waypoints():
            # stroken in de tapijt-Y-zone starten pas aan carpet_x_east (≈0.60m)
            # in plaats van BOUS_X_MIN (-2.35m). Zo wordt de oost-zone wél gemopped
            # zonder het tapijt zelf te betreden. Geen runtime skip-lus meer nodig.

            wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
            dx_wp  = wp_x - self.pos[0]
            dy_wp  = wp_y - self.pos[1]
            dist_wp = math.hypot(dx_wp, dy_wp)

            # ── FIX-7A: Waypoint frustration timeout ──────────────────────
            # Alleen actief als robot dicht bij waypoint is (< WP_NEAR_THRESH).
            # Ver weg is obstakelomzeiling gewoon; frustration zou dan vals positief
            # vuren (logs: WP#0 (-2.4,-2.4) en oost-waypoints onterecht geskipt).
            if dist_wp < WP_NEAR_THRESH:
                # Initialiseer tracking bij eerste aanroep of na reset
                if self.wp_progress_time == 0.0:
                    self.wp_progress_time = t
                    self.wp_prev_dist     = dist_wp

                if dist_wp < self.wp_prev_dist - 0.04:
                    # Betekenisvolle vooruitgang (>4cm dichter bij waypoint)
                    self.wp_progress_time = t
                    self.wp_prev_dist     = dist_wp
                elif (t - self.wp_progress_time) > WP_FRUSTRATION_TIME:
                    # Dichtbij maar vastgelopen: obstakel blokkeert waypoint
                    old_wp    = self.bous_wp_idx
                    phase_key = "STOFZUIGEN" if allow_carpet else "DWEILEN"
                    self.wp_done[phase_key].add(old_wp)
                    self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                    n_done = len(self.wp_done[phase_key])
                    n_total = len(self.bous_waypoints)
                    print(f"[NAVIGATIE] WP#{old_wp} ({wp_x:.1f},{wp_y:.1f}) "
                          f"overgeslagen na {WP_FRUSTRATION_TIME:.0f}s blokkage "
                          f"→ WP#{self.bous_wp_idx} [{n_done}/{n_total} gedaan]")
                    self.wp_progress_time = t
                    self.wp_prev_dist     = float('inf')
                    wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                    dx_wp  = wp_x - self.pos[0]
                    dy_wp  = wp_y - self.pos[1]
                    dist_wp = math.hypot(dx_wp, dy_wp)
            else:
                # Ver van waypoint: reset frustration-timer en navigeer gewoon
                self.wp_progress_time = 0.0
                self.wp_prev_dist     = dist_wp

            # ── Waypoint bereikt → markeer als gedaan, ga naar volgende ──────
            if dist_wp < BOUS_WP_RADIUS:
                phase_key = "STOFZUIGEN" if allow_carpet else "DWEILEN"
                self.wp_done[phase_key].add(self.bous_wp_idx)
                self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                self.wp_progress_time = t
                self.wp_prev_dist     = float('inf')
                # Log elke nieuwe strook (elke 2 waypoints = 1 strook)
                if self.bous_wp_idx % 2 == 0:
                    next_wp = self.bous_waypoints[self.bous_wp_idx]
                    strip_n = self.bous_wp_idx // 2
                    total_s = len(self.bous_waypoints) // 2
                    n_done  = len(self.wp_done[phase_key])
                    print(f"[NAVIGATIE] Strook {strip_n}/{total_s} → Y={next_wp[1]:.2f} "
                          f"[{n_done}/{len(self.bous_waypoints)} WP gedaan]")
                wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                dx_wp  = wp_x - self.pos[0]
                dy_wp  = wp_y - self.pos[1]
                dist_wp = math.hypot(dx_wp, dy_wp)

            # ── Koers berekening naar waypoint ────────────────────────────
            target_deg = math.degrees(math.atan2(dx_wp, dy_wp))
            if target_deg < 0.0:
                target_deg += 360.0

            # FIX-7C: Look-ahead — mix stuursignaal met richting naar volgend
            # waypoint zodra robot dicht genoeg bij huidig waypoint is.
            # Resultaat: de robot begint de bocht al eerder, vloeiender.
            if dist_wp < LOOK_AHEAD_DIST:
                next_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                nwx, nwy = self.bous_waypoints[next_idx]
                ndx = nwx - self.pos[0]
                ndy = nwy - self.pos[1]
                next_deg = math.degrees(math.atan2(ndx, ndy))
                if next_deg < 0.0:
                    next_deg += 360.0
                # blend: 0.0 op LOOK_AHEAD_DIST, max 0.50 bij waypoint
                blend = clamp(1.0 - dist_wp / LOOK_AHEAD_DIST, 0.0, 0.50)
                diff  = angle_error(next_deg, target_deg)
                target_deg = (target_deg + blend * diff) % 360.0

            # FIX-7C: turn gain verlaagd 0.05 → 0.040 voor zachter afbuigen
            bearing_err = angle_error(target_deg, self.bearing)
            turn = clamp(0.040 * bearing_err, -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)

            # Wandsensor P-correctie (sensor gebruikt conform system requirements)
            # Detecteert rechterwand voor zachte koersbijstelling
            wall_corr = 0.0
            if self.wall_val > 80.0:
                wall_err  = 380.0 - self.wall_val
                wall_corr = clamp(0.0008 * wall_err,
                                  -0.06 * MAX_SPEED, 0.06 * MAX_SPEED)

            total_turn = clamp(turn + wall_corr,
                               -0.35 * MAX_SPEED, 0.35 * MAX_SPEED)

            # FIX-7C: Speed ramping — graduele versnelling/vertraging
            # Exploration factor: traag opbouwen aan begin nieuwe fase (verkenning),
            # sneller bij hervatting. Ramt exponentieel naar 1.0.
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

            # ── Passieve wandkartering (STOFZUIGEN only, max 1×/5s) ───────
            # Verzamel LiDAR-wandpunten tijdens normaal rijden zonder toegewijde
            # scanfase. Na eerste volledige STOFZUIGEN worden kamercontouren
            # herberekend (zie Prioriteit 0). Zo gedraagt de robot zich zoals een
            # echte LiDAR-stofzuiger: kaart wordt incrementeel opgebouwd.
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

    # ── stuck-detectie ────────────────────────────────────────────────────────
    def _check_stuck(self, t):
        """
        Positie-gebaseerde stuck-detectie (geïnspireerd op classmate: encodertelling).
        Als robot >STUCK_TIMEOUT sec minder dan STUCK_DIST m beweegt tijdens
        een reinigingsstate → ESCAPE-spin om de impasse te doorbreken.
        """
        if self.state not in self.CLEANING_STATES:
            # Buiten reinigingsstates: reset tracker zodat hij vers begint
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
            # Voldoende beweging → reset timer
            self.stuck_check_pos  = (self.pos[0], self.pos[1])
            self.stuck_check_time = t
        elif t - self.stuck_check_time > STUCK_TIMEOUT:
            print(f"[STUCK] Geen {STUCK_DIST}m beweging in {STUCK_TIMEOUT:.0f}s "
                  f"(pos={self.pos[0]:.2f},{self.pos[1]:.2f}) → ESCAPE-spin")
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
                    print("[UNDOCKING] Achteruit klaar → draaifase naar 180°.")

            elif self.undock_phase == "TURN":
                err = angle_error(180.0, self.bearing)
                if abs(err) < 2.5 or (t - self.undock_timer > 5.0):
                    # Direct naar reinigingsfase — geen toegewijde scanfase.
                    # Kaarting gebeurt incrementeel tijdens het rijden (passieve scan).
                    self.roaming_start_time   = t
                    self.returning_start_time = 0.0
                    self.returning_via        = None
                    self.in_carpet_escape     = False
                    # FIX-7C: reset speed ramp bij start reinigingsfase
                    self.cleaning_speed   = 0.0
                    # FIX-7A: reset frustration tracking bij nieuw reinigingsstart
                    self.wp_progress_time = 0.0
                    self.wp_prev_dist     = float('inf')

                    # Persistente navigatie: hervat bij eerste ONGEREINIGD waypoint.
                    # Hierdoor begint de robot na opladen NIET opnieuw van voren,
                    # maar gaat verder waar hij gebleven was (zoals echte Roomba).
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
                        # Alle waypoints reeds gedaan (kan voorkomen bij reset)
                        self.bous_wp_idx = 0

                    # Verkenningsfactor: voorzichtiger bij nieuwe fase, vertrouwd bij hervatting
                    if self.fresh_phase:
                        self.exploration_factor = 0.50   # nieuwe fase: langzame start
                    else:
                        self.exploration_factor = 0.85   # hervatting: redelijk vertrouwen
                    self.fresh_phase = False

                    print(f"[NAVIGATIE] Hervatting {self.mission_phase}: "
                          f"{len(phase_done)}/{n_wps} WP gedaan, "
                          f"begin bij WP#{self.bous_wp_idx}, "
                          f"exploration={self.exploration_factor:.2f}")
                    self.state = self.mission_phase
                    self._set_leds(self.mission_phase)
                    print(f"[STATE CHANGE] UNDOCKING → {self.mission_phase} "
                          f"(start t={t:.0f}s, WP#{self.bous_wp_idx}/"
                          f"{len(self.bous_waypoints)})")
                else:
                    turn_sp = clamp(0.04 * abs(err),
                                    0.08 * MAX_SPEED, 0.40 * MAX_SPEED)
                    if err > 0:
                        self.set_motors(turn_sp, -turn_sp)
                    else:
                        self.set_motors(-turn_sp, turn_sp)

        # ── STOFZUIGEN ────────────────────────────────────────────────────────
        elif self.state == "STOFZUIGEN":
            # allow_carpet=True: robot mag over tapijt rijden (stofzuiger)
            self._execute_cleaning(t, allow_carpet=True)

        # ── DWEILEN ───────────────────────────────────────────────────────────
        elif self.state == "DWEILEN":
            # allow_carpet=False: tapijt vermijden (dweil mag niet op tapijt)
            self._execute_cleaning(t, allow_carpet=False)

        # ── ESCAPE ────────────────────────────────────────────────────────────
        elif self.state == "ESCAPE":
            if t < self.escape_reverse_end:
                self.set_motors(-0.50 * MAX_SPEED, -0.50 * MAX_SPEED)
            elif t < self.escape_end_time:
                td = self.turn_direction
                self.set_motors(td * 0.45 * MAX_SPEED, -td * 0.45 * MAX_SPEED)
            else:
                # FIX-7C: reset speed ramp na ESCAPE → vloeiende herstart
                # FIX-7A: reset frustration tracking → robot krijgt verse kans
                self.cleaning_speed   = 0.0
                self.wp_progress_time = 0.0
                self.wp_prev_dist     = float('inf')
                print(f"[STATE CHANGE] ESCAPE → {self.prev_state}")
                self.state = self.prev_state
                self._set_leds(self.prev_state)

        # ── RETURNING ─────────────────────────────────────────────────────────
        elif self.state == "RETURNING":
            # FIX-5: track verstreken RETURNING-tijd voor adaptieve tolerantie
            # FIX-6C: tussentijds routepunt wanneer robot diep in tafelzone zit
            if self.returning_start_time == 0.0:
                self.returning_start_time = t
                # Wiskundig bewezen: robot Y < 0.3 → rechte lijn naar predock
                # kruist tafelzone. Omleiding via (0.5, 0.5) vermijdt dit:
                #   (pos)→(0.5,0.5): geen tafelkruising ✅
                #   (0.5,0.5)→predock(2.0,1.8): geen tafelkruising ✅
                if self.pos[1] < 0.30:
                    # FIX-v5.10: relatief aan PREDOCK in plaats van hardcoded (0.5, 0.5)
                    via_x = PREDOCK_X - 1.50                      # 0.50m west van predock
                    via_y = max(0.30, PREDOCK_Y - 1.30)           # 1.30m south van predock
                    self.returning_via = (via_x, via_y)
                    print(f"[RETURNING] Robot Y={self.pos[1]:.2f} < 0.30 → "
                          f"omleiding via tussentijds punt ({via_x:.2f},{via_y:.2f})")
                else:
                    self.returning_via = None

            # Bepaal huidige navigatiedoelstelling (via-punt of predock)
            if self.returning_via is not None:
                tx, ty = self.returning_via
                if math.hypot(tx - self.pos[0], ty - self.pos[1]) < 0.35:
                    print(f"[RETURNING] Via-punt bereikt "
                          f"({self.pos[0]:.2f},{self.pos[1]:.2f}) → koers predock")
                    self.returning_via = None
                    tx, ty = PREDOCK_X, PREDOCK_Y
            else:
                tx, ty = PREDOCK_X, PREDOCK_Y

            dx = tx - self.pos[0]
            dy = ty - self.pos[1]

            # Tolerantie-check: alleen voor finale predock-bestemming
            if self.returning_via is None:
                ret_elapsed = t - self.returning_start_time
                if ret_elapsed > 120.0:
                    tol_x, tol_y = 0.15, 0.40   # noodmodus
                elif ret_elapsed > 60.0:
                    tol_x, tol_y = 0.09, 0.32   # versoepeld
                else:
                    tol_x, tol_y = 0.06, 0.25   # nominaal (strikt voor arm-vrij)

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
            # BUG-16 FIX: drempel 4.0° → 6.0° (voorkomt timeout bij 4.6°)
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
                turn_sp = clamp(0.06 * abs(err),
                                0.02 * MAX_SPEED, 0.18 * MAX_SPEED)
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
                      f"({trigger}, afstand={abs(CHARGER_Y - self.pos[1]):.2f}m)")
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
                    print(f"[DOCKING] Stall buiten radius (Y={self.pos[1]:.2f}) → ALIGNING")
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
                # FIX-1B: GPS X-correctie tijdens insturen
                # x_err > 0 → robot links van lader → stuur rechts (CW: +L,-R)
                # x_err < 0 → robot rechts van lader → stuur links (CCW: -L,+R)
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
                print(f"[CHARGING] Laadcyclus gestart "
                      f"(t={t:.0f}s, batterij={pct}%, "
                      f"volgende fase: "
                      f"{'DWEILEN' if self.mission_phase == 'STOFZUIGEN' else 'STOFZUIGEN'})")

            if self.pos[1] < DOCK_STALL_RADIUS_Y:
                print(f"[CHARGING] Buiten laadzone (Y={self.pos[1]:.2f}) → DOCKING")
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
                """Start UNDOCKING. Wissel fase alleen als alle waypoints verwerkt zijn."""
                n_total    = len(self.bous_waypoints)
                phase_key  = self.mission_phase
                n_done     = len(self.wp_done[phase_key])
                t_now      = self.robot.getTime()
                dur        = t_now - self.last_charge_time if self.last_charge_time else 0

                if n_done >= n_total:
                    # Fase volledig afgerond → wissel naar andere fase
                    old_phase          = self.mission_phase
                    self.mission_phase = ("DWEILEN" if old_phase == "STOFZUIGEN"
                                          else "STOFZUIGEN")
                    self.wp_done[self.mission_phase].clear()   # schoon begin nieuwe fase
                    self.fresh_phase   = True
                    self.bous_wp_idx   = 0                      # FIX-v5.12: altijd hervatten van 0 bij nieuwe fase

                    # FIX-v5.12: Genereer fase-specifieke waypoints bij fasewisseling.
                    # DWEILEN: oost-enkel stroken in tapijt-Y-zone (tapijt vermijden).
                    # STOFZUIGEN: volledige breedte (standaard boustrophedon).
                    if self.mission_phase == "DWEILEN":
                        self.bous_waypoints = self._generate_dweilen_waypoints()
                    else:
                        self.bous_waypoints = self._generate_waypoints()

                    print(f"[STATE CHANGE] CHARGING → UNDOCKING "
                          f"(batterij {pct}%, laadduur: {dur:.0f}s)")
                    print(f"[MISSIE] {old_phase} volledig gereinigd "
                          f"({n_done}/{n_total} WP) → nieuwe fase: {self.mission_phase} "
                          f"({len(self.bous_waypoints)} WP)")
                else:
                    # Fase nog niet klaar → zelfde fase hervatten (waypoints ongewijzigd)
                    remaining = n_total - n_done
                    self.fresh_phase = False
                    print(f"[STATE CHANGE] CHARGING → UNDOCKING "
                          f"(batterij {pct}%, laadduur: {dur:.0f}s)")
                    print(f"[MISSIE] {phase_key} hervat: "
                          f"{n_done}/{n_total} WP gedaan, {remaining} resterend")

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
                print(f"[CHARGING] Fallback {CHARGING_MAX_TIME:.0f}s "
                      f"(batterij: {pct}%) → UNDOCKING")
                _start_next_phase()

    # ── hoofdcyclus ───────────────────────────────────────────────────────────
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
