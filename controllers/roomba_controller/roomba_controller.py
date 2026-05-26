"""roomba_controller controller - Herstel naar Stap 3.4 (Inclusief Carpet & Timer)."""

from controller import Robot
import math

# region Helper Functies
def get_bearing_in_degrees(compass):
    north = compass.getValues()
    if not north or math.isnan(north[0]):
        return 0.0
    rad = math.atan2(north[0], north[1])
    bearing = (rad / math.pi) * 180.0
    if bearing < 0.0:
        bearing += 360.0
    return bearing
# endregion

# region Initialisatie en Setup
robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor_left = robot.getDevice('motor_left')
motor_right = robot.getDevice('motor_right')
if motor_left and motor_right:
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    motor_left.setVelocity(0.0)
    motor_right.setVelocity(0.0)

lidar = robot.getDevice('clearview_lidar')
if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()

robot.batterySensorEnable(timestep)

bumper = robot.getDevice('bumper')
if bumper: bumper.enable(timestep)

wall_sensor = robot.getDevice('wall_sensor')
if wall_sensor: wall_sensor.enable(timestep)

carpet_sensor = robot.getDevice('carpet_sensor')
if carpet_sensor: carpet_sensor.enable(timestep)

gps = robot.getDevice('gps')
if gps: gps.enable(timestep)

compass = robot.getDevice('compass')
if compass:
    compass.enable(timestep)
    compass.zAxis = False

cliff_sensors = {
    "left": robot.getDevice('cliff_sensor_left'),
    "right": robot.getDevice('cliff_sensor_right'),
    "front_left": robot.getDevice('cliff_sensor_front_left'),
    "front_right": robot.getDevice('cliff_sensor_front_right')
}
for key, sensor in cliff_sensors.items():
    if sensor: sensor.enable(timestep)
# endregion

# region Variabelen State Machine
current_state = "ROAMING"
max_speed = 6.28 
battery_max = 10000.0

evasion_end_time = 0.0
evasion_phase_1_end = 0.0
evasion_count = 0
last_evasion_time = 0.0

print(f"[STATE CHANGE] INITIALIZING -> {current_state} (Simulatietijd: 0.00s)")
# endregion

# region Main Loop
last_logged_percentage = 100

while robot.step(timestep) != -1:
    current_time = robot.getTime()
    
    battery_level = robot.batterySensorGetValue()
    current_percentage = int((battery_level / battery_max) * 100) if battery_level >= 0 else 0
    if current_percentage % 10 == 0 and current_percentage != last_logged_percentage and current_percentage > 0:
        print(f"[BATTERIJ STATUS] Capaciteit gedaald naar {current_percentage}% ({battery_level:.0f}J)")
        last_logged_percentage = current_percentage
        
    wall_val = wall_sensor.getValue() if wall_sensor else 0.0
    carpet_val = carpet_sensor.getValue() if carpet_sensor else 0.0
    ranges = lidar.getRangeImage() if lidar else []
    bumper_hit = bumper.getValue() if bumper else 0.0
    cliff_detected = any(s.getValue() < 150.0 for s in cliff_sensors.values() if s)
    
    if current_state == "ROAMING":
        
        # TRANSITIE: Timer (5 min) of Batterij (20%)
        if current_time >= 300.0 or battery_level < 2000.0:
            print(f"[STATE CHANGE] ROAMING -> RETURNING (Timer of batterijlimiet bereikt)")
            current_state = "RETURNING"
            continue
            
        is_evading = (current_time < evasion_end_time)
        
        # 1. UITVOERING VAN EEN NOODSTOP (De Anti-Klim uit Stap 3.4)
        if is_evading:
            if current_time < evasion_phase_1_end:
                motor_left.setVelocity(-0.4 * max_speed)
                motor_right.setVelocity(-0.4 * max_speed)
            else:
                motor_left.setVelocity(-0.5 * max_speed)
                motor_right.setVelocity(0.5 * max_speed)
            continue
            
        # 2. WATCHDOG: Bumper, Afgrond én Tapijt
        # TAPIJT TOEGEVOEGD: Behandel het tapijt als een fysieke muur
        if cliff_detected or bumper_hit > 0.0 or carpet_val > 500.0:
            if current_time - last_evasion_time < 8.0:
                evasion_count += 1
            else:
                evasion_count = 1 
                
            last_evasion_time = current_time
            
            if evasion_count >= 3:
                print(f"[WATCHDOG] Lokaal minimum. Diepe ontsnapping gestart op {current_time:.1f}s")
                evasion_phase_1_end = current_time + 1.2  
                evasion_end_time = current_time + 4.0     
                evasion_count = 0 
            else:
                evasion_phase_1_end = current_time + 0.5  
                evasion_end_time = current_time + 1.4     
            continue
            
        # 3. LIDAR SLALOM (De tafelpoten logica uit Stap 3.4)
        left_obstacle = False
        right_obstacle = False
        
        if ranges:
            left_ranges = ranges[80:128]
            right_ranges = ranges[128:176]
            
            valid_left = [r for r in left_ranges if r != float('inf')]
            valid_right = [r for r in right_ranges if r != float('inf')]
            
            if valid_left and min(valid_left) < 0.16:
                left_obstacle = True
            if valid_right and min(valid_right) < 0.16:
                right_obstacle = True
                
        if left_obstacle and right_obstacle:
            motor_left.setVelocity(-0.2 * max_speed)
            motor_right.setVelocity(0.6 * max_speed)
            continue
        elif left_obstacle:
            motor_left.setVelocity(0.8 * max_speed)
            motor_right.setVelocity(0.4 * max_speed)
            continue
        elif right_obstacle:
            motor_left.setVelocity(0.4 * max_speed)
            motor_right.setVelocity(0.8 * max_speed)
            continue
            
        # 4. P-REGELAAR WANDVOLGING (De stabiele Hysteresis uit Stap 3.4)
        base_speed = 0.6 * max_speed 
        
        if wall_val > 80.0:
            setpoint = 400.0   
            Kp = 0.0015 
            
            if 250.0 < wall_val < 550.0:
                motor_left.setVelocity(base_speed)
                motor_right.setVelocity(base_speed)
            else:
                error = setpoint - wall_val
                turn = Kp * error 
                
                left_speed = max(min(base_speed + turn, max_speed), -max_speed)
                right_speed = max(min(base_speed - turn, max_speed), -max_speed)
                motor_left.setVelocity(left_speed)
                motor_right.setVelocity(right_speed)
        else:
            motor_left.setVelocity(base_speed)
            motor_right.setVelocity(base_speed)
            
    elif current_state == "RETURNING":
        # Hier komt in de volgende stap de GPS-navigatie. 
        # Momenteel stopt de robot hier netjes na 5 minuten.
        motor_left.setVelocity(0.0)
        motor_right.setVelocity(0.0)
# endregion