# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Project:        Auton-48F                                                  #
# 	Module:         Auton-48F.py                                               #
# 	Author:         VEX IQ Team 48F                                            #
# 	Created:        2026-10-29                                                 #
# 	Description:    Autonomous Bag Cycle Routine for VEX IQ                    #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

# ==============================================================================
# DEVICE CONFIGURATION
# ==============================================================================
brain = Brain()

# Port 7: Claw Pneumatics
claw_grab = Pneumatic(Ports.PORT7)

# Ports 1 & 2 + Port 0 Inertial: Drivetrain (SmartDrive)
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_2_5, False)
right_motor = Motor(Ports.PORT2, GearSetting.RATIO_2_5, True)
inertial = Inertial(Ports.PORT0)
drivetrain = SmartDrive(left_motor, right_motor, inertial, 200, 190.5, 190.5, MM)

# Ports 5 & 6: Catapult (2-Motor Group)
cata_motor_a = Motor(Ports.PORT5, GearSetting.RATIO_1_1, True)
cata_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_1_1, False)
cata = MotorGroup(cata_motor_a, cata_motor_b)

# Ports 3 & 4: Claw Arm (2-Motor Group)
claw_motor_a = Motor(Ports.PORT3, GearSetting.RATIO_1_1, True)
claw_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_1_1, False)
claw = MotorGroup(claw_motor_a, claw_motor_b)

# ==============================================================================
# CONFIGURABLE PARAMETERS
# ==============================================================================
# T: Total autonomous duration in seconds
AutonDurationSec = 60.0

# X: Distance to drive forward from loading zone toward goal (mm)
DriveForwardDistMM = 400.0

# Y: Angle to turn right to face the goal before launch (degrees)
TurnAngleDeg = 45.0

# Claw and Catapult calibration parameters
MoveBeforePickmm = 100.0       # Forward drive distance to contact bag before clamping
ClawAboveGroundDeg = 20.0      # Degrees to lower claw arm from rest to ground
ClawMotorMoveDeg = 140.0       # Degrees to lift claw arm back over catapult
CataWindTime = 2.0             # Seconds to rewind/prime catapult


# ==============================================================================
# HELPER ACTIONS
# ==============================================================================
def load_bag_to_catapult():
    """Performs the E-Up sequence: Grabs bag, lifts it, dumps into catapult basket."""
    # 1. Lower claw to ground
    claw.set_stopping(COAST)
    claw.spin_for(FORWARD, ClawAboveGroundDeg, DEGREES, wait=True)
    
    # 2. Drive into bag
    drivetrain.set_drive_timeout(1, SECONDS)
    drivetrain.drive_for(FORWARD, MoveBeforePickmm, MM, wait=True)
    wait(0.2, SECONDS)
    
    # 3. Grip bag with pneumatics
    claw_grab.extend(CYLINDER1)
    claw.set_velocity(100, PERCENT)
    wait(0.5, SECONDS)
    
    # 4. Lift arm backwards over catapult
    claw.spin_for(REVERSE, ClawMotorMoveDeg, DEGREES, wait=True)
    
    # 5. Release bag into catapult bucket
    claw_grab.retract(CYLINDER1)
    wait(0.2, SECONDS)
    
    # 6. Return claw arm back to resting position
    claw.set_velocity(30, PERCENT)
    claw.spin_for(FORWARD, (ClawMotorMoveDeg - ClawAboveGroundDeg), DEGREES, wait=True)
    claw.set_stopping(HOLD)


def fire_and_reload_catapult():
    """Performs the L-Up sequence: Fires catapult forward, rewinds and locks in hold."""
    # 1. Launch forward for 1 second
    cata.spin(FORWARD)
    wait(1.0, SECONDS)
    
    # 2. Rewind in reverse for CataWindTime
    cata.spin(REVERSE)
    wait(CataWindTime, SECONDS)
    
    # 3. Stop and lock holding tension
    cata.stop()


# ==============================================================================
# MAIN AUTONOMOUS ROUTINE
# ==============================================================================
def autonomous():
    # --- 1. Calibrations & Motor Setup ---
    inertial.calibrate()
    while inertial.is_calibrating():
        wait(50, MSEC)
        
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    drivetrain.set_stopping(BRAKE)
    drivetrain.set_drive_timeout(3, SECONDS)
    
    cata.set_max_torque(100, PERCENT)
    cata.set_velocity(100, PERCENT)
    cata.set_stopping(HOLD)
    
    claw.set_max_torque(50, PERCENT)
    claw.set_velocity(100, PERCENT)
    claw.set_stopping(HOLD)
    
    claw_grab.pump_on()
    
    # --- 2. Homing Mechanism ---
    claw_grab.retract(CYLINDER1)
    claw.spin_for(REVERSE, 30, DEGREES, wait=True)
    
    # Prime catapult
    cata.spin(REVERSE)
    wait(CataWindTime, SECONDS)
    cata.stop()
    
    # --- 3. Start Autonomous Timer ---
    brain.timer.reset()
    
    # --- 4. Autonomous Bag Cycle Loop (Repeats for AutonDurationSec) ---
    while brain.timer.time(SECONDS) < AutonDurationSec:
        # Step 1: Pick up bag and load onto catapult (E-up)
        load_bag_to_catapult()
        
        # Step 2: Drive forward X mm
        drivetrain.drive_for(FORWARD, DriveForwardDistMM, MM, wait=True)
        
        # Step 3: Turn Y degrees right and launch bag (L-up)
        drivetrain.turn_for(RIGHT, TurnAngleDeg, DEGREES, wait=True)
        fire_and_reload_catapult()
        
        # Step 4: Straighten itself and reverse X mm back to loading zone
        drivetrain.turn_for(LEFT, TurnAngleDeg, DEGREES, wait=True)
        drivetrain.drive_for(REVERSE, DriveForwardDistMM, MM, wait=True)
        wait(0.2, SECONDS)
        
    # --- 5. Clean Shutdown when Timer Expires ---
    drivetrain.stop()
    cata.stop()
    claw.stop()
    claw_grab.pump_off()


# Run autonomous routine on start
autonomous()
