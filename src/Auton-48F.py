# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Project:        Auton-48F                                                  #
# 	Module:         Auton-48F.py                                               #
# 	Author:         VEX IQ Team 48F                                            #
# 	Created:        2026-10-29                                                 #
# 	Description:    Autonomous Bag Cycle Routine with Configurable Timers      #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

# ==============================================================================
# DEVICE CONFIGURATION
# ==============================================================================
brain = Brain()

claw_grab = Pneumatic(Ports.PORT7)
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_2_5, False)
right_motor = Motor(Ports.PORT2, GearSetting.RATIO_2_5, True)
inertial = Inertial(Ports.PORT0)
drivetrain = SmartDrive(left_motor, right_motor, inertial, 200, 190.5, 190.5, MM)

cata_motor_a = Motor(Ports.PORT5, GearSetting.RATIO_1_1, True)
cata_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_1_1, False)
cata = MotorGroup(cata_motor_a, cata_motor_b)

claw_motor_a = Motor(Ports.PORT3, GearSetting.RATIO_1_1, True)
claw_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_1_1, False)
claw = MotorGroup(claw_motor_a, claw_motor_b)

# ==============================================================================
# CONFIGURABLE PARAMETERS
# ==============================================================================
# Match & Drive Geometry
AutonDurationSec = 60.0
DriveForwardDistMM = 400.0
TurnAngleDeg = 45.0
MoveBeforePickmm = 100.0
ClawAboveGroundDeg = 20.0
ClawMotorMoveDeg = 140.0

# Configurable Cycle Timing
CataThrowTimeSec = 1.0         # Seconds catapult spins forward to launch
CataWindTime = 2.0             # Seconds catapult winds reverse to reload
WaitBeforeClampSec = 0.2       # Settle wait after driving into bag before clamping
WaitClampSettleSec = 0.5       # Grip settle wait before lifting
WaitDropSettleSec = 0.2        # Drop settle wait before returning arm
WaitAtLoadingZoneSec = 0.2     # Settle wait after reversing back to loading zone


# ==============================================================================
# HELPER ACTIONS (FUNCTION BLOCKS)
# ==============================================================================
def pickAndLoadBag():
    """Performs the E-Up sequence: Grabs bag, lifts it, dumps into catapult basket."""
    claw.set_stopping(COAST)
    claw.spin_for(FORWARD, ClawAboveGroundDeg, DEGREES, wait=True)
    drivetrain.drive_for(FORWARD, MoveBeforePickmm, MM, wait=True)
    wait(WaitBeforeClampSec, SECONDS)
    
    claw_grab.extend(CYLINDER1)
    claw.set_velocity(100, PERCENT)
    wait(WaitClampSettleSec, SECONDS)
    
    claw.spin_for(REVERSE, ClawMotorMoveDeg, DEGREES, wait=True)
    claw_grab.retract(CYLINDER1)
    wait(WaitDropSettleSec, SECONDS)
    
    claw.set_velocity(30, PERCENT)
    claw.spin_for(FORWARD, (ClawMotorMoveDeg - ClawAboveGroundDeg), DEGREES, wait=True)
    claw.set_stopping(HOLD)


def fireAndReloadCatapult():
    """Performs the L-Up sequence: Fires catapult forward, rewinds and locks in hold."""
    cata.spin(FORWARD)
    wait(CataThrowTimeSec, SECONDS)
    cata.spin(REVERSE)
    wait(CataWindTime, SECONDS)
    cata.stop()


# ==============================================================================
# MAIN AUTONOMOUS ROUTINE
# ==============================================================================
def autonomous():
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
    
    # Homing
    claw_grab.retract(CYLINDER1)
    claw.spin_for(REVERSE, 30, DEGREES, wait=True)
    cata.spin(REVERSE)
    wait(CataWindTime, SECONDS)
    cata.stop()
    
    brain.timer.reset()
    
    while brain.timer.time(SECONDS) < AutonDurationSec:
        pickAndLoadBag()
        drivetrain.drive_for(FORWARD, DriveForwardDistMM, MM, wait=True)
        drivetrain.turn_to_heading(TurnAngleDeg, DEGREES, wait=True)
        fireAndReloadCatapult()
        drivetrain.turn_to_heading(0, DEGREES, wait=True)
        drivetrain.drive_for(REVERSE, DriveForwardDistMM, MM, wait=True)
        wait(WaitAtLoadingZoneSec, SECONDS)
        
    drivetrain.stop()
    cata.stop()
    claw.stop()
    claw_grab.pump_off()


autonomous()
