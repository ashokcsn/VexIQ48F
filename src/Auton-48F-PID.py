# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Project:        Auton-48F-PID                                              #
# 	Module:         Auton-48F-PID.py                                           #
# 	Author:         VEX IQ Team 48F                                            #
# 	Created:        2026-10-30                                                 #
# 	Description:    Autonomous Bag Cycle Routine with PID drive + PID turns    #
#                   (same sequence as Auton-48F.iqblocks)                      #
#                                                                              #
# ---------------------------------------------------------------------------- #
#
# Why PID?
#   The stock Drivetrain "drive for" block only counts motor degrees and never
#   steers, so any difference between the left and right side (friction, gear
#   mesh, battery, carpet) makes the robot curve. This program drives the two
#   drive motors directly and runs two controllers at the same time:
#     1. Distance PID : left/right motor encoders -> forward speed
#     2. Heading P    : brain inertial sensor     -> steering correction
#   Turns use a separate PID on the inertial sensor.
#
# Paste this file into a new VEXcode IQ Python project (IQ 2nd gen brain), or
# open Auton-48F-PID.iqpython, which contains the same code.

from vex import *

# ==============================================================================
# DEVICE CONFIGURATION (matches Auton-48F.iqblocks)
# ==============================================================================
brain = Brain()
brain_inertial = Inertial()          # Inertial sensor built into the IQ 2nd gen brain

left_motor = Motor(Ports.PORT1, False)
right_motor = Motor(Ports.PORT2, True)

claw_grab = Pneumatic(Ports.PORT7)

cata_motor_a = Motor(Ports.PORT5, True)
cata_motor_b = Motor(Ports.PORT6, False)
cata = MotorGroup(cata_motor_a, cata_motor_b)

claw_motor_a = Motor(Ports.PORT3, True)
claw_motor_b = Motor(Ports.PORT4, False)
claw = MotorGroup(claw_motor_a, claw_motor_b)

# ==============================================================================
# DRIVE GEOMETRY / CALIBRATION
# ==============================================================================
WheelTravelMM = 200.0      # Distance the wheel rolls in one wheel turn (200 mm wheel)
DriveGearRatio = 2.5       # Wheel turns per ONE motor turn (2:5 gearing, geared for speed)
DriveCalibration = 1.0     # Fine tune: (distance asked) / (distance measured). See manual.

# ==============================================================================
# CONFIGURABLE PARAMETERS
# ==============================================================================
# Match & Drive Geometry
AutonDurationSec = 60.0
DriveDistIn = 24.0           # Inches driven back/forth between loading zone and goal
TurnAngleDeg = 5.0           # Degrees turned left to aim the catapult
MoveBeforePickIn = 1.0       # Inches nudged forward into the bag before clamping
MoveBeforePickTimeoutSec = 0.4
ClawAboveGroundDeg = 20.0
ClawMotorMoveDeg = 140.0

# Configurable Cycle Timing
CataThrowTimeSec = 1.0         # Seconds catapult spins forward to launch
CataWindTime = 2.0             # Seconds catapult winds reverse to reload
WaitBeforeClampSec = 0.2       # Settle wait after driving into bag before clamping
WaitClampSettleSec = 0.5       # Grip settle wait before lifting
WaitDropSettleSec = 0.2        # Drop settle wait before returning arm
WaitAtLoadingZoneSec = 0.2     # Settle wait after reversing back to loading zone

# Drive PID (error in inches -> motor speed in %)
DriveKp = 6.0
DriveKi = 0.0
DriveKd = 0.5
DriveMaxSpeedPct = 80.0        # Top speed during PID drives
DriveMinSpeedPct = 8.0         # Minimum push so the robot does not stall near the target
DriveSlewPctPerLoop = 6.0      # Max speed increase per 10 ms (soft start, no wheel slip)
DriveToleranceIn = 0.25        # "Close enough" distance
DriveIntegralZoneIn = 3.0      # Integral only builds up inside this distance
DriveTimeoutSec = 3.0

# Heading hold while driving (error in degrees -> steering in %)
HeadingKp = 2.0
HeadingKd = 0.0
HeadingMaxCorrectionPct = 30.0

# Turn PID (error in degrees -> motor speed in %)
TurnKp = 1.2
TurnKi = 0.0
TurnKd = 0.08
TurnMaxSpeedPct = 60.0
TurnMinSpeedPct = 6.0
TurnToleranceDeg = 1.0
TurnIntegralZoneDeg = 10.0
TurnTimeoutSec = 1.5

PidSettleSec = 0.1             # Must stay inside tolerance this long to finish
PidLoopMs = 10

# Heading (in inertial "rotation" degrees, which never wraps at 360) that every
# drive tries to hold. Turns update it, so drift never accumulates.
target_rotation = 0.0


# ==============================================================================
# PID HELPERS
# ==============================================================================
def clamp(value, low, high):
    return max(low, min(high, value))


def inches_per_motor_degree():
    mm_per_motor_turn = WheelTravelMM * DriveGearRatio * DriveCalibration
    return mm_per_motor_turn / 25.4 / 360.0


def drive_position_in():
    avg_deg = (left_motor.position(DEGREES) + right_motor.position(DEGREES)) / 2.0
    return avg_deg * inches_per_motor_degree()


def stop_drive():
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)


def drive_pid(distance_in, timeout_sec=None, max_speed=None):
    """Drive straight distance_in inches (negative = reverse) holding target_rotation."""
    if timeout_sec is None:
        timeout_sec = DriveTimeoutSec
    if max_speed is None:
        max_speed = DriveMaxSpeedPct

    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)

    dt = PidLoopMs / 1000.0
    integral = 0.0
    prev_error = distance_in
    prev_heading_error = 0.0
    last_speed = 0.0
    settled = 0.0
    start = brain.timer.time(SECONDS)

    while brain.timer.time(SECONDS) - start < timeout_sec:
        # ---- distance PID ----
        error = distance_in - drive_position_in()
        if abs(error) < DriveIntegralZoneIn:
            integral += error * dt
        else:
            integral = 0.0
        derivative = (error - prev_error) / dt
        prev_error = error

        speed = DriveKp * error + DriveKi * integral + DriveKd * derivative
        speed = clamp(speed, -max_speed, max_speed)
        if abs(error) > DriveToleranceIn and abs(speed) < DriveMinSpeedPct:
            speed = DriveMinSpeedPct if error > 0 else -DriveMinSpeedPct
        # slew limit: only limit speeding UP, braking is allowed instantly
        if abs(speed) > abs(last_speed) + DriveSlewPctPerLoop:
            speed = (abs(last_speed) + DriveSlewPctPerLoop) * (1 if speed > 0 else -1)
        last_speed = speed

        # ---- heading hold ----
        heading_error = target_rotation - brain_inertial.rotation(DEGREES)
        correction = HeadingKp * heading_error + HeadingKd * (heading_error - prev_heading_error) / dt
        prev_heading_error = heading_error
        correction = clamp(correction, -HeadingMaxCorrectionPct, HeadingMaxCorrectionPct)

        left_motor.spin(FORWARD, speed + correction, PERCENT)
        right_motor.spin(FORWARD, speed - correction, PERCENT)

        # ---- finished? ----
        if abs(error) < DriveToleranceIn:
            settled += dt
            if settled >= PidSettleSec:
                break
        else:
            settled = 0.0
        wait(PidLoopMs, MSEC)

    stop_drive()


def turn_to_rotation_pid(target_deg):
    """Turn to an absolute inertial rotation (degrees, + = clockwise/right)."""
    global target_rotation
    target_rotation = target_deg

    dt = PidLoopMs / 1000.0
    integral = 0.0
    prev_error = target_deg - brain_inertial.rotation(DEGREES)
    settled = 0.0
    start = brain.timer.time(SECONDS)

    while brain.timer.time(SECONDS) - start < TurnTimeoutSec:
        error = target_deg - brain_inertial.rotation(DEGREES)
        if abs(error) < TurnIntegralZoneDeg:
            integral += error * dt
        else:
            integral = 0.0
        derivative = (error - prev_error) / dt
        prev_error = error

        speed = TurnKp * error + TurnKi * integral + TurnKd * derivative
        speed = clamp(speed, -TurnMaxSpeedPct, TurnMaxSpeedPct)
        if abs(error) > TurnToleranceDeg and abs(speed) < TurnMinSpeedPct:
            speed = TurnMinSpeedPct if error > 0 else -TurnMinSpeedPct

        left_motor.spin(FORWARD, speed, PERCENT)
        right_motor.spin(FORWARD, -speed, PERCENT)

        if abs(error) < TurnToleranceDeg:
            settled += dt
            if settled >= PidSettleSec:
                break
        else:
            settled = 0.0
        wait(PidLoopMs, MSEC)

    stop_drive()


def turn_left_pid(deg):
    turn_to_rotation_pid(target_rotation - deg)


def turn_right_pid(deg):
    turn_to_rotation_pid(target_rotation + deg)


# ==============================================================================
# HELPER ACTIONS (same as the Blocks custom blocks)
# ==============================================================================
def pickAndLoadBag():
    """E-Up sequence: grab bag, lift it, dump into catapult basket."""
    claw.set_stopping(COAST)
    claw.spin_for(FORWARD, ClawAboveGroundDeg, DEGREES, wait=True)
    drive_pid(MoveBeforePickIn, MoveBeforePickTimeoutSec)
    wait(WaitBeforeClampSec, SECONDS)

    claw_grab.extend(CYLINDER1)
    claw.set_velocity(100, PERCENT)
    wait(WaitClampSettleSec, SECONDS)

    claw.spin_for(REVERSE, ClawMotorMoveDeg, DEGREES, wait=True)
    claw_grab.retract(CYLINDER1)
    drive_pid(-MoveBeforePickIn, MoveBeforePickTimeoutSec)
    wait(WaitDropSettleSec, SECONDS)

    claw.set_velocity(30, PERCENT)
    claw.spin_for(FORWARD, (ClawMotorMoveDeg - ClawAboveGroundDeg), DEGREES, wait=True)
    claw.set_stopping(HOLD)


def fireAndReloadCatapult():
    """L-Up sequence: fire catapult forward, rewind and hold."""
    cata.spin(FORWARD)
    wait(CataThrowTimeSec, SECONDS)
    cata.spin(REVERSE)
    wait(CataWindTime, SECONDS)
    cata.stop()


# ==============================================================================
# MAIN AUTONOMOUS ROUTINE
# ==============================================================================
def autonomous():
    global target_rotation

    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
        wait(50, MSEC)
    brain_inertial.set_rotation(0, DEGREES)
    target_rotation = 0.0

    left_motor.set_stopping(BRAKE)
    right_motor.set_stopping(BRAKE)
    left_motor.set_max_torque(100, PERCENT)
    right_motor.set_max_torque(100, PERCENT)

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
    # NOTE: drive_pid/turn PID read brain.timer relative to their own start
    # time, so they do not disturb the match timer.

    while brain.timer.time(SECONDS) < AutonDurationSec:
        pickAndLoadBag()
        drive_pid(-DriveDistIn)          # reverse to the goal
        turn_left_pid(TurnAngleDeg)      # aim
        fireAndReloadCatapult()
        turn_right_pid(TurnAngleDeg)     # back to the original heading
        drive_pid(DriveDistIn)           # return to the loading zone
        wait(WaitAtLoadingZoneSec, SECONDS)

    stop_drive()
    cata.stop()
    claw.stop()
    claw_grab.pump_off()


autonomous()
