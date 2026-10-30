# AI Agent Guidelines - VEX IQ Team 48F Repository

## Mandatory Documentation Synchronization Rule

Whenever **any** of the robot programs in this repository are touched, modified, or created (including but not limited to):
- `src/DrivingSkills.iqblocks`
- `src/Auton-48F-WithInertial.iqblocks`
- `src/Auton-48F.iqblocks`
- `src/Auton-48F.py`
- Any future `.iqblocks` or `.py` files

**You MUST immediately update `manual.html` to stay in sync with the code changes.**

### What Must Be Updated in `manual.html`:
1. **Controller Buttons & Controls**: If any button mapping, manual override, macro, or drive mode is added or changed, update the controller visual and the button mapping table.
2. **Parameters & Default Values**: If any configurable variable is added, removed, or renamed, or if default values/units are changed, update the "Configurable Parameters & Timing Guide" table.
3. **Autonomous Sequences**: If the sequence of steps, heading logic, or function block behaviors are altered, update the autonomous section accordingly.
4. **Hardware & Subsystems**: If port numbers, motor directions, gear ratios, or pneumatic cylinders are modified, update the hardware mapping table.

### Guidelines for Editing VEXcode `.iqblocks` Files:
- Always preserve valid JSON formatting and Blockly XML syntax.
- Ensure all block types and field names match VEXcode IQ's registered block schema (`TURNDIRECTION` for turns, `iq_motion_stop_driving` for drivetrain stop, `iq_sensing_reset_timer` for timer reset, etc.).
- Avoid syntax or closing tag errors by programmatically validating XML where necessary.
