# Subsystems

These handle all the functions other than driving:
- Cameras
- Ball intake
- Shooting
- Climbing

# Robot Notes

- Cameras
    - Two (front/back)
    - Back is used during shoot mode only
    - Front is used at all other times

- Intake
    - Structure
        - intakeMotor: drives two wheels
    - Behavior
        - No need for variable speed
        - Needs to be able to reverse
        - Can be running whenever we're in "intake mode"

- Shooter
    - ballAvailableSwitch: limit switch triggered when a ball is available
    - indexerMotor: pulls balls towards the launch wheel
        - closed loop based on wheel angle
        - two motions: "lock in" ball and "eject" ball
    - launchMotor: shoots balls out
        - can be spinning whenever we're in "shoot mode"
        - can coast if we switch back to intake
        - must brake during climbing

- Climbing
    - pivotMotor: operates open/close pivot on fixed "hook arm"
        - closed loop based on arm angle
        - rotates back and forth through a fixed angles
    - extenderSwitch: limit switch indicating when "grabber" arm is down as far as possible
    - extenderMotor: operates chain drive on rotating "grabber arm"
        - closed loop based on extension distance
    - rotatorMotor: controls angle of "grabber arm"
        - closed loop based on angle of grabber arm

# Implementing a subsystem

- Controls
    - Add controls are in the SpecialOpsController
    - Give control methods names based on what they mean (not just "isStartButtonPressed") 

- Components
    - All port numbers are in the RobotPortMap

- Methods
    - updateDashboard should output any relevant information
    - updateTeleop is what gets called during teleop

- Testing
    - The base class declares an XBoxControllerSim so you can use it in tests

# TODO
