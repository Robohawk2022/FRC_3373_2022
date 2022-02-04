# Special Ops

This system handles everything that's not driving:
- Ball intake
- Shooting
- Climbing

# Robot Architecture

- Three sets of components
    - Intake
        - Switch to detect a ball waiting to be picked up
        - Wheel to accomplish picking up a ball
    - Shooting
        - Switch to detect a ball waiting to be shot
        - Main "launch wheel"
        - Indexer to push a ball towards the launch wheel
    - Climbing
        - TODO

# TODO

- Intake system
    - What kind of motor do we have?
    - How do we know when we have too many balls in the hopper already?
    - Implement and test operation in lab

- Shooting
    - Do we have a switch for "ball waiting"?
    - Implement and test operation in lab

- Climbing
    - What is the architecture of the robot
    - Implement "mode switch" between shooting and climbing


