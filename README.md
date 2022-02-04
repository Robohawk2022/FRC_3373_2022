# Robohawk2022/FRC_3373_2022

## To-Do List

- Ask the build team about the drive chassis. Some specific questions:
     - What is the size of the wheels this year (circumference/diameter)?
     - What is the size of the chassis (width/height)?

- Ask the build team about the shooter. Some specific questions:
     - How many wheels are there (we think 3 - intake, indexer and launch)?
     - What type of wheels are the indexer and intake? Are they brushless? How should they rotate?
     - How many sensors are there (we think 2 - one for intake and one for shoot)?

- Ask the build team about the climber. What will we need?

- Depending on the type of motor for the shooter wheels, hook one of them up to
the test bench and mess around with API

- Hook up a swerve wheel to the test bench and experiment with calibration of 
the position encoder

## Bonus Points

- Shooter
     - Implement a two-level launch control (low speed and high speed)

- Driver
     - Implement a "test mode" for the calibration

- General
     - Figure out how to read data from the dashboard

## Previous TODO list

- Drive/Swerve 
- Intake
- Shoot 
     - Verify system choice with build team
     - Start working with senors 
     - Create Shooter.java file 
          - Create class to be called in Robot.java 
     - Specifications:
          - Large Main Wheel
          - Limit Switch
          - Small Indexer Wheel    
- Climb
     - Nothing to much now
- Eyes
     - Fix boxing around vision tape
     - Framerate
- Navx
     - Course Correct  

# Resources

## General Java

* Online IDE/Compiler for Testing - [https://www.online-java.com/]
* Visual Studio Code (aka VSCode) for Java - [https://code.visualstudio.com/docs/languages/java]

## Robotics-related

* [Install instructions for WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
     * [Install instructions for REV Robotics](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#labview)
     * [Install instructions for navX](https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/)
* [Install instructions for game tools (including) Dashboard](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html#installing-the-frc-game-tools)
* [Buy your own roboRIO](https://www.ni.com/en-us/support/model.roborio.html) (just kidding, you do NOT need to do this)

