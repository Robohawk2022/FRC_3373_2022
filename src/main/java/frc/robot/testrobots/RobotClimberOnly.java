// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testrobots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.TeleopMode;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Implementation of a Robot that only has the climber subsystem, so we can develop
 * and test independently.
 */
public class RobotClimberOnly extends TimedRobot {
  
  private XboxController specialOpsController;
  private ClimberSubsystem climber;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    specialOpsController = new XboxController(RobotPortMap.SPECIALOPS_CONTROLLER_PORT);
    climber = new ClimberSubsystem(specialOpsController);
    climber.teleopInit(TeleopMode.CLIMB);
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    climber.robotPeriodic();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    climber.teleopPeriodic();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    climber.disabledInit();
  }
}