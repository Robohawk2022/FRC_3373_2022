// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.eyes.EyesSubsystem;
import frc.robot.specialops.ClimberSubsystem;
import frc.robot.specialops.ShooterSubsystem;
import frc.robot.specialops.SpecialOpsController;

/**
 * Implementation of a Robot that only has the eyes subsystem, so we can develop
 * and test independently.
 */
public class RobotEyesOnly extends TimedRobot {
  
  private SpecialOpsController specialOpsController;
  private EyesSubsystem eyes;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    specialOpsController = new SpecialOpsController(RobotPortMap.SPECIALOPS_CONTROLLER_PORT);
    eyes = new EyesSubsystem(specialOpsController);
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    eyes.updateDashboard();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    eyes.updateTeleop();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    eyes.disable();
  }
}