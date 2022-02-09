// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testrobots;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.SpecialOpsController;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Implementation of a Robot that only has the shooting subsystem, so we can develop
 * and test independently.
 */
public class RobotShooterOnly extends TimedRobot {
  
  private SpecialOpsController specialOpsController;
  private ShooterSubsystem shooter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    specialOpsController = new SpecialOpsController(RobotPortMap.SPECIALOPS_CONTROLLER_PORT);
    shooter = new ShooterSubsystem(specialOpsController);
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    shooter.updateDashboard();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    shooter.updateTeleop();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    shooter.disable();
  }
}