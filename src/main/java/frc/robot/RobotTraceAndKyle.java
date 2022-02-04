// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.shooter.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class RobotTraceAndKyle extends TimedRobot {
  
  private ShooterSubsystem shooter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    shooter = new ShooterSubsystem();
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    shooter.updateDashboard();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    shooter.updateControls();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    shooter.disable();
  }
}