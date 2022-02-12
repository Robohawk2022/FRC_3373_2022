// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testrobots;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Implementation of a Robot that only has the intake subsystem, so we can develop
 * and test independently.
 */
public class RobotIntakeOnly extends TimedRobot {
  
  private SpecialOpsController specialOpsController;
  private IntakeSubsystem intake;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    specialOpsController = new SpecialOpsController(RobotPortMap.SPECIALOPS_CONTROLLER_PORT);
    intake = new IntakeSubsystem(specialOpsController);
    intake.teleopInit(TeleopMode.INTAKE);
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    intake.robotPeriodic();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    intake.telopPeriodic();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    intake.disabledInit();
  }
}