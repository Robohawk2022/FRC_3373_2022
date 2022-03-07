// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final int DRIVER_PORT = 0;
  public static final int SPECIAL_OPS_PORT = 1;
  public static final int INTAKE_PORT = 9;
  public static final int SHOOTER_LAUNCH_PORT = 10;
  public static final int SHOOTER_INDEXER_PORT = 11;
  public static final int SHOOTER_SWITCH_PORT = 3;
  public static final int CLIMBER_EXTENDER_PORT = 12;
  public static final int CLIMBER_ROTATOR_PORT = 13;
  public static final int CLIMBER_EXTENDER_SWITCH = 1;
  public static final int CLIMBER_ROTATOR_SWITCH = 2;

  public static final boolean USE_CAMERAS = false;
  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;

  private static final int  FLangleID = 8;
  private static final int FLdriveID = 7;
  private static final int  FRangleID = 6;
  private static final int FRdriveID = 5;
  private static final int  BRangleID = 4;
  private static final int BRdriveID = 3;
  private static final int  BLangleID = 2;
  private static final int BLdriveID = 1;

  private XboxController drive_control;

  private XboxController specialops;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ClimberSubsystem climber;
  private DriveSubsystem driver;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putString("MotorTesting: ", "None");

    specialops = new XboxController(SPECIAL_OPS_PORT);
    climber = new ClimberSubsystem(drive_control, CLIMBER_EXTENDER_PORT, CLIMBER_EXTENDER_SWITCH, CLIMBER_ROTATOR_PORT, CLIMBER_ROTATOR_SWITCH);
    shooter = new ShooterSubsystem(specialops, SHOOTER_LAUNCH_PORT, SHOOTER_INDEXER_PORT, SHOOTER_SWITCH_PORT);

    drive_control = new XboxController(DRIVER_PORT);
    driver = new DriveSubsystem(drive_control, FLangleID, FLdriveID, FRangleID, FRdriveID, BRangleID, BRdriveID, BLangleID, BLdriveID);
    intake = new IntakeSubsystem(specialops, INTAKE_PORT);

    if (USE_CAMERAS && !isSimulation()) {
      CameraServer.startAutomaticCapture("Front", FRONT_CAMERA_PORT);
      CameraServer.startAutomaticCapture("Back", BACK_CAMERA_PORT);  
    }
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    if (intake != null) {
      intake.robotPeriodic();
    }
    if (shooter != null) {
      shooter.robotPeriodic();
    }
    if (climber != null) {
      climber.robotPeriodic();
    }
    if (driver != null) {
      driver.robotPeriodic();
    }
  }

  /** This function is called once when the robot is put in autonomous mode. */
  @Override
  public void autonomousInit() {
    if (intake != null) {
      intake.autonomousInit();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (intake != null) {
      intake.autonomousPeriodic();
    }
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.updateValues();
    if (intake != null) {
      intake.telopPeriodic();
    }
    if (shooter != null) {
      shooter.teleopPeriodic();      
    }
    if (climber != null) {
      climber.teleopPeriodic();
    }
    if (driver != null) {
      driver.teleopPeriodic();
    }
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (intake != null) {
      intake.disabledInit();
    }
    if (shooter != null) {
      shooter.disabledInit();
    }
    if (climber != null) {
      climber.disabledInit();
    }
    if (driver != null) {
      driver.disabledInit();
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    if (driver != null) {
      driver.testInit();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (driver != null) {
      driver.testPeriodic();
    }
  }
}