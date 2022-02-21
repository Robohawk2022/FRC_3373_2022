
  
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.EyesSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private XboxController specialops;
  private EyesSubsystem eyes;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ClimberSubsystem climber;
  private TeleopMode teleopMode;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    specialops = new XboxController(1);
    eyes = new EyesSubsystem();
    intake = new IntakeSubsystem(specialops);
    shooter = new ShooterSubsystem(specialops);
    climber = new ClimberSubsystem(specialops);

    // // swerve controls
    // swerve = SwerveControl.getInstance();
    // swerve.setDriveSpeed(0.25);
    // swerve.changeControllerLimiter(3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    eyes.robotPeriodic();
    intake.robotPeriodic();
    shooter.robotPeriodic();
    climber.robotPeriodic();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** Called to change modes during teleop */
  private void setTeleopMode(TeleopMode newMode) {
    eyes.teleopInit(newMode);
    intake.teleopInit(newMode);
    shooter.teleopInit(newMode);
    climber.teleopInit(newMode);
    teleopMode = newMode;    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {    
    // swerve.setControlMode(DriveMode.ROBOTCENTRIC);
    // swerve.recalculateWheelPosition();
    setTeleopMode(TeleopMode.INTAKE);
  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.updateValues();

    swerveControls();

    switch (teleopMode) {
      case INTAKE:
        intake.telopPeriodic();
        break;
      case SHOOT:
        shooter.teleopPeriodic();
        break;
      case CLIMB:
        climber.teleopPeriodic();
        break;
    }
  }

  private void swerveControls() {
    // /*
    //  * ######################## Driver Controls ########################
    //  */

    // swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4) * 0.75);


    // if (driver.isBPushed()) {
    //     swerve.changeControllerLimiter();
    // }

    // if (driver.isLBHeld()) {
    //     swerve.setDriveSpeed(0.45);
    // } else if (driver.isRBHeld()) {
    //     swerve.setDriveSpeed(.75);
    // } else {
    //     swerve.setDriveSpeed(0.15);
    // }

    /*
     * if(driver.isStartPushed()){ swerve.calibrateHome(); }
     */

    // if (driver.isYPushed()) {
    //     swerve.recalculateWheelPosition();
    // }

    // if (driver.getRawAxis(2) > 0.8)
    //     swerve.setControlMode(DriveMode.FIELDCENTRIC);
    // else if (driver.getRawAxis(3) > 0.8)
    //     swerve.setControlMode(DriveMode.ROBOTCENTRIC);

    // switch (driver.getPOV()) {
    // case 0:
    //     swerve.changeFront(SwerveControl.Side.NORTH);
    //     break;
    // case 90:
    //     swerve.changeFront(SwerveControl.Side.EAST);
    //     break;
    // case 180:
    //     swerve.changeFront(SwerveControl.Side.SOUTH);
    //     break;
    // case 270:
    //     swerve.changeFront(SwerveControl.Side.WEST);
    //     break;
    // }

    // if (driver.isBackPushed()) {
    //     swerve.resetOrentation();
    // }
  }

  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    intake.disabledInit();
    shooter.disabledInit();
    climber.disabledInit();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /** 
    String robotMode = "Debug"; 
    SmartDashboard.putString("Robot Mode", robotMode);
    SmartDashboard.updateValues();
    if (driver.get()) {
  }
*/

    }

}