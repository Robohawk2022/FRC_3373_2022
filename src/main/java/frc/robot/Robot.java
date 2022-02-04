
  
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.SwerveControl.DriveMode;
import frc.robot.eyes.EyesSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

// talons
//NOTE: not neccesary unless called in robot.java file
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static double shooter_max_speed = .1;
  private static double TotalBalls = 0;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // declare driver and shooter joysticks
  private SuperJoystick driver;
  private SuperJoystick specialops;
  // declare navx for use during autonomous mode
  private SuperAHRS navx_old;
  // delcare shooting motor
  private CANSparkMax LargeMainWheel;
  private CANSparkMax SmallIndexerWheel;
  private CANSparkMax Intake;
  private DigitalInput indexer;
  private AHRS navx;
  private SwerveControl swerve;
  //end
  private EyesSubsystem eyes;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    LargeMainWheel = new CANSparkMax(1, MotorType.kBrushless);
    SmallIndexerWheel = new CANSparkMax(2, MotorType.kBrushless);
    Intake = new CANSparkMax(3, MotorType.kBrushless);
    driver = new SuperJoystick(0);
    // the two joysticks for both driers will be called speci
    specialops = new SuperJoystick(1);
    indexer = new DigitalInput(0);
    navx = new AHRS(SerialPort.Port.kUSB1);
    // swerve controls
    swerve = SwerveControl.getInstance();
    swerve.setDriveSpeed(0.25);
    swerve.changeControllerLimiter(3);
    //RobohawkVision 2.0
    eyes = new EyesSubsystem();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
    SmartDashboard.putNumber("Total Balls", TotalBalls);
    SmartDashboard.updateValues();
    swerve.setControlMode(DriveMode.ROBOTCENTRIC);
    swerve.recalculateWheelPosition();


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    LargeMainWheel.set(0);
    SmallIndexerWheel.set(0);
    Intake.set(0);
    SmartDashboard.updateValues();
    eyes.setDriveView();
    joystickControls();
    //start shooting wheel
    while (specialops.getTrigger() == true) {
      LargeMainWheel.set(shooter_max_speed);
      eyes.setShootView();
    }

    while (specialops.isStartPushed()) {
      shooter_max_speed += .1;
    }
    //lower shooting wheel speed
    while (specialops.isBackPushed()) {
      shooter_max_speed -= .1;
    }
    //increase shooting wheel speed
    while (indexer.get()) {
      TotalBalls += 1;
    }
    //shoot
    // WORK ON: don't shoot until motor is at full speed
    while (specialops.isAPushed()) {
      Timer indexDelay = new Timer();
      indexDelay.reset();
      indexDelay.start();
      if (indexDelay.get() < 1.1) {
        SmallIndexerWheel.set(0.5);
        TotalBalls -= 1;
      }
      indexDelay.stop();
    }
    // intake
    while (specialops.getRawAxis(5) > 0.05) {     
      Timer accelerationDelay = new Timer();
      accelerationDelay.reset();
      accelerationDelay.start();
      if (accelerationDelay.get() < 1) {
        Intake.set(.1);
      }
      Intake.set(.2);
      accelerationDelay.stop();
    }
    /*
    A problem with the 2020 indexer code was the sensors could be accidentally tripped or think that
    the robot has more/less balls then it actually does. This allows for the ball number displayed in
    code to be reset along with the value send to smart dashboard. 
    */
    while (driver.isBPushed()) {
      TotalBalls = 0;
    }
  }
  private void joystickControls() {
    /*
     * ######################## Driver Controls ########################
     */

    swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4) * 0.75);

    if (driver.isBPushed()) {
        swerve.changeControllerLimiter();
    }

    if (driver.isLBHeld()) {
        swerve.setDriveSpeed(0.45);
    } else if (driver.isRBHeld()) {
        swerve.setDriveSpeed(.75);
    } else {
        swerve.setDriveSpeed(0.15);
    }

    /*
     * if(driver.isStartPushed()){ swerve.calibrateHome(); }
     */

    if (driver.isYPushed()) {
        swerve.recalculateWheelPosition();
    }

    if (driver.getRawAxis(2) > 0.8)
        swerve.setControlMode(DriveMode.FIELDCENTRIC);
    else if (driver.getRawAxis(3) > 0.8)
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);

    switch (driver.getPOV()) {
    case 0:
        swerve.changeFront(SwerveControl.Side.NORTH);
        break;
    case 90:
        swerve.changeFront(SwerveControl.Side.EAST);
        break;
    case 180:
        swerve.changeFront(SwerveControl.Side.SOUTH);
        break;
    case 270:
        swerve.changeFront(SwerveControl.Side.WEST);
        break;
    }

    if (driver.isBackPushed()) {
        swerve.resetOrentation();
    }
  }

  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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