// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimberSubsystem;
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
  // public static final int CLIMBER_EXTENDER_PORT = 1;
  // public static final int CLIMBER_ROTATOR_PORT = 1;
  // public static final int CLIMBER_EXTENDER_SWITCH = 1;
  // public static final int CLIMBER_ROTATOR_SWITCH = 1;

  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;

  private XboxController specialops;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ClimberSubsystem climber;
  private static final int  FLangleID = 8;
  private static final int FLdriveID = 7;
  private static final int  FRangleID = 6;
  private static final int FRdriveID = 5;
  private static final int  BRangleID = 4;
  private static final int BRdriveID = 3;
  private static final int  BLangleID = 2;
  private static final int BLdriveID = 1;
  private CANSparkMax FLangleMotor;
  private CANSparkMax FLdriveMotor;
  private CANSparkMax FRdriveMotor;
  private CANSparkMax FRangleMotor;
  private CANSparkMax BLangleMotor;
  private CANSparkMax BLdriveMotor;
  private CANSparkMax BRdriveMotor;
  private CANSparkMax BRangleMotor;
  private SparkMaxPIDController m_PIDController1;
  private SparkMaxPIDController m_PIDController2;
  private SparkMaxPIDController m_PIDController3;
  private SparkMaxPIDController m_PIDController4;
  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;
  private RelativeEncoder m_encoder3;
  private RelativeEncoder m_encoder4;
  private XboxController drive_control;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    drive_control = new XboxController(DRIVER_PORT);
    
    climber = null; // new ClimberSubsystem(driver, CLIMBER_EXTENDER_PORT, CLIMBER_EXTENDER_SWITCH, CLIMBER_ROTATOR_PORT, CLIMBER_ROTATOR_SWITCH);

    specialops = new XboxController(SPECIAL_OPS_PORT);
    intake = new IntakeSubsystem(specialops, INTAKE_PORT);
    shooter = new ShooterSubsystem(specialops, SHOOTER_LAUNCH_PORT, SHOOTER_INDEXER_PORT, SHOOTER_SWITCH_PORT);

    FLangleMotor = new CANSparkMax(FLangleID, MotorType.kBrushless);
    FLdriveMotor = new CANSparkMax(FLdriveID, MotorType.kBrushless);
    FRangleMotor = new CANSparkMax(FRangleID, MotorType.kBrushless);
    FRdriveMotor = new CANSparkMax(FRdriveID, MotorType.kBrushless);
    BRangleMotor = new CANSparkMax(BRangleID, MotorType.kBrushless);
    BRdriveMotor = new CANSparkMax(BRdriveID, MotorType.kBrushless);
    BLdriveMotor = new CANSparkMax(BLdriveID, MotorType.kBrushless);
    BLangleMotor = new CANSparkMax(BLangleID, MotorType.kBrushless);

    m_PIDController1 = FLangleMotor.getPIDController();
    m_PIDController2 = FRangleMotor.getPIDController();
    m_PIDController3 = BRangleMotor.getPIDController();
    m_PIDController4 = BLangleMotor.getPIDController();

    m_encoder1 = FLangleMotor.getEncoder();
    m_encoder2 = FRangleMotor.getEncoder();
    m_encoder3 = BRangleMotor.getEncoder();
    m_encoder4 = BLangleMotor.getEncoder();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_PIDController1.setP(kP);
    m_PIDController2.setP(kP);
    m_PIDController3.setP(kP);
    m_PIDController4.setP(kP);

    m_PIDController1.setI(kI);
    m_PIDController2.setI(kI);
    m_PIDController3.setI(kI);
    m_PIDController4.setI(kI);

    m_PIDController1.setD(kD);
    m_PIDController2.setD(kD);
    m_PIDController3.setD(kD);
    m_PIDController4.setD(kD);

    m_PIDController1.setIZone(kIz);
    m_PIDController2.setIZone(kIz);
    m_PIDController3.setIZone(kIz);
    m_PIDController4.setIZone(kIz);

    m_PIDController1.setFF(kFF);
    m_PIDController2.setFF(kFF);
    m_PIDController3.setFF(kFF);
    m_PIDController4.setFF(kFF);

    m_PIDController1.setOutputRange(kMinOutput, kMaxOutput);
    m_PIDController2.setOutputRange(kMinOutput, kMaxOutput);
    m_PIDController3.setOutputRange(kMinOutput, kMaxOutput);
    m_PIDController4.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Run Speed Motor?", 0);
    SmartDashboard.putBoolean("Joystick Control", false);

    if (!isSimulation()) {
      CameraServer.startAutomaticCapture("Front", FRONT_CAMERA_PORT);
      CameraServer.startAutomaticCapture("Back", BACK_CAMERA_PORT);  
    }
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
    if (intake != null) {
      intake.robotPeriodic();
    }
    if (shooter != null) {
      shooter.robotPeriodic();
    }
    if (climber != null) {
      climber.robotPeriodic();
    }
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
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    SmartDashboard.updateValues();
    if (intake != null) {
      intake.telopPeriodic();
    }
    if (shooter != null) {
      shooter.teleopPeriodic();      
    }
    shooter.teleopPeriodic();
    if (climber != null) {
      climber.teleopPeriodic();
    }

    if(drive_control.getLeftY() > 0) {
      FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
      FRdriveMotor.set((drive_control.getLeftY() * -1) / 4);
      BRdriveMotor.set(drive_control.getLeftY() / 4);
      BLdriveMotor.set(drive_control.getLeftY() / 4);

      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);            
    }
    if(drive_control.getLeftY() < 0) {
      FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
      FRdriveMotor.set((drive_control.getLeftY() * -1) / 4);
      BRdriveMotor.set(drive_control.getLeftY() / 4);
      BLdriveMotor.set(drive_control.getLeftY() / 4);

      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);   

    }
    if(drive_control.getRightX() > .0) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(.25);
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(-1 * .25);
      m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(-1 * .25);
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(.25);
    }
      // Rotation
    if(drive_control.getRightX() < 0) {
      m_PIDController1.setReference(-1 * 2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(-1 * .25);
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(.25);
      m_PIDController3.setReference(-1 * 2,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(.25);
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(-1 * .25);    
    }

    AimBot();
    StrafeSwerve();
  }
  public void StrafeSwerve() {
    if(drive_control.getRawAxis(0) == 1) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(- .25);
      m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(- .25);
      m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(.25);
      m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(.25);
    }
    if(drive_control.getRawAxis(0) == -1) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(.25);
      m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(.25);
      m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(- .25);
      m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(- .25);
    }    
  }

  public void AimBot() {
    if(drive_control.getLeftBumper() == true) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(drive_control.getRightX() / 15);
      FRdriveMotor.set((drive_control.getRightX() * -1) / 15);
      BRdriveMotor.set(drive_control.getRightX() / 15);
      BLdriveMotor.set((drive_control.getRightX() * -1) / 15);
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
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}