// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.Timer;

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

  // SWERVE CONSTANTS
  public static int DefaultLimit = 4;
  public static double RotationLimit = .25;
  public static double StrafeLimit = .25;

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
  private Timer autotimer;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;




  public Timer autoTimer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putString("MotorTesting: ", "None");
    drive_control = new XboxController(DRIVER_PORT);
    intake = new IntakeSubsystem(drive_control, INTAKE_PORT);
    
    specialops = new XboxController(SPECIAL_OPS_PORT);
    shooter = new ShooterSubsystem(specialops, SHOOTER_LAUNCH_PORT, SHOOTER_INDEXER_PORT, SHOOTER_SWITCH_PORT);
    climber = new ClimberSubsystem(specialops, CLIMBER_EXTENDER_PORT, CLIMBER_EXTENDER_SWITCH, CLIMBER_ROTATOR_PORT, CLIMBER_ROTATOR_SWITCH);

    FLangleMotor = new CANSparkMax(FLangleID, MotorType.kBrushed);
    FLdriveMotor = new CANSparkMax(FLdriveID, MotorType.kBrushless);
    FRangleMotor = new CANSparkMax(FRangleID, MotorType.kBrushed);
    FRdriveMotor = new CANSparkMax(FRdriveID, MotorType.kBrushless);
    BRangleMotor = new CANSparkMax(BRangleID, MotorType.kBrushed);
    BRdriveMotor = new CANSparkMax(BRdriveID, MotorType.kBrushless);
    BLdriveMotor = new CANSparkMax(BLdriveID, MotorType.kBrushless);
    BLangleMotor = new CANSparkMax(BLangleID, MotorType.kBrushed);

    m_PIDController1 = FLangleMotor.getPIDController();
    m_PIDController2 = FRangleMotor.getPIDController();
    m_PIDController3 = BRangleMotor.getPIDController();
    m_PIDController4 = BLangleMotor.getPIDController();

    // RESET SPARK MAX
    FLangleMotor.restoreFactoryDefaults();
    FRangleMotor.restoreFactoryDefaults();
    FLdriveMotor.restoreFactoryDefaults();
    FRdriveMotor.restoreFactoryDefaults();
    BLangleMotor.restoreFactoryDefaults();
    BRangleMotor.restoreFactoryDefaults();
    BLdriveMotor.restoreFactoryDefaults();
    BRdriveMotor.restoreFactoryDefaults();

    //TIMER
    autotimer = new Timer();

    m_encoder1 = FLangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder1.setPosition(0);
    m_encoder1.setInverted(false);
    m_encoder2 = FRangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder2.setInverted(false);
    m_encoder2.setPosition(0);
    m_encoder3 = BRangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder3.setInverted(false);
    m_encoder3.setPosition(0);
    m_encoder4 = BLangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder4.setInverted(false);
    m_encoder4.setPosition(0);

    // kP = 75; 
    // kI = 1e-3;
    // kD = 1; 
    // kIz = 0; 
    // kFF = 0; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // m_encoder1.setPosition(0);
    // m_encoder2.setPosition(0);
    // m_encoder3.setPosition(0);
    // m_encoder4.setPosition(0);
    // m_PIDController3.setP(kP);
    // m_PIDController3.setP(kP);
    // m_PIDController3.setP(kP);
    // m_PIDController3.setP(kP);
    // m_PIDController3.setFF(kFF);
    // m_PIDController3.setOutputRange(kMinOutput, kMaxOutput);

    kP = 1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_PIDController1.setFeedbackDevice(m_encoder1);
    m_PIDController2.setFeedbackDevice(m_encoder2);
    m_PIDController3.setFeedbackDevice(m_encoder3);
    m_PIDController4.setFeedbackDevice(m_encoder4);

    m_PIDController1.setP(kP);
    m_PIDController2.setP(kP);
    m_PIDController3.setP(kP);
    m_PIDController4.setP(kP);

    m_PIDController1.setI(kI);
    m_PIDController2.setI(kI);
    m_PIDController3.setP(kP);
    m_PIDController4.setI(kI);

    m_PIDController1.setD(kD);
    m_PIDController2.setD(kD);
    m_PIDController3.setP(kP);
    m_PIDController4.setD(kD);

    m_PIDController1.setIZone(kIz);
    m_PIDController2.setIZone(kIz);
    m_PIDController3.setP(kP);
    m_PIDController4.setIZone(kIz);

    m_PIDController1.setFF(kFF);
    m_PIDController2.setFF(kFF);
    m_PIDController3.setFF(kFF);
    m_PIDController4.setFF(kFF);

    m_PIDController1.setOutputRange(kMinOutput, kMaxOutput);
    m_PIDController2.setOutputRange(kMinOutput, kMaxOutput);
    m_PIDController3.setOutputRange(kMinOutput, kMaxOutput);
    m_PIDController4.setOutputRange(kMinOutput, kMaxOutput);

    
    FLangleMotor.setInverted(true);
    FRangleMotor.setInverted(true);
    BLangleMotor.setInverted(true);
    BRangleMotor.setInverted(true);

    if (USE_CAMERAS && !isSimulation()) {
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
    MotorFactory.updateDashboard();
    SmartDashboard.putNumber("ProcessVariable 1", m_encoder1.getPosition());
    SmartDashboard.putNumber("ProcessVariable 2", m_encoder2.getPosition());
    SmartDashboard.putNumber("ProcessVariable 3", m_encoder3.getPosition());
    SmartDashboard.putNumber("ProcessVariable 4", m_encoder4.getPosition());

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
    if (intake != null) {
      intake.autonomousInit();
    }
    autotimer.stop();
    autotimer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (intake != null) {
      intake.autonomousPeriodic();
    }
    autotimer.start();
    while(autotimer.get() > 0) {
      if (autotimer.get() < 4) {
        FLdriveMotor.set(.10);
        FRdriveMotor.set(.10);
        BLdriveMotor.set(.10);
        BRdriveMotor.set(.10);
        m_PIDController1.setReference(0, CANSparkMax.ControlType.kPosition);
        m_PIDController2.setReference(0, CANSparkMax.ControlType.kPosition);
        m_PIDController3.setReference(0, CANSparkMax.ControlType.kPosition);
        m_PIDController4.setReference(0, CANSparkMax.ControlType.kPosition);  
  
      }
      else {
        FLdriveMotor.set(0);
        FRdriveMotor.set(0);
        BLdriveMotor.set(0);
        BRdriveMotor.set(0);  
      }
   }

    
  }

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
    if (climber != null) {
      climber.teleopPeriodic();
    }
    // Aim Bot, Squared and limtied, Snake drive, turbo enable, strafe to sensative and porportional and cubed, adjust squaring 
    if(drive_control.getLeftY() > 0) {
      FLdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));
      FRdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));
      BRdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));
      BLdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));

      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);            
    }
    if(drive_control.getLeftY() < 0) {
      FLdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));
      FRdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));
      BRdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));
      BLdriveMotor.set((drive_control.getLeftY()) / (-1 * DefaultLimit));

      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);   
    }
    // rotation
    if(drive_control.getRightX() > 0) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      // FLdriveMotor.set(drive_control.getRightX() / 2);
      FLdriveMotor.set(Math.sqrt(drive_control.getRightX()));
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      // FRdriveMotor.set(drive_control.getRightX() / -2);
      FRdriveMotor.set(-1 * (Math.sqrt(drive_control.getRightX())));
      m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
      // BLdriveMotor.set(drive_control.getRightX() / 2);
      BLdriveMotor.set(Math.sqrt(drive_control.getRightX()));
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      // BRdriveMotor.set(drive_control.getRightX() / -2);
      BRdriveMotor.set(-1 * (Math.sqrt(drive_control.getRightX())));

    }
      // Rotation
    if(drive_control.getRightX() < 0) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      // FLdriveMotor.set(drive_control.getRightX() / 2);
      FLdriveMotor.set(Math.sqrt(drive_control.getRightX()));
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      // FRdriveMotor.set(drive_control.getRightX() / -2);
      FRdriveMotor.set(-1 * (Math.sqrt(drive_control.getRightX())));
      m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
      // BLdriveMotor.set(drive_control.getRightX() / 2);
      BLdriveMotor.set(Math.sqrt(drive_control.getRightX()));
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      // BRdriveMotor.set(drive_control.getRightX() / -2);
      BRdriveMotor.set(-1 * (Math.sqrt(drive_control.getRightX()))); 
    }

    AimBot();
    SnakeDrive();
    // StrafeSwerve();
    ChangeLimited();
  }
  public void StrafeSwerve() {
    if(drive_control.getRawAxis(0) > .5) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(StrafeLimit);
      m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(StrafeLimit);
      m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(StrafeLimit);
      m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(StrafeLimit);
    }
    if(drive_control.getRawAxis(0) < .5) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(-StrafeLimit);
      m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(-StrafeLimit);
      m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(-StrafeLimit);
      m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(-StrafeLimit);
    }    
  }

  public void AimBot() {
    if(drive_control.getLeftBumper() == true) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(drive_control.getRightX() / 8);
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(drive_control.getRightX() / -8);
      m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(drive_control.getRightX() / 8);
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(drive_control.getRightX() / -8);  
    }
  }
  public void ChangeLimited() {
    if(drive_control.getRightBumper() == true) {
      DefaultLimit = 2;
      RotationLimit = .50;
      StrafeLimit = .50;
    }
    else {
      DefaultLimit = 4;
      RotationLimit = .25;
      StrafeLimit = .25; 
    }
  }
  
  public void SnakeDrive() {
    if(drive_control.getRightTriggerAxis() > .05) {
      m_PIDController1.setReference(2, ControlType.kPosition);
      FLdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
      FRdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
      BLdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
      BRdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
      m_PIDController2.setReference(2, ControlType.kPosition);
    }
      if(drive_control.getLeftTriggerAxis() > .05) {
        m_PIDController1.setReference(-2, ControlType.kPosition);
        FLdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
        FRdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
        BLdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
        BRdriveMotor.set(Math.sqrt(drive_control.getLeftY()));
        m_PIDController2.setReference(-2, ControlType.kPosition);
    }

  }
  
  // /** This function is called once when the robot is disabled. */
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
  public void testInit() {
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);


  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putString("MotorTesting: ", "None");
    if(drive_control.getStartButton() == true) {
      m_encoder1.setPosition(0);
      System.out.print("Encoder 1 Reset");
      m_encoder2.setPosition(0);
      System.out.print("Encoder 2 Reset");
      m_encoder3.setPosition(0);
      System.out.print("Encoder 3 Reset");
      m_encoder4.setPosition(0);
      System.out.print("Encoder 4 Reset");
    }
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    if((p != kP)) { m_PIDController1.setP(p); kP = p; }
    if((i != kI)) { m_PIDController1.setI(i); kI = i; }
    if((d != kD)) { m_PIDController1.setD(d); kD = d; }
    if((iz != kIz)) { m_PIDController1.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_PIDController1.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PIDController1.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { m_PIDController2.setP(p); kP = p; }
    if((i != kI)) { m_PIDController2.setI(i); kI = i; }
    if((d != kD)) { m_PIDController2.setD(d); kD = d; }
    if((iz != kIz)) { m_PIDController2.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_PIDController2.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PIDController2.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { m_PIDController3.setP(p); kP = p; }
    if((i != kI)) { m_PIDController3.setI(i); kI = i; }
    if((d != kD)) { m_PIDController3.setD(d); kD = d; }
    if((iz != kIz)) { m_PIDController3.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_PIDController3.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PIDController3.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { m_PIDController4.setP(p); kP = p; }
    if((i != kI)) { m_PIDController4.setI(i); kI = i; }
    if((d != kD)) { m_PIDController4.setD(d); kD = d; }
    if((iz != kIz)) { m_PIDController4.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_PIDController4.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PIDController4.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }



    // MOTOR TEST CONTROLS


    while(drive_control.getAButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Front Left");
      FLdriveMotor.set(drive_control.getLeftY());
      FLangleMotor.set(drive_control.getRightY());
    }
    while(drive_control.getBButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Front Right");
      FRdriveMotor.set(drive_control.getLeftY());
      FRangleMotor.set(drive_control.getRightY());
    }
    while(drive_control.getXButton() == true) {   
       SmartDashboard.putString("MotorTesting: ", "Back Left");
      BLdriveMotor.set(drive_control.getLeftY());
      BLangleMotor.set(drive_control.getRightY());
    }
    while(drive_control.getYButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Back Right");
      BRdriveMotor.set(drive_control.getLeftY());
      BRangleMotor.set(drive_control.getRightY());
    }

  }
}