// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class DriveSubsystem
{
  public static int DefaultLimit = 4;
  public static double RotationLimit = .25;
  public static double StrafeLimit = .25;

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

  public Timer autoTimer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public DriveSubsystem(XboxController drive_control,
    int FLangleID, int FLdriveID, int FRangleID, int FRdriveID, 
    int BRangleID, int BRdriveID, int BLangleID, int BLdriveID) {

    SmartDashboard.putString("MotorTesting: ", "None");
    
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

    allMotors().forEach(motor -> {
      motor.restoreFactoryDefaults();
    });

    m_encoder1 = FLangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder2 = FRangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder3 = BRangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    m_encoder4 = BLangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    allEncoders().forEach(encoder -> {
      encoder.setPosition(0);
      encoder.setInverted(false);
    });

    // kP = 75; 
    // kI = 1e-3;
    // kD = 1; 
    // kIz = 0; 
    // kFF = 0; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;

    kP = 0.1; 
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
    allControllers().forEach(this::setPIDParams);
    
    FLangleMotor.setInverted(true);
    FRangleMotor.setInverted(true);
    BLangleMotor.setInverted(true);
    BRangleMotor.setInverted(true);
  }

  private void setPIDParams(SparkMaxPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setIZone(kIz);
    controller.setFF(kFF);
    controller.setOutputRange(kMinOutput, kMaxOutput);
  }

  private List<CANSparkMax> allMotors() {
    return Arrays.asList(
      FLangleMotor, FLdriveMotor, FRangleMotor, FRdriveMotor,
      BRangleMotor, BRdriveMotor, BLangleMotor, BLdriveMotor);
  }

  private List<SparkMaxPIDController> allControllers() {
    return Arrays.asList(m_PIDController1, m_PIDController2, m_PIDController3, m_PIDController4);
  }

  private List<RelativeEncoder> allEncoders() {
    return Arrays.asList(m_encoder1, m_encoder2, m_encoder3, m_encoder4);
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("ProcessVariable 1", m_encoder1.getPosition());
    SmartDashboard.putNumber("ProcessVariable 2", m_encoder2.getPosition());
    SmartDashboard.putNumber("ProcessVariable 3", m_encoder3.getPosition());
    SmartDashboard.putNumber("ProcessVariable 4", m_encoder4.getPosition());
  }

  public void teleopPeriodic() {

    if(drive_control.getLeftY() > 0) {
      FLdriveMotor.set((drive_control.getLeftY() * -1) / DefaultLimit);
      FRdriveMotor.set((drive_control.getLeftY() * -1) / DefaultLimit);
      BRdriveMotor.set(drive_control.getLeftY() / DefaultLimit);
      BLdriveMotor.set(drive_control.getLeftY() / DefaultLimit);

      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);            
    }

    if(drive_control.getLeftY() < 0) {
      FLdriveMotor.set((drive_control.getLeftY() * -1) / DefaultLimit);
      FRdriveMotor.set((drive_control.getLeftY() * -1) / DefaultLimit);
      BRdriveMotor.set(drive_control.getLeftY() / DefaultLimit);
      BLdriveMotor.set(drive_control.getLeftY() / DefaultLimit);

      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);   
    }

    if(drive_control.getRightX() > .0) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(RotationLimit);
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(-1 * RotationLimit);
      m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(-1 * RotationLimit);
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(RotationLimit);
    }

    // Rotation
    if(drive_control.getRightX() < 0) {
      m_PIDController1.setReference(-1 * 2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(-1 * RotationLimit);
      m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(RotationLimit);
      m_PIDController3.setReference(-1 * 2,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(RotationLimit);
      m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(-1 * RotationLimit);    
    }

    AimBot();
    StrafeSwerve();
  }

  public void StrafeSwerve() {
    if(drive_control.getRawAxis(0) == 1) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(- StrafeLimit);
      m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(- StrafeLimit);
      m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(StrafeLimit);
      m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(StrafeLimit);
    }
    if(drive_control.getRawAxis(0) == -1) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(StrafeLimit);
      m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
      FRdriveMotor.set(StrafeLimit);
      m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
      BLdriveMotor.set(- StrafeLimit);
      m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
      BRdriveMotor.set(- StrafeLimit);
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

  public void Auto() {
    autoTimer = new Timer();

    autoTimer.start();

    if(autoTimer.get() < 2.5) {
      FLdriveMotor.set(-.25);
      FRdriveMotor.set(-.25);
      BLdriveMotor.set(-.25);
      BRdriveMotor.set(-.25);
    }
  }
  
  public void disabledInit() {
    allMotors().forEach(motor -> {
      motor.stopMotor();
    });
  }

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
  public void testPeriodic() {
    SmartDashboard.putString("MotorTesting: ", "None");
    if(drive_control.getBButton() == true) {
      System.out.println("resetting encoders");
      allEncoders().stream().forEach(encoder -> encoder.setPosition(0));
    }

    boolean update = false;

    double p = SmartDashboard.getNumber("P Gain", 0);
    if (p != kP) {
      kP = p;
      update = true;
    }

    double i = SmartDashboard.getNumber("I Gain", 0);
    if (i != kI) {
      kI = i;
      update = true;
    }
    
    double d = SmartDashboard.getNumber("D Gain", 0);
    if (d != kD) {
      kD = d;
      update = true;
    }
    
    double iz = SmartDashboard.getNumber("I Zone", 0);
    if (iz != kIz) {
      kIz = iz;
      update = true;
    }
    
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    if (ff != kFF) {
      kFF = ff;
      update = true;
    }
    
    double min = SmartDashboard.getNumber("Min Output", 0);
    if (min != kMinOutput) {
      kMinOutput = min;
      update = true;
    }
    
    double max = SmartDashboard.getNumber("Max Output", 0);
    if (max != kMaxOutput) {
      kMaxOutput = max;
      update = true;
    }

    if (update) {
      allControllers().forEach(this::setPIDParams);
    }

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