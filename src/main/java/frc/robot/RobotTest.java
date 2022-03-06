// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogEncoder;
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
public class RobotTest extends TimedRobot {
 
  private static final int  FLangleID = 4;
  private static final int FLdriveID = 3;
  // private static final int  FLangleID = 8;
  // private static final int FLdriveID = 7;
  // private static final int  FRangleID = 6;
  // private static final int FRdriveID = 5;
  // private static final int  BRangleID = 4;
  // private static final int BRdriveID = 3;
  // private static final int  BLangleID = 2;
  // private static final int BLdriveID = 1;
  private CANSparkMax FLangleMotor;
  private CANSparkMax FLdriveMotor;

  private int count = 0;
  
  private SparkMaxPIDController m_PIDController1;
  
  private SparkMaxAnalogSensor m_encoder1;
  private RelativeEncoder m_encoder2;
 
  private XboxController drive_control;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    drive_control = new XboxController(0);
    
    FLangleMotor = new CANSparkMax(FLangleID, MotorType.kBrushed);
    FLdriveMotor = new CANSparkMax(FLdriveID, MotorType.kBrushless);

    m_PIDController1 = FLangleMotor.getPIDController();

    //m_encoder1 = FLangleMotor.getAnalog(Mode.kAbsolute);
    m_encoder2 = FLangleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 1600);
    m_encoder2.setPosition(0);
    m_encoder2.setInverted(true);

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_PIDController1.setFeedbackDevice(m_encoder2);
  
    m_PIDController1.setP(kP);
    
    m_PIDController1.setI(kI);

    m_PIDController1.setD(kD);

    m_PIDController1.setIZone(kIz);

    m_PIDController1.setFF(kFF);
    
    m_PIDController1.setOutputRange(kMinOutput, kMaxOutput);

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

    SmartDashboard.putNumber("Position", m_encoder2.getPosition());

    SmartDashboard.putNumber("")

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
    count++;
    SmartDashboard.putNumber("FLPosition", m_encoder2.getPosition());
    SmartDashboard.putNumber("liveCount", count);
    SmartDashboard.updateValues();
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
    if(drive_control.getLeftY() > 0) {
      FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
     
      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
      
    }
    if(drive_control.getLeftY() < 0) {
      FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
      
      m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
      
    }
    if(drive_control.getRightX() > .0) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(0.25);
    }
      // Rotation
    if(drive_control.getRightX() < 0) {
      m_PIDController1.setReference(-1 * 2,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(-1 * .25);
          
    }

    SnakeDrive();
    AimBot();
    StrafeSwerve();
  }
  
  public void StrafeSwerve() {
    if(drive_control.getRawAxis(0) == 1) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(- 0.25);
      
    }
    if(drive_control.getRawAxis(0) == -1) {
      m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
      FLdriveMotor.set(0.25);
    }    
  }
  public void SnakeDrive() {
    if(drive_control.getLeftTriggerAxis() > 0.05) {
      int limiter = 5;
    }
    else {
      int limiter = 10;
    }
  }

  public void AimBot() {
    if(drive_control.getLeftBumper() == true) {
      m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
      
      FLdriveMotor.set(drive_control.getRightX() / 15);
      
    }
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    
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