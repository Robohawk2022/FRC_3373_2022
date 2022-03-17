// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Logger;
import frc.robot.util.PIDConstant;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.RobotConstants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Field2d field = new Field2d();
  private XboxController specialops;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ClimberSubsystem climber;
  private CANSparkMax frontLeftAngleMotor;
  private CANSparkMax frontLeftDriveMotor;
  private CANSparkMax frontRightDriveMotor;
  private CANSparkMax frontRightAngleMotor;
  private CANSparkMax backLeftAngleMotor;
  private CANSparkMax backLeftDriveMotor;
  private CANSparkMax backRightDriveMotor;
  private CANSparkMax backRightAngleMotor;
  private SparkMaxPIDController frontLeftPidController;
  private SparkMaxPIDController frontRightPidController;
  private SparkMaxPIDController backRightPidController;
  private SparkMaxPIDController backLeftPidController;
  private RelativeEncoder frontLeftAngleEncoder;
  private RelativeEncoder frontRightAngleEncoder;
  private RelativeEncoder backRightAngleEncoder;
  private RelativeEncoder backLeftAngleEncoder;
  private XboxController drive_control;
  private double turboFactor;
  private double reverseFactor;
  private PIDConstant pidConstant;
  private double autonomousStart;
  private SendableChooser<String> autoMode;
  private SendableChooser<String> driveMode;
  private TrueSwerve trueSwerve;
  private double rotateUntil;
  private boolean useSwerve;

/* ==============================================================================
  _____   ____  ____   ____ _______ 
 |  __ \ / __ \|  _ \ / __ \__   __|
 | |__) | |  | | |_) | |  | | | |   
 |  _  /| |  | |  _ <| |  | | | |   
 | | \ \| |__| | |_) | |__| | | |   
 |_|  \_\\____/|____/ \____/  |_|   
                                                                      
============================================================================== */

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", field);
    SmartDashboard.putString("MotorTesting: ", "None");
    drive_control = new XboxController(DRIVER_PORT);
    intake = new IntakeSubsystem(drive_control, INTAKE_PORT);
    specialops = new XboxController(SPECIAL_OPS_PORT);
    shooter = new ShooterSubsystem(specialops, SHOOTER_LAUNCH_PORT, SHOOTER_INDEXER_PORT, SHOOTER_SWITCH_PORT);
    climber = new ClimberSubsystem(specialops, CLIMBER_EXTENDER_PORT, CLIMBER_EXTENDER_SWITCH, CLIMBER_ROTATOR_PORT, CLIMBER_ROTATOR_SWITCH);

    frontLeftAngleMotor = new CANSparkMax(FRONT_LEFT_ANGLE_ID, MotorType.kBrushed);
    frontLeftDriveMotor = new CANSparkMax(FRONT_LEFT_DRIVE_ID, MotorType.kBrushless);
    frontRightAngleMotor = new CANSparkMax(FRONT_RIGHT_ANGLE_ID, MotorType.kBrushed);
    frontRightDriveMotor = new CANSparkMax(FRONT_RIGHT_DRIVE_ID, MotorType.kBrushless);
    backRightAngleMotor = new CANSparkMax(BACK_RIGHT_ANGLE_ID, MotorType.kBrushed);
    backRightDriveMotor = new CANSparkMax(BACK_RIGHT_DRIVE_ID, MotorType.kBrushless);
    backLeftDriveMotor = new CANSparkMax(BACK_LEFT_DRIVE_ID, MotorType.kBrushless);
    backLeftAngleMotor = new CANSparkMax(BACK_LEFT_ANGLE_ID, MotorType.kBrushed);

    frontLeftPidController = frontLeftAngleMotor.getPIDController();
    frontRightPidController = frontRightAngleMotor.getPIDController();
    backRightPidController = backRightAngleMotor.getPIDController();
    backLeftPidController = backLeftAngleMotor.getPIDController();

    // RESET SPARK MAX
    frontLeftAngleMotor.restoreFactoryDefaults();
    frontRightAngleMotor.restoreFactoryDefaults();
    frontLeftDriveMotor.restoreFactoryDefaults();
    frontRightDriveMotor.restoreFactoryDefaults();
    backLeftAngleMotor.restoreFactoryDefaults();
    backRightAngleMotor.restoreFactoryDefaults();
    backLeftDriveMotor.restoreFactoryDefaults();
    backRightDriveMotor.restoreFactoryDefaults();

    frontRightDriveMotor.setOpenLoopRampRate(1.0);
    frontLeftDriveMotor.setOpenLoopRampRate(1.0);
    backRightDriveMotor.setOpenLoopRampRate(1.0);
    backLeftDriveMotor.setOpenLoopRampRate(1.0);

    autoMode = new SendableChooser<>();
    autoMode.setDefaultOption("SingleShooter", "SingleShooter");
    autoMode.addOption("DoubleShooter", "DoubleShooter");
    autoMode.addOption("ClimberOnly", "ClimberOnly");
    SmartDashboard.putData("Auto Mode", autoMode);

    frontLeftAngleEncoder = frontLeftAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    frontLeftAngleEncoder.setPosition(0);
    frontLeftAngleEncoder.setInverted(false);

    frontRightAngleEncoder = frontRightAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    frontRightAngleEncoder.setInverted(false);
    frontRightAngleEncoder.setPosition(0);

    backRightAngleEncoder = backRightAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    backRightAngleEncoder.setInverted(false);
    backRightAngleEncoder.setPosition(0);

    backLeftAngleEncoder = backLeftAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    backLeftAngleEncoder.setInverted(false);
    backLeftAngleEncoder.setPosition(0);

    turboFactor = 1.0;
    reverseFactor = 1.0;

    setPidConstant(DEFAULT_ANGLE_PID);
    
    frontLeftAngleMotor.setInverted(true);
    frontRightAngleMotor.setInverted(true);
    backLeftAngleMotor.setInverted(true);
    backRightAngleMotor.setInverted(true);

    if (USE_CAMERAS && !isSimulation()) {
      CameraServer.startAutomaticCapture("Front", FRONT_CAMERA_PORT);
      CameraServer.startAutomaticCapture("Back", BACK_CAMERA_PORT);  
    }

    autonomousStart = 0.0;
    rotateUntil = 0.0;

    trueSwerve = new TrueSwerve();
    useSwerve = false;
    driveMode = new SendableChooser<>();
    driveMode.setDefaultOption("DriveDrive", "DriveDrive");
    driveMode.addOption("TrueSwerve", "TrueSwerve");
    SmartDashboard.putData("Drive Mode", driveMode);
  }

  private void setPidConstant(PIDConstant newConstant) {
    pidConstant = newConstant;
    pidConstant.configPID(frontLeftPidController);
    pidConstant.configPID(frontRightPidController);
    pidConstant.configPID(backRightPidController);
    pidConstant.configPID(backLeftPidController);
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
    SmartDashboard.putNumber("FL Angle Position", frontLeftAngleEncoder.getPosition());
    SmartDashboard.putNumber("FR Angle Position", frontRightAngleEncoder.getPosition());
    SmartDashboard.putNumber("BR Angle Position", backRightAngleEncoder.getPosition());
    SmartDashboard.putNumber("BL Angle Position", backLeftAngleEncoder.getPosition());
    SmartDashboard.putBoolean("Drive Reversed?", reverseFactor < -1.0);
  }

/* ==============================================================================
          _    _ _______ ____  _   _  ____  __  __  ____  _    _  _____ 
     /\  | |  | |__   __/ __ \| \ | |/ __ \|  \/  |/ __ \| |  | |/ ____|
    /  \ | |  | |  | | | |  | |  \| | |  | | \  / | |  | | |  | | (___  
   / /\ \| |  | |  | | | |  | | . ` | |  | | |\/| | |  | | |  | |\___ \ 
  / ____ \ |__| |  | | | |__| | |\  | |__| | |  | | |__| | |__| |____) |
 /_/    \_\____/   |_|  \____/|_| \_|\____/|_|  |_|\____/ \____/|_____/ 
                                                                                                                                              
============================================================================== */

  @Override
  public void autonomousInit() {

    // always clock the start of autonomous mode, and run the climber
    autonomousStart = Timer.getFPGATimestamp();
    climber.autonomousInit();

    Logger.log("starting auto program ", autoMode.getSelected());

    if ("DoubleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.doubleShooterInit();
      shooter.doubleShooterInit();
    } else if ("SingleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.singleShooterInit();
      shooter.singleShooterInit();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // always run the climber
    climber.autonomousPeriodic();

    double seconds = Timer.getFPGATimestamp() - autonomousStart;

    if ("DoubleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.doubleShooterPeriodic(seconds);
      shooter.doubleShooterPeriodic(seconds);
      doubleShooterPeriodic(seconds);
    } else if ("SingleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.singleShooterPeriodic(seconds);
      shooter.singleShooterPeriodic(seconds);
      singleShooterPeriodic(seconds);
    }
  }

  private void singleShooterPeriodic(double seconds) {
    if (seconds < 0.3) {   // let the intake wheel wind up a bit
      forwardBy(0.0, 0.0);
    } else if (seconds < 0.5) {   // ooch forward (picks up speed)
      forwardBy(1.0, 0.0);
    } else if (seconds < 1.0) {   // ooch backwards (drops intake frame)
      forwardBy(-1.0, 0.0);
    } else if (seconds < 6.2) {  // wait in place for intake to drop
      forwardBy(0.0, 0.0);
    } else if (seconds < 9.5) {   // exit the tarmac
      forwardBy(0.1, 0.0);
    } else {
      forwardBy(0.0, 0.0);
    }
  }

  private void doubleShooterPeriodic(double seconds) {
    if (seconds < 0.3) {   // let the intake wheel wind up a bit
      forwardBy(0.0, 0.0);
    } else if (seconds < 0.5) {   // ooch forward (picks up speed)
      forwardBy(1.0, 0.0);
    } else if (seconds < 1.0) {   // ooch backwards (drops intake frame)
      forwardBy(-1.0, 0.0);
    } else if (seconds < 6.2) {  // wait in place for intake to drop
      forwardBy(0.0, 0.0);
    } else if (seconds < 9.0) {   // go get a ball
      forwardBy(0.1, -0.5);
    } else if (seconds < 9.3) {   // reorient slightly to shoot
      forwardBy(0.1, 0.7);
    } else {
      forwardBy(0.0, 0.0);
    }
  }

  private void forwardBy(double speed, double angle) {
    frontLeftPidController.setReference(0, CANSparkMax.ControlType.kPosition);
    frontRightPidController.setReference(0, CANSparkMax.ControlType.kPosition);
    backRightPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    backLeftPidController.setReference(angle, CANSparkMax.ControlType.kPosition);  
    frontLeftDriveMotor.set(-speed);
    frontRightDriveMotor.set(speed);
    backLeftDriveMotor.set(-speed);
    backRightDriveMotor.set(speed);
  }

/* ==============================================================================
  _______ ______ _      ______ ____  _____  
 |__   __|  ____| |    |  ____/ __ \|  __ \ 
    | |  | |__  | |    | |__ | |  | | |__) |
    | |  |  __| | |    |  __|| |  | |  ___/ 
    | |  | |____| |____| |___| |__| | |     
    |_|  |______|______|______\____/|_|     
                                                                                      
============================================================================== */

  @Override
  public void teleopInit() {
    useSwerve = "TrueSwerve".equalsIgnoreCase(driveMode.getSelected());
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

    double rightX = drive_control.getRightX();
    double leftX = drive_control.getLeftX();
    double leftY = drive_control.getLeftY();

    // if someone hits start, we'll invert the "front" of the vehicle for driving
    if (drive_control.getStartButtonPressed()) {
      reverseFactor = reverseFactor * -1.0;
    }

    // if someone hits the dpad back, we'll start rotating for a fixed amount of time
    if (drive_control.getPOV() == 180 && rotateUntil < 0) {
      rotateUntil = Timer.getFPGATimestamp() + ROTATION_SECONDS;
    }

    // if someone is holding the right trigger, we'll double speed
    if (drive_control.getRightTriggerAxis() > 0.5) {
      turboFactor = 2.0;
    } else {
      turboFactor = 1.0;
    }

    // if we're turning, let's turn!
    if (rotateUntil < Timer.getFPGATimestamp()) {
      AimBot(2.0); // rotate in a fixed direction at 2x normal aimbot speed
    }

    if (drive_control.getRightBumper()) {
      AimBot(rightX);
      return;
    }

    if (drive_control.getLeftBumper()) {
      macDrive(leftX, leftY, rightX);
      return;
    }
    
    if (useSwerve) {
      trueSwerve(leftX, leftY, rightX);
      return;
    }

    driveDrive(leftX, leftY, rightX);
  }

  /* --------------------------------------------------
     Mac Drive turns like this (strafing):
         \---\
         |   |     <-- there's a glitch at +/- 90
         \---\         where the wheels flip around
    -------------------------------------------------- */

  public void macDrive(double leftX, double leftY, double rightX) {

    double moveSpeed = Math.sqrt(leftX * leftX + leftY * leftY) * MAX_DRIVE_POWER * turboFactor * reverseFactor;
    double turnAngle = leftX * leftX * leftX * MAX_ROTATION_POWER * reverseFactor;    

    if (drive_control.getLeftY() >= 0) {
      frontLeftDriveMotor.set(moveSpeed);
      frontRightDriveMotor.set(-moveSpeed);
      backRightDriveMotor.set(-moveSpeed);
      backLeftDriveMotor.set(moveSpeed);
      frontLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);            
    }

    if (drive_control.getLeftY() < 0) {
      frontLeftDriveMotor.set(-moveSpeed);
      frontRightDriveMotor.set(moveSpeed);
      backRightDriveMotor.set(moveSpeed);
      backLeftDriveMotor.set(-moveSpeed);
      frontLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);            
    }
  }

  /* --------------------------------------------------
     Drive Drive turns like this (car steering):
         /---/
         |   |
         \---\
    -------------------------------------------------- */

  public void driveDrive(double leftX, double leftY, double rightX) {

    double moveSpeed = Math.sqrt(leftX * leftX + leftY * leftY) * MAX_DRIVE_POWER * turboFactor * reverseFactor;
    double turnAngle = rightX * rightX * rightX * MAX_ROTATION_POWER * reverseFactor;   
    if (turnAngle > DRIVE_MODE_ROTATION_LIMIT) {
      turnAngle = DRIVE_MODE_ROTATION_LIMIT;
    }

    if (drive_control.getLeftY() >= 0) {
      frontLeftDriveMotor.set(moveSpeed);
      frontRightDriveMotor.set(-moveSpeed);
      backRightDriveMotor.set(-moveSpeed);
      backLeftDriveMotor.set(moveSpeed);
      frontLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);            
    }

    if (drive_control.getLeftY() < 0) {
      frontLeftDriveMotor.set(-moveSpeed);
      frontRightDriveMotor.set(moveSpeed);
      backRightDriveMotor.set(moveSpeed);
      backLeftDriveMotor.set(-moveSpeed);
      frontLeftPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);            
    }
  }

  /* --------------------------------------------------
     AimBot turns like this (rotation):
         /---\
         |   |
         \---/
    -------------------------------------------------- */

  public void AimBot(double rightX) {
    double rotateSpeed = -rightX / 8.0 * turboFactor;
    frontLeftDriveMotor.set(rotateSpeed);
    frontRightDriveMotor.set(rotateSpeed);
    backLeftDriveMotor.set(rotateSpeed);
    backRightDriveMotor.set(rotateSpeed);  
    frontLeftPidController.setReference(-MAGIC_ROTATE_ANGLE,  CANSparkMax.ControlType.kPosition);
    frontRightPidController.setReference(MAGIC_ROTATE_ANGLE,  CANSparkMax.ControlType.kPosition);
    backRightPidController.setReference(-MAGIC_ROTATE_ANGLE,  CANSparkMax.ControlType.kPosition);
    backLeftPidController.setReference(MAGIC_ROTATE_ANGLE,  CANSparkMax.ControlType.kPosition);
  }

  /* --------------------------------------------------
     True swerve mode (drives like a dolly)
    -------------------------------------------------- */

  private void trueSwerve(double leftX, double leftY, double rightX) {

    // get current position of wheels
    double frontLeftPos = frontLeftAngleEncoder.getPosition();
    double frontRightPos = frontRightAngleEncoder.getPosition();
    double backRightPos = backRightAngleEncoder.getPosition();
    double backLeftPos = backLeftAngleEncoder.getPosition();

    // compute new speed/position for each wheel
    SwerveModuleState [] states = trueSwerve.computeStates(
      leftX, leftY, rightX, 
      frontLeftPos, frontRightPos, backRightPos, backLeftPos);

    // convert degree angles to wheel position
    double frontLeftNewPos = trueSwerve.degreesToPosition(states[0].angle.getDegrees());
    double frontRightNewPos = trueSwerve.degreesToPosition(states[1].angle.getDegrees());
    double backRightNewPos = trueSwerve.degreesToPosition(states[2].angle.getDegrees());
    double backLeftNewPos = trueSwerve.degreesToPosition(states[3].angle.getDegrees());

    // apply speed/position
    frontLeftDriveMotor.set(states[0].speedMetersPerSecond);
    frontRightDriveMotor.set(-states[1].speedMetersPerSecond);
    backRightDriveMotor.set(-states[1].speedMetersPerSecond);
    backLeftDriveMotor.set(states[3].speedMetersPerSecond);
    frontLeftPidController.setReference(frontLeftNewPos, CANSparkMax.ControlType.kPosition);
    frontRightPidController.setReference(frontRightNewPos, CANSparkMax.ControlType.kPosition);
    backRightPidController.setReference(backRightNewPos, CANSparkMax.ControlType.kPosition);
    backLeftPidController.setReference(backLeftNewPos, CANSparkMax.ControlType.kPosition);            
  }

/* ==============================================================================
  _____ _____  _____         ____  _      ______ _____  
 |  __ \_   _|/ ____|  /\   |  _ \| |    |  ____|  __ \ 
 | |  | || | | (___   /  \  | |_) | |    | |__  | |  | |
 | |  | || |  \___ \ / /\ \ |  _ <| |    |  __| | |  | |
 | |__| || |_ ____) / ____ \| |_) | |____| |____| |__| |
 |_____/_____|_____/_/    \_\____/|______|______|_____/ 
                                                                                                              
============================================================================== */

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

/* ==============================================================================
  _______ ______  _____ _______ 
 |__   __|  ____|/ ____|__   __|
    | |  | |__  | (___    | |   
    | |  |  __|  \___ \   | |   
    | |  | |____ ____) |  | |   
    |_|  |______|_____/   |_|   
                                                            
============================================================================== */

  @Override
  public void testInit() {
    SmartDashboard.putNumber("P Gain", pidConstant.getP());
    SmartDashboard.putNumber("I Gain", pidConstant.getI());
    SmartDashboard.putNumber("D Gain", pidConstant.getD());
    SmartDashboard.putNumber("I Zone", pidConstant.getIZone());
    SmartDashboard.putNumber("Feed Forward", pidConstant.getFeedForward());
    SmartDashboard.putNumber("Min Output", pidConstant.getMinOutput());
    SmartDashboard.putNumber("Max Output", pidConstant.getMaxOutput());
  }

  @Override
  public void testPeriodic() {

    SmartDashboard.putString("MotorTesting: ", "None");

    if(drive_control.getBButton() == true) {
      frontLeftAngleEncoder.setPosition(0);
      frontRightAngleEncoder.setPosition(0);
      backRightAngleEncoder.setPosition(0);
      backLeftAngleEncoder.setPosition(0);
      Logger.log("robot: reset all angle encoders");
    }

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if (p != pidConstant.getP()
      || i != pidConstant.getI()
      || d != pidConstant.getD()
      || iz != pidConstant.getIZone()
      || ff != pidConstant.getFeedForward()
      || min != pidConstant.getMinOutput()
      || max != pidConstant.getMaxOutput()) {
        setPidConstant(new PIDConstant(p, i, d, ff, iz, min, max));
    }

    while(drive_control.getAButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Front Left");
      frontLeftDriveMotor.set(drive_control.getLeftY());
      frontLeftAngleMotor.set(drive_control.getRightY());
    }

    while(drive_control.getBButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Front Right");
      frontRightDriveMotor.set(drive_control.getLeftY());
      frontRightAngleMotor.set(drive_control.getRightY());
    }

    while(drive_control.getXButton() == true) {   
       SmartDashboard.putString("MotorTesting: ", "Back Left");
      backLeftDriveMotor.set(drive_control.getLeftY());
      backLeftAngleMotor.set(drive_control.getRightY());
    }

    while(drive_control.getYButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Back Right");
      backRightDriveMotor.set(drive_control.getLeftY());
      backRightAngleMotor.set(drive_control.getRightY());
    }
  }
}