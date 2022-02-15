package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.testrobots.RobotPortMap;

import java.lang.System.Logger;

import javax.swing.plaf.TreeUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {


    private final SpecialOpsController controller;
    
    private CANSparkMax indexerMoter;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;



  double IntakeSpeed = 0.0;
  double IntakeCurrent = 0.0;
  double IntakeAcceleration = 0.02;
  double FeedSpeed = 0.0;
  double FeedCurrent = 0.0;
  double FeedAcceleration = 0.02;
  double ShooterSpeed = 0.0;
  double ShooterAcceleration = 0.01;
  double ShooterCurrent = 0.0;
  boolean PRESSED = false;
  
    public IntakeSubsystem(SpecialOpsController controller) {
        this.controller = controller;

        indexerMoter = new CANSparkMax(1, MotorType.kBrushless);
        indexerMoter.restoreFactoryDefaults();
        m_pidController = indexerMoter.getPIDController();
        m_encoder = indexerMoter.getEncoder();
    
        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;


    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    }

    public void robotPeriodic() {
       // SmartDashboard.putNumber("Intake/totalBalls", totalBalls);
       // SmartDashboard.putNumber("Intake/totalBalls2", totalBalls);
    }

    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
        System.err.println("*** setting to "+rotations);
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    


        // else if (controller.getBButtonPressed()) {
        //     BallVomit.reset();
        //     BallVomit.start();
        //     boolean ballvomit = true;
        // }  //while the timer is under 3 spit all the balls
        // else {
        //     IntakeMotor.set(0);
        //     // sets it to 0
        // }

        // while (BallVomit.get() <= 3) {
        //     IntakeMotor.set(-.1);
        // }
        // // For kicking balls out
        // while (ballvomit == true) {
        //     if (controller.getXButtonPressed()) {
        //         IntakeMotor.set(+.1);
        //     }
        //     if (controller.getYButtonPressed()) {
        //         IntakeMotor.set(-.1);
        //     } // This is for adding speed
        // }
        

    }

    public void disabledInit() {
        // what should happen here?
    }
}
// Determine wether person ball vomit to be fast or slow
