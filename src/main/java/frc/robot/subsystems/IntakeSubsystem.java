package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.testrobots.RobotPortMap;

import java.lang.System.Logger;

import javax.swing.plaf.TreeUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {
    

    private final SpecialOpsController controller;
    private final DigitalInput intakeReadySwitch;
    private int totalBalls;
    private CANSparkMax IntakeMotor;
    private Timer BallVomit;
    private boolean ballvomit = false;
    private double targetSpeed;
    private double Inversetargetspeed;
    private Double Motorspeed = IntakeMotor.get();
    public IntakeSubsystem(SpecialOpsController controller) {
        this.controller = controller;
        this.intakeReadySwitch = new DigitalInput(RobotPortMap.INTAKE_READY_PORT);
        IntakeMotor = new CANSparkMax(1, MotorType.kBrushless); //TODO change motor ID in CAN
        IntakeMotor.setOpenLoopRampRate(3.0);
        IntakeMotor.setClosedLoopRampRate(3.0);
        BallVomit = new Timer();
        // this.intakeWheel = ???
        targetSpeed = 0.4;
        Inversetargetspeed = -0.4;
    }

    public void robotPeriodic() {
        SmartDashboard.setDefaultNumber("Intake.totalBalls", totalBalls);
    }
    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {

        if (controller.wasIntakeRequested()) {
            IntakeMotor.set(targetSpeed);
            SmartDashboard.putBoolean("Was Intake Requested", controller.wasIntakeRequested());
            SmartDashboard.putNumber("Current Speed", Motorspeed);
            // TODO what should happen here?
            // beginning with nothing special just when it was requested
        }
        if (controller.wasIntakeReverseRequested()) {
            IntakeMotor.set(Inversetargetspeed);
            SmartDashboard.putBoolean("Was Inverse Intake Requested", controller.wasIntakeReverseRequested());
        }
        
        if (controller.getAButtonPressed()) {
            IntakeMotor.set(+.1);
            SmartDashboard.putBoolean("Was A Pressed", controller.getAButtonPressed());
            
        }
        if (controller.getBButtonPressed()) {
            IntakeMotor.set(-.1);
            SmartDashboard.putBoolean("Was B Pressed", controller.getBButtonPressed());
        } 
        else {
            IntakeMotor.set(0.0);
        }
    }

    public void disabledInit() {
        // what should happen here?
    }
}
// Determine wether person ball vomit to be fast or slow
