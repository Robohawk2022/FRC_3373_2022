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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {
    

    private final SpecialOpsController controller;
    private CANSparkMax IntakeMotor;
    private RelativeEncoder intakeEncoder;
    private double targetSpeed;
    private double Inversetargetspeed;
    private double maxTargetSpeed;
    private boolean spinWheel;
    //private Double Motorspeed = IntakeMotor.get();
    public IntakeSubsystem(SpecialOpsController controller) {
        this.controller = controller;
        IntakeMotor = new CANSparkMax(1, MotorType.kBrushless); //TODO change motor ID in CAN
        IntakeMotor.setOpenLoopRampRate(3.0);
        IntakeMotor.setClosedLoopRampRate(3.0);
        intakeEncoder = IntakeMotor.getEncoder();
        targetSpeed = 0.0;
        Inversetargetspeed = -0.0;
        maxTargetSpeed = 0.4;
        spinWheel = false;
        SmartDashboard.putNumber("Intake.MaxSpeed", maxTargetSpeed);
    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("Intake.currentspeed", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Intake.targetspeed", targetSpeed);
        System.err.println("reading dashboard");
        try {
            double newMax = SmartDashboard.getNumber("Intake.MaxSpeed", maxTargetSpeed);
            if (newMax != maxTargetSpeed) {
                System.err.println("setting target speed to "+newMax);
                maxTargetSpeed = newMax;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        //SmartDashboard.setDefaultNumber("Intake.totalBalls", totalBalls);
    }
    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {
         
        if (controller.getYButtonPressed()) {
            maxTargetSpeed = maxTargetSpeed+.1;
             //SmartDashboard.putBoolean("Was A Pressed", controller.getYButtonPressed());
             
        }
        if (controller.getXButtonPressed()) {
             maxTargetSpeed = maxTargetSpeed-.1;
             //SmartDashboard.putBoolean("Was B Pressed", controller.getXButtonPressed());
        } 

        if (controller.getBackButtonPressed()) {
            spinWheel = !spinWheel;
        }
        if (spinWheel) {
            targetSpeed = maxTargetSpeed;
            Inversetargetspeed = -maxTargetSpeed;
        }
        else {
            targetSpeed = 0;
            Inversetargetspeed = -0.0;
        }

        if (controller.getAButton()) {
            IntakeMotor.set(Inversetargetspeed);
            //SmartDashboard.putBoolean("Was Inverse Intake Requested", controller.wasIntakeReverseRequested());
        }
        else {
            IntakeMotor.set(targetSpeed);
        }
       
    }

    public void disabledInit() {
        // what should happen here?
    }
}
// Determine wether person ball vomit to be fast or slow