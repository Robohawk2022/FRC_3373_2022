package frc.robot.specialops;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotPortMap;

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
    public IntakeSubsystem(SpecialOpsController controller) {
        this.controller = controller;
        this.intakeReadySwitch = new DigitalInput(RobotPortMap.INTAKE_READY_PORT);
        IntakeMotor = new CANSparkMax(1, MotorType.kBrushless); //TODO change motor ID in CAN
        IntakeMotor.setOpenLoopRampRate(3.0);
        IntakeMotor.setClosedLoopRampRate(3.0);
        BallVomit = new Timer();
        // this.intakeWheel = ???
        targetSpeed = 0.4;
    }

    public void updateDashboard() {
        SmartDashboard.setDefaultNumber("Intake.totalBalls", totalBalls);
    }

    public void updateControls() {
        if (controller.wasIntakeReverseRequested()) {
            // TODO what should happen here?
            // beginning with nothing special just when it was requested
            targetSpeed = -targetSpeed;
        }
        IntakeMotor.set(targetSpeed);



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

    public void disable() {
        // what should happen here?
    }
}
// Determine wether person ball vomit to be fast or slow