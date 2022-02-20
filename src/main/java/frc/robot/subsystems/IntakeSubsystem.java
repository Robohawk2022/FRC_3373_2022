package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TeleopMode;
import frc.robot.motors.Motor;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.MotorSettings;
import frc.robot.motors.MotorSettings.Type;
import frc.robot.util.LogDedupe;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    /** This is the starting value for the speed for the intake wheel (in RPM) */
    public static final double STARTING_SPEED = 2000;
    
    private final XboxController controller;
    private final Motor intakeMotor;
    private boolean spinWheel = false;             // should the wheel be spinning right now?
    private double targetSpeed = STARTING_SPEED;   // what speed (in rpm) should it spin at when it spins?

    public IntakeSubsystem(XboxController controller) {
        this.controller = controller;
        this.intakeMotor = createMotor();
    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("Intake Target Speed", targetSpeed);
    }

    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {

        // back button turns the wheel on and off
        if (controller.getBackButtonPressed()) {
            System.err.println("toggling intake wheel");
            spinWheel = !spinWheel;
        }

        // if the wheel is spinning, we'll allow speed changes
        if (spinWheel) {

            // hold both bumpers: reset target speed
            if (controller.getLeftBumper() && controller.getRightBumper()) {
                System.err.println("reset intake wheel");
                targetSpeed = STARTING_SPEED;
            }
            // press left bumper: go 10% slower
            else if (controller.getLeftBumperPressed()) {
                System.err.println("slow down intake wheel");
                targetSpeed *= 0.9;
            }
            // press right bumper: go 10% faster
            else if (controller.getRightBumperPressed()) {
                System.err.println("speed up intake wheel");
                targetSpeed *= 1.1;
            }

            // hold left trigger: reverse the wheel
            // if (controller.getLeftTriggerAxis() > 0.5) {
            //     LogDedupe.log("reversing target to "+targetSpeed);
            //     intakeMotor.set(-targetSpeed);
            // }
            // else {
                LogDedupe.log("setting target to "+targetSpeed);
                intakeMotor.set(targetSpeed);
            // }       
        }
        else {
            LogDedupe.log("stopping");
            intakeMotor.set(0.0);
        }
    }

    public void disabledInit() {
        spinWheel = false;
        intakeMotor.set(0);
    }

    private Motor createMotor() {
        MotorSettings settings = new MotorSettings();
        settings.port = 1;
        settings.name = "Intake";
        settings.rampRate = 0.5;
        settings.allowCoasting = true;
        settings.type = Type.VELOCITY;
        return MotorFactory.createMotor(settings);
    }
}
