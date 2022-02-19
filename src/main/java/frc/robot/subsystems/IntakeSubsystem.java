package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TeleopMode;
import frc.robot.motors.Motor;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.MotorSettings;
import frc.robot.motors.MotorSettings.Type;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    /** This is the starting value for the speed for the intake wheel (in RPM) */
    public static final double STARTING_SPEED = 1200;
    
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
            spinWheel = !spinWheel;
        }

        // if the wheel is spinning, we'll allow speed changes
        if (spinWheel) {

            // hold both bumpers: reset target speed
            if (controller.getLeftBumper() && controller.getRightBumper()) {
                targetSpeed = STARTING_SPEED;
            }
            // press left bumper: go 10% slower
            else if (controller.getLeftBumperPressed()) {
                targetSpeed *= 0.9;
            }
            // press right bumper: go 10% faster
            else if (controller.getRightBumperPressed()) {
                targetSpeed *= 1.1;
            }

            // hold left trigger: reverse the wheel
            if (controller.getLeftTriggerAxis() > 0.5) {
                intakeMotor.set(-targetSpeed);
            }
            else {
                intakeMotor.set(targetSpeed);
            }       
        }
        else {
            intakeMotor.set(0.0);
        }
    }

    public void disabledInit() {
        spinWheel = false;
        intakeMotor.set(0.0);
    }

    private Motor createMotor() {
        MotorSettings settings = new MotorSettings();
        settings.port = 1;
        settings.name = "Intake";
        settings.rampRate = 2.0;
        settings.type = Type.VELOCITY;
        return MotorFactory.createMotor(settings);
    }
}
