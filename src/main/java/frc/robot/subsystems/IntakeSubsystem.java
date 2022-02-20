package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TeleopMode;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    /** This is the starting value for the speed for the intake wheel (in RPM) */
    public static final double STARTING_SPEED = 2000;
    
    private final XboxController controller;
    private final VelocityClosedLoopMotor intakeMotor;
    private boolean spinWheel = false;             // should the wheel be spinning right now?
    private double targetSpeed = STARTING_SPEED;   // what speed (in rpm) should it spin at when it spins?

    public IntakeSubsystem(XboxController controller) {
        this.controller = controller;
        this.intakeMotor = MotorFactory.makeVelocityClosedLoopMotor("Intake", 3);
    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("Intake Target Speed", targetSpeed);
        SmartDashboard.putBoolean("Intake Spinning?", spinWheel);
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
            Logger.log("toggled intake wheel to ", spinWheel);
        }

        // if the wheel is spinning, we'll allow speed changes
        if (spinWheel) {

            // hold both bumpers: reset target speed
            if (controller.getLeftBumper() && controller.getRightBumper()) {
                targetSpeed = STARTING_SPEED;
                Logger.log("reset intake wheel to ", targetSpeed);
            }
            // press left bumper: go 10% slower
            else if (controller.getLeftBumperPressed()) {
                targetSpeed *= 0.9;
                Logger.log("slowed down intake wheel to ", targetSpeed);
            }
            // press right bumper: go 10% faster
            else if (controller.getRightBumperPressed()) {
                targetSpeed *= 1.1;
                Logger.log("sped up intake wheel to ", targetSpeed);
            }

            // hold left trigger: reverse the wheel
            if (controller.getLeftTriggerAxis() > 0.5) {
                intakeMotor.setRpm(-targetSpeed);
                Logger.log("reversing intake wheel to ", targetSpeed);
            }
            else {
                Logger.log("spinning intake wheel at ", targetSpeed);
                intakeMotor.setRpm(targetSpeed);
            }       
        }
        else {
            Logger.log("coasting intake wheel");
            intakeMotor.coast();
        }
    }

    public void disabledInit() {
        Logger.log("stopping intake wheel");
        spinWheel = false;
        intakeMotor.halt();
    }
}
