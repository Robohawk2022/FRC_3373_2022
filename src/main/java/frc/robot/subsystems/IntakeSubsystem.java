package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.IntakeConfig;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {
    
    private final XboxController controller;
    private final IntakeConfig config;
    private final VelocityClosedLoopMotor intakeMotor;
    private boolean spinWheel;
    private double targetSpeed;

    public IntakeSubsystem(XboxController controller, IntakeConfig config) {
        this.controller = controller;
        this.config = config;
        this.intakeMotor = MotorFactory.makeVelocityClosedLoopMotor("Intake", config.intakePort);
        disabledInit();
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putNumber("Intake Target Speed", targetSpeed);
        SmartDashboard.putBoolean("Intake Spinning?", spinWheel);
    }
    
    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("putting intake system in disabled mode");
        spinWheel = false;
        targetSpeed = config.intakeStartingRpm * config.intakeRpmScale;
        intakeMotor.halt();
    }

    // called 50x per second in teleop mode
    public void telopPeriodic() {

        // back button turns the wheel on and off
        if (controller.getRawButtonPressed(config.startStopButton)) {
            spinWheel = !spinWheel;
            Logger.log("toggled intake wheel to ", spinWheel);
        }

        // if the wheel is spinning, we'll allow speed changes
        if (spinWheel) {

            // reset speed
            if (controller.getRawButtonPressed(config.resetSpeedButton)) {
                targetSpeed = config.intakeStartingRpm;
                Logger.log("reset launch wheel to ", targetSpeed);
            }
            // go 10% slower
            else if (controller.getRawButtonPressed(config.slowDownButton)) {
                targetSpeed *= 0.9;
                Logger.log("slowed down intake wheel to ", targetSpeed);
            }
            // go 10% faster
            else if (controller.getRawButtonPressed(config.speedUpButton)) {
                targetSpeed *= 1.1;
                Logger.log("sped up intake wheel to ", targetSpeed);
            }

            // hreverse the wheel
            if (controller.getRawAxis(config.reverseTrigger) > 0.5) {
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
}
