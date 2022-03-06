package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    /** Starting value for the speed of the intake wheel */
    public static final double STARTING_RPM = 2000;

    /** Speed value for dropping the frame */
    public static final double DROP_FRAME_SPEED = 0.1;

    /** Amount of time for dropping the frame */
    public static final double DROP_FRAME_SECONDS = 3.0;
    
    private final XboxController controller;
    private final VelocityClosedLoopMotor intakeMotor;
    private boolean spinWheel;
    private double targetSpeed;
    private double autonomousStart;

    public IntakeSubsystem(XboxController controller, int intakeMotorPort) {
        this.controller = controller;
        this.intakeMotor = MotorFactory.makeVelocityClosedLoopMotor("Intake", intakeMotorPort);
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
        targetSpeed = STARTING_RPM;
        autonomousStart = -1L;
        intakeMotor.halt();
    }

    // called at the beginning of autonomous
    public void autonomousInit() {
        autonomousStart = Timer.getFPGATimestamp();
    }

    // called 50x per second in autonomous
    public void autonomousPeriodic() {
        double now = Timer.getFPGATimestamp();
        if (now - Timer.getFPGATimestamp() < DROP_FRAME_SECONDS) {
            intakeMotor.set(DROP_FRAME_SPEED);
        } else {
            intakeMotor.set(0.0);
        }
    }

    // called 50x per second in teleop mode
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
                targetSpeed = STARTING_RPM;
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
            intakeMotor.coast();
        }
    }
}
