package frc.robot.subsystems;

import frc.robot.motors.NamedMotor;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for climbing.
 * 
 * Both the extender and the rotator move forward to their limit switches. That means:
 *  - Reset speed is positive (to move forward to the limit)
 *  - Maximum position is where the "known" limit is
 *  - Minimum position is derived by subtracting from that
 * 
 * The extender is at its largest position when the arm is contracted.
 * 
 * The rotator is at its largest position when its vertical.
 */ 
public class ClimberSubsystem {

    /** Reset speed */
    public static final double RESET_SPEED = 0.05;

    /** Deadband around center of joystick to ignore */
    public static final double DEADBAND = 0.1;

    /** Max speed of the extension motor */
    public static final double MAX_EXTENSION_OUTPUT = 0.3;

    /** Rotations per inch for the extension motor */
    public static final double EXTENDER_ROTATIONS_PER_INCH = 4.45813;

    /** Value of extender switch when pressed */
    public static final boolean EXTENDER_SWITCH_PRESSED = true;

    /** Maximum number of rotations of the extender motor */
    public static final double EXTENSION_LIMIT = 95;
    
    /** Max speed of the extension motor */
    public static final double MAX_ROTATION_OUTPUT = 0.2;

    /** Value of rotator switch when pressed */
    public static final boolean ROTATOR_SWITCH_PRESSED = true;

    /** Maximum number of rotations of the rotator motor */
    public static final double ROTATION_LIMIT = 104;

    private final XboxController controller;
    private final NamedMotor extenderMotor;
    private final NamedMotor rotatorMotor;
    private final DigitalInput extenderSwitch;
    private final DigitalInput rotatorSwitch;
    private double extenderMax;
    private double extenderMin;
    private double rotatorMin;
    private double rotatorMax;
    private boolean resetting;

    public ClimberSubsystem(XboxController controller, 
        int extenderMotorPort, int extenderSwitchPort,
        int rotatorMotorPort,  int rotatorSwitchPort) {
        this.controller = controller;
        this.extenderMotor = new NamedMotor("Extender", extenderMotorPort);
        this.extenderSwitch = new DigitalInput(extenderSwitchPort);
        this.rotatorMotor = new NamedMotor("Rotator", rotatorMotorPort);
        this.rotatorSwitch = new DigitalInput(rotatorSwitchPort);

        // default: run without limits (dangerous!)
        this.extenderMax = Double.POSITIVE_INFINITY;
        this.extenderMin = Double.NEGATIVE_INFINITY;
        this.rotatorMax = Double.POSITIVE_INFINITY;
        this.rotatorMin = Double.NEGATIVE_INFINITY;

        disabledInit();
    }

    private boolean atExtenderLimit() {
        return extenderSwitch.get() == EXTENDER_SWITCH_PRESSED;
    }

    private boolean atRotatorLimit() {
        return rotatorSwitch.get() == ROTATOR_SWITCH_PRESSED;
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putNumber("POV Angle", controller.getPOV());
        SmartDashboard.putNumber("Rotator Min", rotatorMin);
        SmartDashboard.putNumber("Rotator Current", rotatorMotor.getPosition());
        SmartDashboard.putNumber("Rotator Max", rotatorMax);
        SmartDashboard.putBoolean("Rotator At Max?", atRotatorLimit());
        SmartDashboard.putNumber("Extender Min", extenderMin);
        SmartDashboard.putNumber("Extender Current", extenderMotor.getPosition());
        SmartDashboard.putNumber("Extender Max", extenderMax);
        SmartDashboard.putBoolean("Extender At Max?", atExtenderLimit());
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("climber: putting climbing system in disabled mode");
        extenderMotor.set(0.0);
        rotatorMotor.set(0.0);
        resetting = false;
    }

    // called when the robot is put into autonomous mode (we will begin to reset)
    public void autonomousInit() {
        resetting = true;
    }

    // updates motor speed for the reset routine. both motors move slowly towards their
    // reset limit. once they hit their switch, they're done and we capture their position
    // as the "zero point". when both are done, we're done resetting.
    public void autonomousPeriodic() {
        if (resetting) {
            boolean done = resetLimits();
            if (done) {
                Logger.log("climber: done resetting");
                resetting = false;
            }
        }
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {

        if (controller.getLeftBumperPressed()) {
            resetting = true;
        }

        if (resetting) {
            boolean done = resetLimits();
            if (done) {
                Logger.log("climber: done resetting");
                resetting = false;
                return;
            }
        }

        boolean extenderAtMax = extenderMotor.getPosition() >= extenderMax || atExtenderLimit();
        boolean extenderAtMin = extenderMotor.getPosition() <= extenderMin;
        double extRate = clean(controller.getLeftY());

        if (extRate > 0.0 && extenderAtMax) {
            Logger.log("climber: extender won't go too high ...");
            extenderMotor.set(0.0);
        }
        else if (extRate < 0.0 && extenderAtMin) {
            Logger.log("climber: extender won't go too low ...");
            extenderMotor.set(0.0);
        } else {
            extRate *= MAX_EXTENSION_OUTPUT;
            extenderMotor.set(extRate);
        }

        boolean rotatorAtMax = rotatorMotor.getPosition() >= rotatorMax || atRotatorLimit();
        boolean rotatorAtMin = rotatorMotor.getPosition() <= rotatorMin;
        double rotRate = clean(controller.getRightY());
        if (rotRate > 0.0 && rotatorAtMax) {
            Logger.log("climber: rotator won't go too high ...");
            rotatorMotor.set(0.0);
        }
        else if (rotRate < 0.0 && rotatorAtMin) {
            Logger.log("climber: rotator won't go too low ...");
            rotatorMotor.set(0.0);
        } else {
            rotRate *= MAX_ROTATION_OUTPUT;
            rotatorMotor.set(rotRate);
        }
    }

    private double clean(double stickValue) {
        double absValue = Math.abs(stickValue);
        if (absValue < DEADBAND) {
            return 0.0;
        }
        return stickValue * absValue;
    }

    // resets the extender and rotator limits by slowly moving them to until the
    // limit switches trip
    private boolean resetLimits() {

        // this gets unset if either the extender or the rotator still needs to move
        boolean done = true;

        // check the extender for hitting its limit switch
        if (atExtenderLimit()) {
            extenderMax = extenderMotor.getPosition();
            extenderMin = extenderMax - EXTENSION_LIMIT;
            extenderMotor.set(0.0);
            Logger.log("climber: finished resetting extender");
        } else {
            extenderMotor.set(RESET_SPEED);
            done = false;
        }

        // check the rotator for hitting its limit switch
        if (atRotatorLimit()) {
            rotatorMax = rotatorMotor.getPosition();
            rotatorMin = rotatorMax - ROTATION_LIMIT;
            rotatorMotor.set(0.0);
            Logger.log("climber: finished resetting rotator");
        } else {
            rotatorMotor.set(RESET_SPEED);
            done = false;
        }

        // if both of those previous guys are done, we're done resetting
        return done;
    }
}
