package frc.robot.subsystems;

import frc.robot.motors.NamedMotor;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for climbing
 * TODO calibrate RPM for extension/rotation motors
 * TODO write routine to reset limits
 */ 
public class ClimberSubsystem {

    /*
     * SOMEWHAT IMPORTANT
     * 
     *   - For the extender
     *     - Setting the motor to positive motion will LOWER the arm
     *     - The limit switch is at the bottom (maximum positive position value)
     */

    /** Rotations per inch for the extension motor */
    public static final double EXTENDER_ROTATIONS_PER_INCH = 4.45813;

    /** Max speed of the extension motor */
    public static final double MAX_EXTENSION_OUTPUT = 0.3;

    /** Max speed of the extension motor */
    public static final double MAX_ROTATION_OUTPUT = 0.6;

    /** Reset speed (negative for backwards) */
    public static final double RESET_SPEED = -0.05;

    /** Value of extender switch when pressed */
    public static final boolean EXTENDER_SWITCH_PRESSED = true;

    /** Value of rotator switch when pressed */
    public static final boolean ROTATOR_SWITCH_PRESSED = true;

    /** Deadband around center of joystick to ignore */
    public static final double DEADBAND = 0.1;

    /**Total Length in Inches of Extender Arm and Math */
    public static final double TOTALLENGTH = 0; 
    public static double ExtHeight = (TOTALLENGTH * EXTENDER_ROTATIONS_PER_INCH);
    /**Extender Arm Limited */
    public static final double MAXHEIGHT = ExtHeight;

    public static final double MAX_EXTENSION = 95;
    

    private final XboxController controller;
    private final NamedMotor extenderMotor;
    private final NamedMotor rotatorMotor;
    private final DigitalInput extenderSwitch;
    private final DigitalInput rotatorSwitch;
    private double extenderMax;
    private double extenderMin;
    private double rotatorZero;
    private boolean resetting;

    public ClimberSubsystem(XboxController controller, 
        int extenderMotorPort, int extenderSwitchPort,
        int rotatorMotorPort,  int rotatorSwitchPort) {
        this.controller = controller;
        this.extenderMotor = new NamedMotor("Extender", extenderMotorPort);
        this.extenderSwitch = new DigitalInput(extenderSwitchPort);
        this.rotatorMotor = new NamedMotor("Rotator", rotatorMotorPort);
        this.rotatorSwitch = new DigitalInput(rotatorSwitchPort);
        this.extenderMax = Double.NEGATIVE_INFINITY;
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
        SmartDashboard.putBoolean("Extender Max?", atExtenderLimit());
        SmartDashboard.putNumber("Extender Min", extenderMin);
        SmartDashboard.putBoolean("Rotator Limit?", atRotatorLimit());
        SmartDashboard.putNumber("Rotator Zero", rotatorZero);
        SmartDashboard.putNumber("Extender Max", extenderMax);
        extenderMotor.updateDashboard();
        rotatorMotor.updateDashboard();
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("climber: putting climbing system in disabled mode");
        extenderMotor.set(0.0);
        rotatorMotor.set(0.0);
        resetting = false;
    }

    public void autonomousInit() {
        resetting = true;
    }

    // updates motor speed for the reset routine. both motors move slowly towards their
    // reset limit. once they hit their switch, they're done and we capture their position
    // as the "zero point". when both are done, we're done resetting.
    public void autonomousPeriodic() {

        // if we're done resetting, bail out
        if (!resetting) {
            return;
        }

        boolean done = true;

        // if we haven't captured a max value yet, we check the extender
        if (atExtenderLimit()) {
            extenderMax = extenderMotor.getPosition();
            extenderMin = extenderMax - MAX_EXTENSION;
            extenderMotor.set(0.0);
            Logger.log("climber: finished resetting extender; max=", extenderMax);
        } else {
            extenderMotor.set(-RESET_SPEED);
            done = false;
        }

        if (done) {
            Logger.log("climber: done resetting; extenderMax=", extenderMax, ", rotatorZero=", rotatorZero);
            resetting = false;
        }
    }
    // called 50x per second in teleop mode
    public void teleopPeriodic() {

        // for the extender:
        //   - forward on the joystick means a negative rate, which sends the arm higher
        //   - backward on the joystick means a positive rate, which sends the arm lower
        //   - we don't want to go backwards past the limit

        boolean extenderAtMax = extenderMotor.getPosition() >= extenderMax || atExtenderLimit();
        boolean extenderAtMin = extenderMotor.getPosition() <= extenderMin;
        Logger.log("checking extender; atMax=", extenderAtMax, ", atMin=", extenderAtMin);

        double extRate = clean(controller.getLeftY());
        if (extRate > 0.0 && extenderAtMax) {
            Logger.log("climber: extender won't go too low ...");
            extenderMotor.set(0.0);
        }
        else if (extRate < 0.0 && extenderAtMin) {
            Logger.log("climber: extender won't go too high ...");
            extenderMotor.set(0.0);
        } else {
            extRate *= MAX_EXTENSION_OUTPUT;
            extenderMotor.set(extRate);
        }

        // double rotRate = clean(controller.getRightX());
        // if (rotRate != 0.0) {
        //     if (rotRate > 0.0 && atRotatorLimit()) {
        //         rotRate = 0.0;
        //     }
        //     rotRate *= MAX_ROTATION_OUTPUT;
        //     Logger.log("climber: rotating at ", rotRate);
        // }
        // rotatorMotor.set(rotRate);



    }





    private double clean(double stickValue) {
        double absValue = Math.abs(stickValue);
        if (absValue < DEADBAND) {
            return 0.0;
        }
        return stickValue * absValue;
    }
}
