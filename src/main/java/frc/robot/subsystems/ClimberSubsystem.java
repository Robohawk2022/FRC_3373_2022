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

    /** Max speed of the extension motor */
    public static final double MAX_EXTENSION_OUTPUT = 0.2;

    /** Max speed of the extension motor */
    public static final double MAX_ROTATION_OUTPUT = 0.6;

    /** Max speed of the extension motor */
    public static final double RESET_SPEED = -0.1;

    /** Value of extender switch when pressed */
    public static final boolean EXTENDER_SWITCH_PRESSED = true;

    /** Value of rotator switch when pressed */
    public static final boolean ROTATOR_SWITCH_PRESSED = true;

    /** Deadband around center of joystick to ignore */
    public static final double DEADBAND = 0.1;

    private final XboxController controller;
    private final NamedMotor extenderMotor;
    private final NamedMotor rotatorMotor;
    private final DigitalInput extenderSwitch;
    private final DigitalInput rotatorSwitch;
    private double extenderZero;
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
        SmartDashboard.putBoolean("Extender Limit?", atExtenderLimit());
        SmartDashboard.putBoolean("Rotator Limit?", atRotatorLimit());
        SmartDashboard.putNumber("Rotator Zero", rotatorZero);
        SmartDashboard.putNumber("Extender Zero", extenderZero);
        extenderMotor.updateDashboard();
        rotatorMotor.updateDashboard();
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("putting climbing system in disabled mode");
        extenderMotor.set(0.0);
        rotatorMotor.set(0.0);
        resetting = false;
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {

        if (controller.getBackButtonPressed()) {
            System.err.println("climber: toggling reset mode");
            resetting = !resetting;
        }

        if (resetting) {
            periodicForReset();
        } else {
            periodicForJoysticks();
        }
    }

    // updates motor speed for the reset routine. both motors move slowly towards their
    // reset limit. once they hit their switch, they're done and we capture their position
    // as the "zero point". when both are done, we're done resetting.
    private void periodicForReset() {

        boolean done = true;

        if (atExtenderLimit()) {
            extenderMotor.set(0.0);
            extenderZero = extenderMotor.getPosition();
        } else {
            extenderMotor.set(RESET_SPEED);
            done = false;
        }

        if (atRotatorLimit()) {
            rotatorMotor.set(0.0);
            rotatorZero = rotatorMotor.getPosition();
        } else {
            rotatorMotor.set(-RESET_SPEED); // rotator is backwards
            done = false;
        }

        if (done) {
            Logger.log("done resetting; extenderZero=", extenderZero, ", rotatorZero=", rotatorZero);
            resetting = false;
        }
    }

    // updates motor speed for normal joystick controls. both motors can travel freely
    // until they hit their limit switch;
    private void periodicForJoysticks() {

        double extRate = clean(controller.getLeftY());
        double rotRate = clean(controller.getRightX());

        if (extRate != 0.0) {
            if (extRate < 0.0 && atExtenderLimit()) {
                extRate = 0.0;
            }
            extRate *= MAX_EXTENSION_OUTPUT;
            Logger.log("climber: extending at ", extRate);
        }
        extenderMotor.set(extRate);
    
        if (rotRate != 0.0) {
            if (rotRate > 0.0 && atRotatorLimit()) {
                rotRate = 0.0;
            }
            rotRate *= MAX_ROTATION_OUTPUT;
            Logger.log("climber: rotating at ", rotRate);
        }
        rotatorMotor.set(rotRate);
    }

    private double clean(double stickValue) {
        double absValue = Math.abs(stickValue);
        if (absValue < DEADBAND) {
            return 0.0;
        }
        return stickValue * absValue;
    }
}
