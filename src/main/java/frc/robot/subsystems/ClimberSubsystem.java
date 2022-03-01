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
    public static final double MAX_EXTENSION_OUTPUT = 0.7;

    /** Max speed of the extension motor */
    public static final double MAX_ROTATION_OUTPUT = 0.7;

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

    public ClimberSubsystem(XboxController controller, 
        int extenderMotorPort, int extenderSwitchPort,
        int rotatorMotorPort,  int rotatorSwitchPort) {
        this.controller = controller;
        this.extenderMotor = new NamedMotor("Extender", extenderMotorPort);
        this.extenderSwitch = new DigitalInput(extenderSwitchPort);
        this.rotatorMotor = new NamedMotor("Rotator", rotatorMotorPort);
        this.rotatorSwitch = new DigitalInput(rotatorSwitchPort);
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
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("putting climbing system in disabled mode");
        extenderMotor.set(0.0);
        rotatorMotor.set(0.0);
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {

        double extensionOutput = clean(controller.getLeftY()) * MAX_EXTENSION_OUTPUT;
        if (extensionOutput < 0.0 && atExtenderLimit()) {
            extensionOutput = 0.0;
        }
        extenderMotor.set(extensionOutput);

        double rotatorOutput = clean(controller.getLeftY()) * MAX_ROTATION_OUTPUT;
        if (rotatorOutput < 0.0 && atRotatorLimit()) {
            rotatorOutput = 0.0;
        }
        rotatorMotor.set(rotatorOutput);
    }

    private double clean(double stickValue) {
        double absValue = Math.abs(stickValue);
        if (absValue < DEADBAND) {
            return 0.0;
        }
        return stickValue * absValue;
    }
}
