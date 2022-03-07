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

        if (controller.getBButtonPressed()) {
            System.err.println("climber: toggling reset mode");
            resetting = !resetting;
        }

        double extRate = 0.0;
        double rotRate = 0.0;

        if (resetting) {
            extRate = RESET_SPEED;
            rotRate = RESET_SPEED;
            if (atExtenderLimit() && atRotatorLimit()) {
                System.err.println("climber: done w/ reset");
                resetting = false;
            }
        }
        else {
            extRate = clean(controller.getLeftY());
            rotRate = clean(controller.getRightX());
        }

        if (extRate != 0.0) {
            if (extRate < 0.0 && atExtenderLimit()) {
                extRate = 0.0;
            }
            extRate *= MAX_EXTENSION_OUTPUT;
            System.err.println("climber: extending at "+extRate);
        }
        extenderMotor.set(extRate);
    
        if (rotRate != 0.0) {
            if (rotRate > 0.0 && atRotatorLimit()) {
                rotRate = 0.0;
            }
            rotRate *= MAX_ROTATION_OUTPUT;
            System.err.println("climber: rotating at "+rotRate);
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
