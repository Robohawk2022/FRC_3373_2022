package frc.robot.subsystems;

import frc.robot.config.ClimberConfig;
import frc.robot.motors.NamedMotor;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for climbing
 */ 
public class ClimberSubsystem {

    private final XboxController controller;
    private final ClimberConfig config;
    private final NamedMotor extenderMotor;
    private final DigitalInput extenderLimitSwitch;
    private final NamedMotor rotatorMotor;
    private final DigitalInput rotatorLimitSwitch;

    public ClimberSubsystem(XboxController controller, ClimberConfig config) {
        this.controller = controller;
        this.config = config;
        this.extenderMotor = new VelocityClosedLoopMotor("Extender", config.extenderPort);
        this.extenderLimitSwitch = new DigitalInput(config.extenderLimitPort);
        this.rotatorMotor = new VelocityClosedLoopMotor("Rotator", config.rotatorPort);
        this.rotatorLimitSwitch = new DigitalInput(config.rotatorLimitPort);
    }

    public boolean atExtenderLimit() {
        return extenderLimitSwitch.get() == config.extenderLimitPressedValue;
    }

    public boolean atRotatorLimit() {
        return rotatorLimitSwitch.get() == config.rotatorLimitPressedValue;
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putBoolean("At Extender Limit?", atExtenderLimit());
        SmartDashboard.putBoolean("At Rotator Limit?", atRotatorLimit());
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("putting climbing system in disabled mode");
        extenderMotor.set(0.0);
        rotatorMotor.set(0.0);
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {
        updateExtender();
        updateRotator();
    }

    private void updateExtender() {

        double extensionRate = computeOutput(            
            controller.getRawAxis(config.extenderAxis), 
            config.extenderDeadband,
            config.extenderMaxOutput);

        // stop extension if we're at the limit
        if (extensionRate < 0.0 && atExtenderLimit()) {
            extensionRate = 0.0;
        }

        extenderMotor.set(extensionRate);
    }

    private void updateRotator() {

        double rotationRate = computeOutput(            
            controller.getRawAxis(config.rotatorAxis), 
            config.rotatorDeadband,
            config.rotatorMaxOutput);

        // stop rotation if we're at the limit
        if (rotationRate < 0.0 && atRotatorLimit()) {
            rotationRate = 0.0;
        }

        rotatorMotor.set(rotationRate);
    }

    // converts stick input into an output rate. three considerations:
    //   - ignore very small inputs so we're not sensitive to random bumps
    //   - square input so we speed up gradually
    //   - don't lose the sign value (pos/neg)
    private double computeOutput(double stickValue, double deadband, double maxOutput) {
        double absStick = Math.abs(stickValue);
        if (absStick < deadband) {
            return 0.0;
        }
        return (absStick * stickValue) * maxOutput;
    }
}
