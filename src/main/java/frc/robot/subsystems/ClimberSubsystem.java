package frc.robot.subsystems;

import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for climbing
 * TODO calibrate RPM for extension/rotation motors
 * TODO write routine to reset limits
 */ 
public class ClimberSubsystem {

    /** Max speed of the extension motor */
    public static final double MAX_EXTENSION_RPM = 4000;

    /** Max speed of the extension motor */
    public static final double MAX_ROTATION_RPM = 4000;

    private final XboxController controller;
    private final VelocityClosedLoopMotor extenderMotor;
    private final VelocityClosedLoopMotor rotatorMotor;
    private double minRotation;
    private double maxRotation;
    private double minExtension;
    private double maxExtension;

    public ClimberSubsystem(XboxController specialOpsController, int extenderMotorPort, int rotatorMotorPort) {
        controller = specialOpsController;
        extenderMotor = new VelocityClosedLoopMotor("Extender", extenderMotorPort);
        rotatorMotor = new VelocityClosedLoopMotor("Rotator", rotatorMotorPort);
        setLimits();
    }

    private void setLimits() {
        minRotation = Double.NEGATIVE_INFINITY;
        maxRotation = Double.POSITIVE_INFINITY;
        minExtension = Double.NEGATIVE_INFINITY;
        maxExtension = Double.POSITIVE_INFINITY;
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putNumber("Climber Max Rotation", maxRotation);
        SmartDashboard.putNumber("Climber Min Rotation", minRotation);
        SmartDashboard.putNumber("Climber Max Extension", maxExtension);
        SmartDashboard.putNumber("Climber Min Extension", minExtension);
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("putting climbing system in disabled mode");
        extenderMotor.halt();
        rotatorMotor.halt();
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {
        updateExtender();
        updateRotator();
    }

    private void updateExtender() {
        double extensionRate = controller.getLeftY() * MAX_EXTENSION_RPM;
        double currentExtensionPosition = extenderMotor.getPosition();

        if (extensionRate > 0.0 && currentExtensionPosition < maxExtension) {
            extenderMotor.setRpm(extensionRate);
        }
        else if (extensionRate < 0.0 && currentExtensionPosition > minExtension) {
            extenderMotor.setRpm(extensionRate);
        }
        else {
            extenderMotor.halt();
        }
    }

    private void updateRotator() {
        double rotationRate = controller.getRightX() * MAX_ROTATION_RPM;
        double currentRotationPosition = rotatorMotor.getPosition();

        if (rotationRate > 0.0 && currentRotationPosition < maxRotation) {
            rotatorMotor.setRpm(rotationRate);
        }
        else if (rotationRate < 0.0 && currentRotationPosition > minRotation) {
            rotatorMotor.setRpm(rotationRate);
        }
        else {
            rotatorMotor.halt();
        }
    }
}
