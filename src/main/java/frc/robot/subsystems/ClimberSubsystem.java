package frc.robot.subsystems;

import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.motors.VelocityClosedLoopMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Subsystem for climbing
 * @param <motorOne>
 */ 
public class ClimberSubsystem<motorOne> {

    public static final double MAX_EXTENSION_RPM = 4000; // TODO calibrate with arm on robot
    public static final double MAX_ROTATION_RPM = 4000; // TODO calibrate with arm on robot

    private final XboxController controller;
    private final VelocityClosedLoopMotor extenderMotor;
    private final VelocityClosedLoopMotor rotatorMotor;
    private double minRotation;
    private double maxRotation;
    private double minExtension;
    private double maxExtension;

    public ClimberSubsystem(XboxController specialOpsController) {
        controller = specialOpsController;
        extenderMotor = new VelocityClosedLoopMotor("Extender", 1);
        rotatorMotor = new VelocityClosedLoopMotor("Rotator", 3);
        setLimits();
    }

    private void setLimits() {
         // TODO write limit reset routine
         minRotation = Double.NEGATIVE_INFINITY;
        maxRotation = Double.POSITIVE_INFINITY;
        minExtension = Double.NEGATIVE_INFINITY;
        maxExtension = Double.POSITIVE_INFINITY;
    }

    public void robotPeriodic() {
        // what should happen here?
    }

    public void teleopInit(TeleopMode newMode) {
        // what should happen here?
    }

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

    public void disabledInit() {
        // what should happen here?
    }
}
