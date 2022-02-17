package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.util.MathUtil;
import frc.robot.util.PIDSettingsUpdater;

/**
 * Subsystem for ball intake.
 */
public class IntakeSubsystem {

    public static final double INTAKE_VELOCITY = 120;

    private final SpecialOpsController controller;
    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkMaxPIDController intakePidController;
    private final PIDSettingsUpdater intakeSettings;
    private double targetVelocity;
  
    public IntakeSubsystem(SpecialOpsController controller) {

        this.controller = controller;

        // create the intake motor normally
        intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setClosedLoopRampRate(2.0);

        // these get created when you first access them like this
        intakeEncoder = intakeMotor.getEncoder();
        intakePidController = intakeMotor.getPIDController();

        // this is the thing that will set up the PID controller
        intakeSettings = new PIDSettingsUpdater("Intake", intakePidController);

        targetVelocity = 0.0;
    }

    public void robotPeriodic() {

        // every 50ms, we'll sync our PID settings with the dashboard
        intakeSettings.updateFromDashboard();

        // we'll also report our wheel's target and current velocity
        SmartDashboard.putNumber("Intake Target Velocity", targetVelocity);
        SmartDashboard.putNumber("Intake Current Velocity", intakeEncoder.getVelocity());
    }

    public void teleopInit(TeleopMode newMode) {        
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {

        // This sets the target velocity to 120 rpm (2 revolutions per second), and
        // then tells the PID controller to make sure the motor stays at that speed.

        targetVelocity = 120.0;

        intakePidController.setReference(targetVelocity, ControlType.kVelocity);

    }

    /**
     * You can use the "encoder" to read the motor's position and velocity. That way
     * you can tell whether it's at the target value.
     * 
     * You don't want this comparison to be too precise - the motor will be kept "near"
     * the target value, but it's not likely to be exactly right to all decimal places.
     * 
     * @return true if the motor is within +/- 5% of the target
     */
    public boolean isAtFullSpeed() {
        double currentVelocity = intakeEncoder.getVelocity();
        return MathUtil.equalsWithinDelta(currentVelocity,  targetVelocity, 0.05);
    }

    public void disabledInit() {
        
        // stops the motor
        targetVelocity = 0;
        intakeMotor.stopMotor();      
    }
}