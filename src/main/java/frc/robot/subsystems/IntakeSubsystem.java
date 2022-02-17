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

        // deciding the target velocity of the intake wheel is pretty straightforward
        targetVelocity = INTAKE_VELOCITY;
        if (shouldReverse()) {
            targetVelocity = -targetVelocity;
        }

        // this is how you tell the PID controller to spin at a certain velocity
        intakePidController.setReference(targetVelocity, ControlType.kVelocity);
    }

    public void disabledInit() {
        
        // stops the motor
        targetVelocity = 0;
        intakeMotor.stopMotor();      
    }

    private boolean shouldReverse() {
        return controller.getAButton();
    }
}