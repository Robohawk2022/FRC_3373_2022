package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.motors.Motor;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.MotorSettings;
import frc.robot.motors.MotorSettings.Type;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    public static final double STARTING_MAX = 1200;
    
    private final SpecialOpsController controller;
    private final Motor intakeMotor;
    private double targetSpeed;
    private double maxSpeed;
    private boolean spinWheel;

    public IntakeSubsystem(SpecialOpsController controller) {
        this.controller = controller;
        this.intakeMotor = createMotor();
        this.maxSpeed = STARTING_MAX;
        this.targetSpeed = 0.0;
        this.spinWheel = false;

        SmartDashboard.putNumber("Intake Max Speed", maxSpeed);
    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("Intake Target Speed", targetSpeed);
        double newMax = SmartDashboard.getNumber("Intake Max Speed", maxSpeed);
        if (newMax != maxSpeed) {
            System.err.println("setting max speed to "+newMax);
            maxSpeed = newMax;
        }
    }

    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {

        if (controller.getBackButtonPressed()) {
            spinWheel = !spinWheel;
        }
        if (spinWheel) {
            targetSpeed = maxSpeed;
        }
        else {
            targetSpeed = 0.0;
        }

        if (controller.getLeftTriggerAxis() > 0.5) {
            intakeMotor.set(-targetSpeed);
        }
        else {
            intakeMotor.set(targetSpeed);
        }       
    }

    public void disabledInit() {
        targetSpeed = 0.0;
        intakeMotor.set(0.0);
    }

    private Motor createMotor() {
        MotorSettings settings = new MotorSettings();
        settings.port = 1;
        settings.name = "Intake";
        settings.rampRate = 2.0;
        settings.type = Type.VELOCITY;
        return MotorFactory.createMotor(settings);
    }
}
