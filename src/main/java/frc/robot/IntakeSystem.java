package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSystem {

    public static final double DEFAULT_SPEED = 0.6;

    private final CANSparkMax intakeMotor;
    private double speed;

    public IntakeSystem(int intakeMotorPort) {
        intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushless);
        speed = DEFAULT_SPEED;
    }

    public void collectMotors(TesterSystem tester) {
        tester.addMotor("IntakeMotor", intakeMotor);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("IntakeMotor/TargetSpeed", speed);
        SmartDashboard.putNumber("IntakeMotor/CurrentSpeed", intakeMotor.get());
        SmartDashboard.putNumber("IntakeMotor/Position", intakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("IntakeMotor/Velocity", intakeMotor.getEncoder().getVelocity());
    }

    public void update(HawkbotControls controls) {
        if (controls.wasIntakeReverseRequested()) {
            speed = -speed;
            System.err.println("intake wheel set to "+speed);
        }
        intakeMotor.set(speed);
    }

    public void stopAll() {
        intakeMotor.stopMotor();
    }
}
