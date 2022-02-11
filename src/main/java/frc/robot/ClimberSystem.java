package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSystem {

    private final CANSparkMax hookMotor;
    private final CANSparkMax extensionMotor;
    private final CANSparkMax rotateMotor;

    public ClimberSystem(int hookMotorPort, int extensionMotorPort, int rotateMotorPort) {
        hookMotor = new CANSparkMax(hookMotorPort, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(extensionMotorPort, MotorType.kBrushless);
        rotateMotor = new CANSparkMax(rotateMotorPort, MotorType.kBrushless);
    }

    public void collectMotors(TesterSystem tester) {
        tester.addMotor("HookMotor", hookMotor);
        tester.addMotor("ExtensionMotor", extensionMotor);
        tester.addMotor("RotateMotor", rotateMotor);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("HookMotor/CurrentSpeed", hookMotor.get());
        SmartDashboard.putNumber("ExtensionMotor/CurrentSpeed", extensionMotor.get());
        SmartDashboard.putNumber("RotateMotor/CurrentSpeed", rotateMotor.get());
    }

    public void resetMotors() {
        
    }

    public void update(HawkbotControls controls) {

    }

    public void stopAll() {
        hookMotor.stopMotor();
        extensionMotor.stopMotor();
        rotateMotor.stopMotor();
    }
}
