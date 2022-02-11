package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSystem {

    public static final double DEFAULT_SPEED = 0.6;

    public static final double SPEED_INCREMENT = 0.05;

    private final CANSparkMax launchMotor;
    private final CANSparkMax indexerMotor;
    private final DigitalInput ballReadySwitch;
    private double launchSpeed;
    private int shotCount;

    public ShooterSystem(int launchMotorPort, int indexerMotorPort, int ballReadyPort) {
        launchMotor = new CANSparkMax(launchMotorPort, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(indexerMotorPort, MotorType.kBrushless);
        ballReadySwitch = new DigitalInput(ballReadyPort);
        launchSpeed = DEFAULT_SPEED;
        shotCount = 0;
    }

    public void collectMotors(TesterSystem tester) {
        tester.addMotor("LaunchMotor", launchMotor);
        tester.addMotor("IndexerMotor", indexerMotor);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Shooter/LaunchSetSpeed", launchMotor.get());
        SmartDashboard.putNumber("Shooter/LaunchVelocity", launchMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/IndexPosition", indexerMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Shooter/BallReader", ballReadySwitch.get());
        SmartDashboard.putNumber("Shooter/ShotCount", shotCount);
    }

    public void update(HawkbotControls controls) {

        if (controls.wasLaunchSpeedIncreaseRequested()) {
            launchSpeed += SPEED_INCREMENT;
            System.err.println("launch speed increased to "+launchSpeed);
        }
        else if (controls.wasLaunchSpeedDecreaseRequested()) {
            launchSpeed -= SPEED_INCREMENT;
            System.err.println("launch speed decreased to "+launchSpeed);
        }
        launchMotor.set(launchSpeed);

    }

    public void stopAll() {
        launchMotor.stopMotor();
        indexerMotor.stopMotor();
    }
}
