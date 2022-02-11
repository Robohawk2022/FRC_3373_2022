package frc.robot;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TesterSystem {

    public static final double DEADBAND = 0.1;

    private final Map<String,CANSparkMax> motors = new HashMap<>();
    private final SendableChooser<CANSparkMax> chooser = new SendableChooser<>();

    public void addMotor(String name, CANSparkMax motor) {
        motors.put(name, motor);
        chooser.addOption(name, motor);
    }

    public Collection<CANSparkMax> getAllMotors() {
        return motors.values();
    }
    
    public SendableChooser<CANSparkMax> getChooser() {
        return chooser;
    }
    
    public void update(HawkbotControls controls) {

        CANSparkMax motor = chooser.getSelected();
        if (motor == null) {
            SmartDashboard.putNumber("MotorUnderTest/get", -1.23);
            SmartDashboard.putNumber("MotorUnderTest/getPosition", -1.23);
            SmartDashboard.putNumber("MotorUnderTest/getVelocity", -1.23);
            return;
        }

        if (controls.wasTestStopRequested()) {
            motor.set(0.0);
        } else if (controls.wasTestSpeedUpRequested()) {
            motor.set(motor.get() + 0.05);
        } else if (controls.wasTestSlowDownRequested()) {
            motor.set(motor.get() - 0.05);
        } else {
            double speed = deadbandSquared(controls.getTestSpeed());
            if (speed != 0.0) {
                motor.set(speed);
            }
        }

        SmartDashboard.putNumber("MotorUnderTest/get", motor.get());
        SmartDashboard.putNumber("MotorUnderTest/getPosition", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("MotorUnderTest/getVelocity", motor.getEncoder().getVelocity());
    }
    
    private double deadbandSquared(double x) {
        if (x < -DEADBAND) {
            return x * -x;
        }
        if (x > DEADBAND) {
            return x * x;
        }
        return 0.0;
    }
}
