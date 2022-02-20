package frc.robot.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.PIDConstant;

public class MotorFactory {

    public static final PIDConstant DEFAULT_PID = new PIDConstant(0.3, 0, 1.0, 0.0, 0.0, -1.0, 1.0);

    private static final List<CANSparkMax> ALL_MOTORS = new ArrayList<>();
    private static final List<MotorDashboardUpdater> ALL_UPDATERS = new ArrayList<>();
    
    private static Motor wrapOpenLoopMotor(CANSparkMax motor) {        
        return new Motor() {

            double lastSetPoint = 0.0;

            public void set(double setPoint) {
                motor.set(setPoint);
                lastSetPoint = setPoint;
            }

            public double get() {
                return lastSetPoint;
            }
        };
    }

    private static Motor wrapPositionMotor(CANSparkMax motor) {
        final RelativeEncoder encoder = motor.getEncoder();
        final SparkMaxPIDController controller = motor.getPIDController();
        return new Motor() {

            public void set(double setPoint) {
                controller.setReference(setPoint, ControlType.kPosition);
            }

            public double get() {
                return encoder.getPosition();
            }
        };
    }

    private static Motor wrapVelocityMotor(CANSparkMax motor) {
        final RelativeEncoder encoder = motor.getEncoder();
        final SparkMaxPIDController controller = motor.getPIDController();
        return new Motor() {

            public void set(double setPoint) {
                controller.setReference(setPoint, ControlType.kVelocity);
            }

            public double get() {
                return encoder.getPosition();
            }
        };
    }

    public static Motor createMotor(MotorSettings settings) {

        Objects.requireNonNull(settings);
        Objects.requireNonNull(settings.name, "a name is required");
        if (settings.port < 0) {
            throw new IllegalArgumentException("bad port number: "+settings.port);
        }

        CANSparkMax canSpark = new CANSparkMax(settings.port, MotorType.kBrushless);
        canSpark.restoreFactoryDefaults();
        canSpark.setIdleMode(settings.allowCoasting ? IdleMode.kCoast : IdleMode.kBrake);
        ALL_MOTORS.add(canSpark);

        Motor motor = null;
        MotorDashboardUpdater updater = null;

        switch (settings.type) {

            case OPEN_LOOP:
                canSpark.setOpenLoopRampRate(settings.rampRate);
                motor = wrapOpenLoopMotor(canSpark);
                updater = new MotorDashboardUpdater(settings.name, canSpark, null);
                break;

            case VELOCITY:
                canSpark.setClosedLoopRampRate(settings.rampRate);
                motor = wrapVelocityMotor(canSpark);
                updater = new MotorDashboardUpdater(settings.name, canSpark, DEFAULT_PID);
                break;

            case POSITION:
                canSpark.setClosedLoopRampRate(settings.rampRate);
                motor = wrapPositionMotor(canSpark);
                updater = new MotorDashboardUpdater(settings.name, canSpark, DEFAULT_PID);
                break;
        }

        ALL_MOTORS.add(canSpark);
        ALL_UPDATERS.add(updater);
        return motor;
    }

    public static void updateDashboard() {
        for (MotorDashboardUpdater updater : ALL_UPDATERS) {
           // updater.updateDashboard();
        }
    }

    public static void simulationInit() {
        for (CANSparkMax canSpark : ALL_MOTORS) {
            REVPhysicsSim.getInstance().addSparkMax(canSpark, DCMotor.getNEO(canSpark.getDeviceId()));
        }
    }
}
