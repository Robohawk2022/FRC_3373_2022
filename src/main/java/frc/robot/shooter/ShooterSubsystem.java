package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotPortMap;

/**
 * The shooter subsystem has five components:
 * 
 *   - An "intake ready" switch to determine when a ball is waiting for intake
 *   - An "intake wheel" to suck the ball into the robot
 *   - A "shot ready" switch to determine when a ball is waiting to be shot
 *   - A "launch wheel" to launch the ball
 *   - An "indexer wheel" to push the ball in front of the launch wheel
 * 
 * TODO is this layout correct? (double check switches)
 */
public class ShooterSubsystem {

    /** Default launch speed (can be adjusted in realtime using controller) */
    public static final double DEFAULT_LAUNCH_SPEED = 0.7;

    /** How much should the launch speed change when someone uses the controller? */
    public static final double SPEED_CHANGE_AMOUNT = 0.1;
    
    /** How long (in seconds) should it take the motor to spin to max power? */
    public static final double RAMP_RATE = 2.0;

    /** What should the motor do when power is set to 0? */
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    private final ShooterController controller;
    private final DigitalInput intakeReadySwitch;
    private final DigitalInput shotReadySwitch;
    // private final XXX intakeWheel;  // TODO what type of component is the intake?
    // private final XXX indexerWheel;  // TODO what type of component is the indexer?
    private final MotorController launchWheel;
    private double maxLaunchSpeed;
    private double targetLaunchSpeed;

    public double getMaxLaunchSpeed() {
        return maxLaunchSpeed;
    }

    public double getTargetLaunchSpeed() {
        return targetLaunchSpeed;
    }
    
    public ShooterSubsystem() {
        this.controller = new ShooterController(RobotPortMap.SHOOTER_CONTROLLER_PORT);
        this.intakeReadySwitch = new DigitalInput(RobotPortMap.SHOOTER_INTAKE_READY_PORT);
        this.shotReadySwitch = new DigitalInput(RobotPortMap.SHOOTER_SHOT_READY_PORT);
        this.launchWheel = makeLaunchWheel(RobotPortMap.SHOOTER_LAUNCH_WHEEL_PORT);
        this.maxLaunchSpeed = DEFAULT_LAUNCH_SPEED;
        this.targetLaunchSpeed = 0.0;
    }

    public void updateDashboard() {
        SmartDashboard.setDefaultNumber("Shooter.maxLaunchSpeed", maxLaunchSpeed);
        SmartDashboard.setDefaultNumber("Shooter.targetLaunchSpeed", targetLaunchSpeed);
        SmartDashboard.setDefaultNumber("Shooter.currentLaunchSpeed", launchWheel.get());
    }

    public void updateControls() {

        if (controller.wasIntakeRequested()) {
            // TODO what should happen here?
        }
        if (controller.wasLaunchSpeedIncreaseRequested()) {
            // TODO what should happen here?
        }
        if (controller.wasLaunchSpeedDecreaseRequested()) {
            // TODO what should happen here?
        }
        if (controller.isLaunchWheelActive()) {
            // TODO what should happen here?
        }
        if (controller.wasShotRequested()) {
            // TODO what should happen here?
        }

        launchWheel.set(targetLaunchSpeed);
    }

    public void disable() {
        // what should happen here?
    }

    private MotorController makeLaunchWheel(int port) {
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.setClosedLoopRampRate(RAMP_RATE);
        motor.setOpenLoopRampRate(RAMP_RATE);
        motor.setIdleMode(IDLE_MODE);
        return motor;
    }
}