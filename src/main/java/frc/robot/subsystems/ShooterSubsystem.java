package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.testrobots.RobotPortMap;


/**
 * Subsystem for shooting
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

    private final SpecialOpsController controller;
    private final DigitalInput shotReadySwitch;
    // private final SomethingSomething indexerWheel;
    private final CANSparkMax launchWheel;
    private double maxLaunchSpeed;
    private double targetLaunchSpeed;
    private boolean wheelSpin;
    public double getMaxLaunchSpeed() {
        return maxLaunchSpeed;
    }

    public double getTargetLaunchSpeed() {
        return targetLaunchSpeed;
    }

    public ShooterSubsystem(SpecialOpsController specialOpsController) {
        controller = specialOpsController;
        shotReadySwitch = new DigitalInput(RobotPortMap.SHOOTER_SHOT_READY_PORT);
        launchWheel = makeLaunchWheel(RobotPortMap.SHOOTER_LAUNCH_WHEEL_PORT);
        // indexerWheel = ???
        maxLaunchSpeed = DEFAULT_LAUNCH_SPEED;
        targetLaunchSpeed = 0.0;
    }

    public void robotPeriodic() {
        SmartDashboard.setDefaultNumber("Shooter.maxLaunchSpeed", maxLaunchSpeed);
        SmartDashboard.setDefaultNumber("Shooter.targetLaunchSpeed", targetLaunchSpeed);
        SmartDashboard.setDefaultNumber("Shooter.currentLaunchSpeed", launchWheel.get());
    }

    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public boolean teleopPeriodic() {
        if (controller.getYButtonPressed()) {
            maxLaunchSpeed = maxLaunchSpeed+.1;
             //SmartDashboard.putBoolean("Was A Pressed", controller.getYButtonPressed());
    
        }
        if (controller.getXButtonPressed()) {
             maxLaunchSpeed = maxLaunchSpeed-.1;
             //SmartDashboard.putBoolean("Was B Pressed", controller.getXButtonPressed());
        } 

        if (controller.getBackButtonPressed()) {
            wheelSpin = !wheelSpin;
        }
        if (wheelSpin) {
            targetLaunchSpeed = maxLaunchSpeed;
        }
        else {
            targetLaunchSpeed = 0;
        }
        launchWheel.set(.7);



        // because that will determine which camera we use
        return true;
    }


    public void disabledInit() {
        // what should happen here?
    }

    private CANSparkMax makeLaunchWheel(int port) {
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.setClosedLoopRampRate(RAMP_RATE);
        motor.setOpenLoopRampRate(RAMP_RATE);
        motor.setIdleMode(IDLE_MODE);
        return motor;
    }
}