package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Logger;
import frc.robot.util.PIDConstant;

/**
 * Subsystem for shooting
 */
public class ShooterSubsystem {

    /** PID tuning constants copied from https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java */
    public static final PIDConstant PID_CONSTANT = new PIDConstant(5e-5, 1e-6, 0.0, 0.0, 0.000156, -1.0, 1.0);

    /** Default preset launch speeds */
    public static final double [] LAUNCH_PRESETS = { -3900.0, -4900.0, -5900.0, -6900.0 };

    /** Maximum speed at which to rotate the indexer */
    // IF INDEXER IS REVERSED - negate me
    public static final double INDEXER_MAX_SPEED = 0.5;

    /** Speed tolerance for launch wheel (will only allow shot when w/in this %% of target */
    public static final double LAUNCH_SPEED_TOLERANCE = 0.1;

    /** How many rotations does the indexer need to lock in a ball? */
        // IF INDEXER IS REVERSED - negate me
    public static final double LOCKIN_ROTATIONS = 15;

    /** How many rotations does the indexer need to push out a ball? */
    // IF INDEXER IS REVERSED - negate me
    public static final double SHOOT_ROTATIONS = 40;

    private final XboxController controller;
    private final DigitalInput ballSensor;

    private final CANSparkMax launchMotor;
    private final RelativeEncoder launchEncoder;
    private final SparkMaxPIDController launchController;
    private double launchTargetRpm;

    private final CANSparkMax indexerMotor;
    private final RelativeEncoder indexerEncoder;
    private double indexerTargetPos;

    private boolean spinLaunchWheel;
    private int autoShotsPending;

    public ShooterSubsystem(XboxController controller, 
            int launchMotorPort, 
            int indexerMotorPort, 
            int ballAvailableSwitchPort) {

        System.err.println("initializing shooter");

        this.controller = controller;
        
        ballSensor = new DigitalInput(ballAvailableSwitchPort);

        launchMotor = new CANSparkMax(launchMotorPort, MotorType.kBrushless);
        launchMotor.restoreFactoryDefaults();
        launchMotor.setIdleMode(IdleMode.kCoast);
        launchMotor.setClosedLoopRampRate(0.5);
        launchEncoder = launchMotor.getEncoder();
        launchController = launchMotor.getPIDController();
        PID_CONSTANT.configPID(launchController);

        indexerMotor = new CANSparkMax(indexerMotorPort, MotorType.kBrushless);
        indexerMotor.restoreFactoryDefaults();
        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setClosedLoopRampRate(0.1);
        indexerEncoder = indexerMotor.getEncoder();

        SmartDashboard.putNumber("Shooter Preset A", LAUNCH_PRESETS[0]);
        SmartDashboard.putNumber("Shooter Preset B", LAUNCH_PRESETS[1]);
        SmartDashboard.putNumber("Shooter Preset X", LAUNCH_PRESETS[2]);
        SmartDashboard.putNumber("Shooter Preset Y", LAUNCH_PRESETS[3]);

        autoShotsPending = 2;
        disabledInit();
    }

    // toggles the launch wheel on/off (note: this doesn't move the motor or reset target speed)
    public void setLaunchWheelEnabled(boolean enabled) {
        Logger.log("shooter: setting launch wheel to enabled=", enabled);
        spinLaunchWheel = enabled;
    }

    // update the indexer position to indicate that a shot should be taken (note: doesn't move the motor)
    public void shoot() {
        double launchCurrentRpm = indexerEncoder.getVelocity();
        double speedDiff = Math.abs( (launchTargetRpm - launchCurrentRpm) / launchTargetRpm );
        if (speedDiff < LAUNCH_SPEED_TOLERANCE) {
            Logger.log("shooter: shooting!");
            indexerTargetPos += SHOOT_ROTATIONS;
        } else {
            Logger.log("shooter: refusing to shoot; not at speed");
        }
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {

        // update the world about what we're doing
        SmartDashboard.putBoolean("Launch Spinning?", spinLaunchWheel);
        SmartDashboard.putNumber("Launch Target RPM", launchTargetRpm);
        SmartDashboard.putNumber("Launch Current RPM", launchEncoder.getVelocity());
        SmartDashboard.putNumber("Indexer Target Pos", indexerTargetPos);
        SmartDashboard.putNumber("Indexer Current Pos", indexerEncoder.getPosition());
        SmartDashboard.putBoolean("Ball Sensor", ballSensor.get());

        // get new values for presets if necessary
        LAUNCH_PRESETS[0] = SmartDashboard.getNumber("Shooter Preset A", LAUNCH_PRESETS[0]);
        LAUNCH_PRESETS[1] = SmartDashboard.getNumber("Shooter Preset B", LAUNCH_PRESETS[1]);
        LAUNCH_PRESETS[2] = SmartDashboard.getNumber("Shooter Preset X", LAUNCH_PRESETS[2]);
        LAUNCH_PRESETS[3] = SmartDashboard.getNumber("Shooter Preset Y", LAUNCH_PRESETS[3]);

        // clean up presets (make sure they are all negative in case someone didn't type in the sign)
        for (int i=0; i<LAUNCH_PRESETS.length; i++) {
            if (LAUNCH_PRESETS[i] > 0) {
                LAUNCH_PRESETS[i] = -LAUNCH_PRESETS[i];
            }
        }
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {

        launchTargetRpm = LAUNCH_PRESETS[0];
        indexerTargetPos = indexerEncoder.getPosition();
        autoShotsPending = 0;
        spinLaunchWheel = false;

        launchMotor.set(0.0);
        indexerMotor.set(0.0);
    }

    // ================================================================
    // SINGLE SHOOTER
    // Autonomous routine that shoots a single ball after a while
    // ================================================================

    public void singleShooterInit() {
        indexerTargetPos = indexerEncoder.getPosition();
        autoShotsPending = 1;
        Logger.log("shooter: starting single shooter");
    }

    public void singleShooterPeriodic(double seconds) {
        if (seconds > 2.0) {
            if (!spinLaunchWheel) {
                setLaunchWheelEnabled(true);
            }
            if (seconds > 10.5 && autoShotsPending > 1) {
                shoot();
                autoShotsPending--;
            }
            if (seconds > 12) {
                setLaunchWheelEnabled(false);
            }    
        }
        updateLaunchWheel();
        updateIndexerWheel();
    }

    // ================================================================
    // DOUBLE SHOOTER
    // Start routine that shoots two balls after a while
    // ================================================================

    public void doubleShooterInit() {
        autoShotsPending = 2;
        Logger.log("shooter: starting double shooter");
    }

    public void doubleShooterPeriodic(double seconds) {
        if (seconds > 2.0) {
            if (!spinLaunchWheel) {
                setLaunchWheelEnabled(true);
            }
            if (seconds > 10.5 && autoShotsPending > 1) {
                shoot();
                autoShotsPending--;
            }
            if (seconds > 12.5 && autoShotsPending > 0) {
                shoot();
                autoShotsPending--;
            }
            if (seconds > 14) {
                setLaunchWheelEnabled(false);
            }         
        }
        updateLaunchWheel();
        updateIndexerWheel();
    }

    public void teleopInit() {
        indexerTargetPos = indexerEncoder.getPosition();
        launchTargetRpm = LAUNCH_PRESETS[0];
        spinLaunchWheel = false;
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {
        updateLaunchWheel();
        updateIndexerWheel();
    }

    /**
     * Updates the launch wheel - turning it on/off and raising/lowering
     * the target speed.
     */
    protected void updateLaunchWheel() {

        // start button toggles the launch wheel on and off
        if (controller.getStartButtonPressed()) {
            setLaunchWheelEnabled(!spinLaunchWheel);
        }

        // if the launch wheel is spinning, we'll allow speed changes
        if (spinLaunchWheel) {

            if (controller.getAButtonPressed()) {
                launchTargetRpm = LAUNCH_PRESETS[0];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getBButtonPressed()) {
                launchTargetRpm = LAUNCH_PRESETS[1];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getXButtonPressed()) {
                launchTargetRpm = LAUNCH_PRESETS[2];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getYButtonPressed()) {
                launchTargetRpm = LAUNCH_PRESETS[3];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }

            launchController.setReference(launchTargetRpm, ControlType.kVelocity);
        }
        else {
            launchMotor.set(0.0);
        }
    }

    /**
     * Updates the indexer wheel - turning it to fire or intake a ball.
     */
    public void updateIndexerWheel() {

        // trigger indexing if there is a ball waiting
        if (ballSensor.get()) {
            Logger.log("shooter: rotating for intake");
            indexerTargetPos += LOCKIN_ROTATIONS;
        }

        // trigger shooting if someone wants to shoot
        if (controller.getBButtonPressed()) {
            shoot();
        }

        // if the indexer is behind where it needs to be, run it
        // IF INDEXER IS REVERSED - turn this into ">"
        if (indexerTargetPos > indexerEncoder.getPosition()) {
            indexerMotor.set(INDEXER_MAX_SPEED);
        } else {
            indexerMotor.set(0.0);
        }
    }
}