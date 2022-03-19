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
 * 
 * NOTE: both the shooter and indexer motor directions are reversed. This means:
 * 
 *   - The shooter preset speeds will all be negative RPMs, so the PID loop knows
 *   to run the wheel backwards.
 * 
 *   - The "minimum" launch speed for the shooter wheel is actually a maximum,
 *   because we want it rotating at least that many negative rotations
 * 
 *   - The indexer's target position will move in a negative direction when we
 *   want to advance it for lockin or shooting
 * 
 *   - The indexer motor will receive negative speed values normally, so we can
 *   convince it to spin the correct direction
 * 
 */
public class ShooterSubsystem {

    /** PID tuning constants, determined by trial and error */
    public static final PIDConstant DEFAULT_PID_CONSTANT = new PIDConstant(0.0008, 0.0000003, 3.0, 0.0, 0.0, -1.0, 1.0);

    /** Default preset launch speeds */
    public static final double [] LAUNCH_PRESETS = { -3500.0, -4500.0, -5500.0, -6500.0 };

    /** Maximum speed at which to rotate the indexer */
    public static final double INDEXER_MAX_SPEED = 0.3;

    /** How many rotations does the indexer need to lock in a ball? */
    public static final double LOCKIN_ROTATIONS = 15;

    /** How many rotations does the indexer need to push out a ball? */
    public static final double SHOOT_ROTATIONS = 15;

    /** How close (in RPM) should we be to target to allow shooting? */
    public static final double LAUNCH_RPM_THRESHOLD = 100;

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
    private PIDConstant pidConstant;

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
        setPidConstant(DEFAULT_PID_CONSTANT);

        indexerMotor = new CANSparkMax(indexerMotorPort, MotorType.kBrushless);
        indexerMotor.restoreFactoryDefaults();
        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setClosedLoopRampRate(0.1);
        indexerEncoder = indexerMotor.getEncoder();

        SmartDashboard.putNumber("Shooter Preset Up", LAUNCH_PRESETS[0]);
        SmartDashboard.putNumber("Shooter Preset Right", LAUNCH_PRESETS[1]);
        SmartDashboard.putNumber("Shooter Preset Down", LAUNCH_PRESETS[2]);
        SmartDashboard.putNumber("Shooter Preset Left", LAUNCH_PRESETS[3]);

        autoShotsPending = 2;
        disabledInit();
    }

    private void setPidConstant(PIDConstant newConstant) {
        pidConstant = newConstant;
        pidConstant.configPID(launchController);
        Logger.log("shooter: setting PID to ", pidConstant);
    }

    // toggles the launch wheel on/off (note: this doesn't move the motor or reset target speed)
    public void setLaunchWheelEnabled(boolean enabled) {
        Logger.log("shooter: setting launch wheel to enabled=", enabled);
        spinLaunchWheel = enabled;
    }

    // update the indexer position to indicate that a shot should be taken (note: doesn't move the motor)
    public void shoot() {
        double currentRpm = launchEncoder.getVelocity();
        double deltaRpm = Math.abs(currentRpm - launchTargetRpm);
        if (deltaRpm < LAUNCH_RPM_THRESHOLD) { // less than because we're going backwards
            Logger.log("shooter: shooting!");
            indexerTargetPos = indexerEncoder.getPosition() + SHOOT_ROTATIONS;
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
        LAUNCH_PRESETS[0] = SmartDashboard.getNumber("Shooter Preset Up", LAUNCH_PRESETS[0]);
        LAUNCH_PRESETS[1] = SmartDashboard.getNumber("Shooter Preset Right", LAUNCH_PRESETS[1]);
        LAUNCH_PRESETS[2] = SmartDashboard.getNumber("Shooter Preset Down", LAUNCH_PRESETS[2]);
        LAUNCH_PRESETS[3] = SmartDashboard.getNumber("Shooter Preset Left", LAUNCH_PRESETS[3]);

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
            SmartDashboard.putNumber("POV Angle", controller.getPOV());
            if (controller.getPOV() == 0) {
                launchTargetRpm = LAUNCH_PRESETS[0];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getPOV() == 90) {
                launchTargetRpm = LAUNCH_PRESETS[1];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getPOV() == 180) {
                launchTargetRpm = LAUNCH_PRESETS[2];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getPOV() == 270) {
                launchTargetRpm = LAUNCH_PRESETS[3];
                Logger.log("shooter: reset launch wheel to ", launchTargetRpm);
            }
            else if (controller.getXButtonPressed()) {
                launchTargetRpm = Math.floor(launchTargetRpm * 0.975);
                Logger.log("shooter: setting launch wheel to ", launchTargetRpm);
            }
            else if (controller.getYButtonPressed()) {
                launchTargetRpm = Math.ceil(launchTargetRpm * 1.025);
                Logger.log("shooter: setting launch wheel to ", launchTargetRpm);
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
            indexerTargetPos = indexerEncoder.getPosition() + LOCKIN_ROTATIONS;
            Logger.log("shooter: rotating for intake");
        }

        // trigger shooting if someone wants to shoot
        if (controller.getBButtonPressed()) {
            shoot();
        }

        // if A is held, we'll rotate the indexer slowly backwards; otherwise,
        // we'll only spin it if it's out of position
        if (controller.getAButton()) {
            indexerMotor.set(-INDEXER_MAX_SPEED / 2.0);
        } else if (indexerEncoder.getPosition() < indexerTargetPos) {
            indexerMotor.set(INDEXER_MAX_SPEED);
        } else {
            indexerMotor.set(0.0);
        }
    }

  public void testInit() {
    SmartDashboard.putNumber("Launch P Gain", pidConstant.getP());
    SmartDashboard.putNumber("Launch I Gain", pidConstant.getI());
    SmartDashboard.putNumber("Launch D Gain", pidConstant.getD());
    SmartDashboard.putNumber("Launch I Zone", pidConstant.getIZone());
    SmartDashboard.putNumber("Launch Feed Forward", pidConstant.getFeedForward());
    SmartDashboard.putNumber("Launch Min Output", pidConstant.getMinOutput());
    SmartDashboard.putNumber("Launch Max Output", pidConstant.getMaxOutput());
  }

  public void testPeriodic() {

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if (p != pidConstant.getP()
      || i != pidConstant.getI()
      || d != pidConstant.getD()
      || iz != pidConstant.getIZone()
      || ff != pidConstant.getFeedForward()
      || min != pidConstant.getMinOutput()
      || max != pidConstant.getMaxOutput()) {
        setPidConstant(new PIDConstant(p, i, d, ff, iz, min, max));
    }

    updateLaunchWheel();
  }
}