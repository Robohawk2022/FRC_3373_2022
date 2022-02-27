package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.PositionClosedLoopMotor;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for shooting
 */
public class ShooterSubsystem {

    /** How close to target speed does the launch wheel need to be in order to shoot? */
    public static final double LAUNCH_WINDOW = 0.1;

    /** Because sometimes DigitalInput switches are reversed */
    public static final boolean BALL_AVAILABLE_PRESSED = true;

    /** Starting speed of the launch wheel */
    public static final double STARTING_LAUNCH_RPM = 2000;

    /** How many rotations does the indexer need to lock in a ball? */
    public static final double LOCKIN_ROTATIONS = 10;

    /** How many rotations does the indexer need to push out a ball? */
    public static final double SHOOT_ROTATIONS = 10;

    private final XboxController controller;
    private final DigitalInput ballAvailable;
    private final VelocityClosedLoopMotor launchWheel;
    private final PositionClosedLoopMotor indexerWheel;
    private boolean spinLaunchWheel;
    private boolean launchWheelAtSpeed;
    private double targetLaunchSpeed;
    private boolean ballLocked;
    private int shotCount;

    public ShooterSubsystem(XboxController controller, 
            int launchMotorPort, 
            int indexerMotorPort, 
            int ballAvailableSwitchPort) {
        this.controller = controller;
        this.ballAvailable = new DigitalInput(ballAvailableSwitchPort);
        this.launchWheel = MotorFactory.makeVelocityClosedLoopMotor("Launch", launchMotorPort);
        this.indexerWheel = MotorFactory.makePositionClosedLoopMotor("Indexer", indexerMotorPort);
        disabledInit();
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putBoolean("Launch Spinning?", spinLaunchWheel);
        SmartDashboard.putNumber("Launch Target Speed", targetLaunchSpeed);
        SmartDashboard.putBoolean("Ball Locked?", ballLocked);
        SmartDashboard.putNumber("Shot Count", shotCount);
        SmartDashboard.putBoolean("Launch wheel at speed??", launchWheelAtSpeed);
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        spinLaunchWheel = false;
        targetLaunchSpeed = STARTING_LAUNCH_RPM;
        ballLocked = true;
        shotCount = 0;
        launchWheel.halt();
        indexerWheel.set(0);
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

        // start button turns the launch wheel on and off
        if (controller.getStartButtonPressed()) {
            spinLaunchWheel = !spinLaunchWheel;
            Logger.log("toggled launch wheel to ", spinLaunchWheel);
        }

        // if the launch wheel is spinning, we'll allow speed changes
        if (spinLaunchWheel) {

            // A button: reset launch speed
            if (controller.getAButtonPressed()) {
                targetLaunchSpeed = STARTING_LAUNCH_RPM;
                Logger.log("reset launch wheel to ", targetLaunchSpeed);
            }
            // X button: go 10% slower
            else if (controller.getXButtonPressed()) {
                targetLaunchSpeed *= 0.9;
                Logger.log("slowed down launch wheel to ", targetLaunchSpeed);
            }
            // Y button: go 10% faster
            else if (controller.getYButtonPressed()) {
                targetLaunchSpeed *= 1.1;
                Logger.log("sped up launch wheel to ", targetLaunchSpeed);
            }

            Logger.log("spinning launch wheel at ", targetLaunchSpeed);
            launchWheel.setRpm(targetLaunchSpeed);

            // check to see if it's up to speed
            launchWheelAtSpeed = Math.abs(3.0 * launchWheel.getRpm() - targetLaunchSpeed) < (LAUNCH_WINDOW * targetLaunchSpeed);
        }
        else {
            Logger.log("coasting launch wheel");
            launchWheel.coast();
        }
    }

    /**
     * Updates the indexer wheel - turning it to fire or intake a ball.
     */
    public void updateIndexerWheel() {
        
        // if we have a ball locked, then the only option we need to worry
        // about is whether someone wants to shoot it. the launch wheel has
        // to be up to speed before they can.
        if (ballLocked) {
            Logger.log("Checking for shot");
            if (controller.getBButtonPressed() && launchWheelAtSpeed) {
                Logger.log("Engaging indexer wheel to SHOOT ball");
                indexerWheel.rotate(SHOOT_ROTATIONS);
                // TODO - we will probably have to replace this with a timer, because we
                // don't want to consider the ball "unlocked" until it's actually been
                // ejected, which will take a little time.
                ballLocked = false;
                shotCount++;
            }
        }

        // if we don't have a ball locked, and there's one available, we'll
        // go ahead and lock one in
        else if (ballAvailable.get() == BALL_AVAILABLE_PRESSED) {
            Logger.log("Engaging indexer wheel to lock ball");
            indexerWheel.rotate(LOCKIN_ROTATIONS);

            // TODO - we will probably have to replace this with a timer, because we
            // don't want to consider the ball "locked" until it's actually been
            // grabbed, which will take a little time.
            ballLocked = true;
        }
    }
}