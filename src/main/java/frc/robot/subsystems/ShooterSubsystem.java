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
    public static final double STARTING_LAUNCH_RPM = -2000;

    /** How many rotations does the indexer need to lock in a ball? */
    public static final double LOCKIN_ROTATIONS = 25;

    /** How many rotations does the indexer need to push out a ball? */
    public static final double SHOOT_ROTATIONS = 20;

    private final XboxController controller;
    private final DigitalInput ballSensor;
    private final VelocityClosedLoopMotor launchWheel;
    private final PositionClosedLoopMotor indexerWheel;
    private boolean spinLaunchWheel;
    private boolean launchWheelAtSpeed;
    private double targetLaunchSpeed;
    private boolean sensorWasTripped;
    private int shotCount;

    public ShooterSubsystem(XboxController controller, 
            int launchMotorPort, 
            int indexerMotorPort, 
            int ballAvailableSwitchPort) {
        System.err.println("initializing shooter");
        this.controller = controller;
        this.ballSensor = new DigitalInput(ballAvailableSwitchPort);
        this.launchWheel = MotorFactory.makeVelocityClosedLoopMotor("Launch", launchMotorPort);
        this.indexerWheel = MotorFactory.makePositionClosedLoopMotor("Indexer", indexerMotorPort);
        disabledInit();
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putBoolean("Launch Spinning?", spinLaunchWheel);
        SmartDashboard.putNumber("Launch Target Speed", targetLaunchSpeed);
        SmartDashboard.putBoolean("Sensor Tripped?", sensorWasTripped);
        SmartDashboard.putNumber("Shot Count", shotCount);
        SmartDashboard.putBoolean("Launch wheel at speed?", launchWheelAtSpeed);
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        spinLaunchWheel = false;
        targetLaunchSpeed = STARTING_LAUNCH_RPM;
        sensorWasTripped = true;
        shotCount = 0;
        launchWheel.halt();
        indexerWheel.resetClosedLoopControl();
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
            Logger.log("shooter: toggled launch wheel to ", spinLaunchWheel);
        }

        // if the launch wheel is spinning, we'll allow speed changes
        if (spinLaunchWheel) {

            // A button: reset launch speed
            if (controller.getAButtonPressed()) {
                targetLaunchSpeed = STARTING_LAUNCH_RPM;
                Logger.log("shooter: reset launch wheel to ", targetLaunchSpeed);
            }
            // X button: go 10% slower
            else if (controller.getXButtonPressed()) {
                targetLaunchSpeed *= 0.9;
                Logger.log("shooter: slowed down launch wheel to ", targetLaunchSpeed);
            }
            // Y button: go 10% faster
            else if (controller.getYButtonPressed()) {
                targetLaunchSpeed *= 1.1;
                Logger.log("shooter: sped up launch wheel to ", targetLaunchSpeed);
            }

            launchWheel.setRpm(targetLaunchSpeed);
            launchWheelAtSpeed = Math.abs(3.0 * launchWheel.getRpm() - targetLaunchSpeed) < Math.abs(LAUNCH_WINDOW * targetLaunchSpeed);
        }
        else {
            launchWheel.coast();
            launchWheelAtSpeed = false;
        }
    }

    /**
     * Updates the indexer wheel - turning it to fire or intake a ball.
     */
    public void updateIndexerWheel() {

        // trigger indexing IF there is a ball in front of the sensor now,
        // and there wasn't one last time
        if (ballSensor.get()) {
            if (!sensorWasTripped) {
                Logger.log("shooter: rotating for intake");
                indexerWheel.rotate(LOCKIN_ROTATIONS);
            }
            sensorWasTripped = true;
        }
        else {
            sensorWasTripped = false;
        }

        // if someone wants to shoot, and the wheel's at speed and we 
        // have a ball, go for it!
        if (controller.getBButtonPressed()) {
            boolean haveBall = ballSensor.get();
            Logger.log("shooter: attempting to shoot: have ball? "+haveBall+"; at speed? "+launchWheelAtSpeed);
            if (haveBall && launchWheelAtSpeed) {
                Logger.log("shooter: shooting!");
                indexerWheel.rotate(SHOOT_ROTATIONS);    
            }
        }

        indexerWheel.updateSpeed();
    }
}