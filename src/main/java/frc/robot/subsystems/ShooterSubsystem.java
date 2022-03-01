package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ShooterConfig;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.PositionClosedLoopMotor;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for shooting
 */
public class ShooterSubsystem {

    private final XboxController controller;
    private final DigitalInput ballAvailable;
    private final VelocityClosedLoopMotor launchWheel;
    private final PositionClosedLoopMotor indexerWheel;
    private final ShooterConfig config;
    private boolean spinLaunchWheel;
    private boolean launchWheelAtSpeed;
    private double targetLaunchSpeed;
    private boolean ballLocked;
    private int shotCount;

    public ShooterSubsystem(XboxController controller, ShooterConfig config) {
        this.controller = controller;
        this.config = config;
        this.ballAvailable = new DigitalInput(config.ballSwitchPort);
        this.launchWheel = MotorFactory.makeVelocityClosedLoopMotor("Launch", config.launcherPort);
        this.indexerWheel = MotorFactory.makePositionClosedLoopMotor("Indexer", config.indexerPort);
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
        targetLaunchSpeed = config.launcherStartingRpm;
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
        if (controller.getBackButtonPressed()) {
            spinLaunchWheel = !spinLaunchWheel;
            Logger.log("toggled launch wheel to ", spinLaunchWheel);
        }

        // if the launch wheel is spinning, we'll allow speed changes
        if (spinLaunchWheel) {

            // reset launch speed
            if (controller.getRawButtonPressed(config.resetSpeedButton)) {
                targetLaunchSpeed = config.launcherStartingRpm;
                Logger.log("reset launch wheel to ", targetLaunchSpeed);
            }
            // go 10% slower
            else if (controller.getRawButtonPressed(config.slowDownButton)) {
                targetLaunchSpeed *= 0.9;
                Logger.log("slowed down launch wheel to ", targetLaunchSpeed);
            }
            // go 10% faster
            else if (controller.getRawButtonPressed(config.speedUpButton)) {
                targetLaunchSpeed *= 1.1;
                Logger.log("sped up launch wheel to ", targetLaunchSpeed);
            }

            launchWheel.setRpm(targetLaunchSpeed);
            launchWheelAtSpeed = Math.abs(config.launcherRpmScale * launchWheel.getRpm() - targetLaunchSpeed) < (config.launcherFudgeWindow * targetLaunchSpeed);
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
        
        // if we have a ball locked, then the only option we need to worry
        // about is whether someone wants to shoot it. the launch wheel has
        // to be up to speed before they can.
        if (ballLocked) {
            Logger.log("Checking for shot");
            if (controller.getRawButtonPressed(config.shootButton) && launchWheelAtSpeed) {
                Logger.log("Engaging indexer wheel to SHOOT ball");
                rotateIndexer(config.indexerEjectDegrees);
                ballLocked = false;
                shotCount++;
            }
        }

        // if we don't have a ball locked, and there's one available, we'll
        // go ahead and lock one in
        else if (ballAvailable.get() == config.ballSwitchPressedValue) {
            Logger.log("Engaging indexer wheel to lock ball");
            rotateIndexer(config.indexerLockinDegrees);
            ballLocked = true;
        }
        indexerWheel.updateSpeed();
    }

    private void rotateIndexer(double degrees) {
        indexerWheel.rotate((degrees / 360.0) * config.indexerGearRatio);
    }
}