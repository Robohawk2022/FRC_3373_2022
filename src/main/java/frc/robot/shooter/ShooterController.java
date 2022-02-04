package frc.robot.shooter;

import edu.wpi.first.wpilibj.XboxController;

/**
 * By writing the ShooterController this way, we solve two problems:
 * 
 *   - First, we don't need all the complication of SmartJoystick, because
 *   all the functionality we need is in the default XBoxController class;
 *   and,
 * 
 *   - Second, we keep the mapping of the buttons to actions inside this class,
 *   so if it changes we don't need to go anywhere else.
 * 
 * Read the code to determine the control mapping.
 */
public class ShooterController extends XboxController {

    public ShooterController(int port) {
        super(port);
    }

    /**
     * If someone holds the right trigger, we'll spin the launch wheel
     */
    public boolean isLaunchWheelActive() {
        return getRawAxis(Axis.kRightTrigger.value) > 0.5;
    }

    /**
     * If someone presses the left bumper, we'll decrease the current launch speed
     */
    public boolean wasLaunchSpeedDecreaseRequested() {
        return getLeftBumperPressed();
    }

    /**
     * If someone presses the right bumper, we'll increase the current launch speed
     */
    public boolean wasLaunchSpeedIncreaseRequested() {
        return getRightBumperPressed();
    }

    /**
     * If someone presses B, we'll push a ball in front of the launch wheel
     */
    public boolean wasShotRequested() {
        return getBButtonPressed();
    }

    /**
     * If someone presses A, we'll attempt to intake a ball
     */
    public boolean wasIntakeRequested() {
        return getAButtonPressed();
    }
}
