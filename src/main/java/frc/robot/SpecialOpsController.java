package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Controller for "special operations": intake, shooting and climbing.
 * 
 * Each method maps a physical control to a logical action. That way,
 * the mapping is fully described in this one file, and code outside
 * this class doesn't need to refer to specific buttons.
 *  
 * Read the code to determine the control mapping.
 */
public class SpecialOpsController extends XboxController {

    public SpecialOpsController(int port) {
        super(port);
    }

    public boolean wasNextTeleopModeRequested() {
        return getStartButtonPressed();
    }

    public boolean wasPreviousTeleopModeRequested() {
        return getBackButtonPressed();
    }

    // ======================================================================
    // SHOOTING CONTROLS
    // ======================================================================

    public boolean isLaunchWheelActive() {
        return getRawAxis(Axis.kLeftTrigger.value) > 0.5;
    }

    public boolean wasLaunchSpeedDecreaseRequested() {        
        return getLeftBumperPressed();
    }

    public boolean wasLaunchSpeedIncreaseRequested() {
        return getRightBumperPressed();
    }

    public boolean wasShotRequested() {
        return getBButtonPressed();
    }
    public boolean wasLaunchSpeedResetRequested() {
        return getAButtonPressed();
        
    }
}
