package frc.robot.subsystems;

import org.junit.Before;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

/**
 * Abstract base class for tests that use an XBoxController. This does
 * the following:
 *   - gives easy access to all the sim methods (e.g. setLeftTrigger)
 *   - adds a "reset" method to zero out the controls
 *   - makes sure controls are zeroed after each test
 */
public abstract class AbstractSubsystemTest extends XboxControllerSim {
    
    public AbstractSubsystemTest() {
        super(0);
    }

    @Before
    public void resetController() {
        for (Button button : Button.values()) {
            setRawButton(button.value, false);
        }
        for (Axis axis : Axis.values()) {
            setRawAxis(axis.value, 0.0);
        }

        // we notify twice here to make sure all the "pressed" and "released"
        // indicators are cleared out, too
        notifyNewData();
        notifyNewData();
    }
}
