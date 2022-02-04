package frc.robot.specialops;

import org.junit.Ignore;
import org.junit.Test;

import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

public class ClimberSubsystemTest {

    public static final double EPSILON = 0.0001;

    private final SpecialOpsController controller = new SpecialOpsController(0);
    private final XboxControllerSim controllerSim = new XboxControllerSim(0);
    private final ClimberSubsystem intake = new ClimberSubsystem(controller);
  
    @Test
    @Ignore("skip until we implement this")
    public void testSuccess() {

    }
}
