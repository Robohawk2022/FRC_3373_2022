package frc.robot.specialops;

import org.junit.Ignore;
import org.junit.Test;

import frc.robot.AbstractTestWithController;

public class ClimberSubsystemTest extends AbstractTestWithController {

    public static final double EPSILON = 0.0001;

    private final SpecialOpsController controller = new SpecialOpsController(0);
    private final ClimberSubsystem intake = new ClimberSubsystem(controller);
  
    @Test
    @Ignore("skip until we implement this")
    public void testSuccess() {

    }
}
