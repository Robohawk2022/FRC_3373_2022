package frc.robot.specialops;

import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.assertTrue;


public class IntakeSubsystemTest extends AbstractSubsystemTest {

    public static final double EPSILON = 0.0001;

    private final SpecialOpsController controller = new SpecialOpsController(0);
    private final IntakeSubsystem intake = new IntakeSubsystem(controller);
  
    @Test
    public void testIntakeControls() {
        setAButton(true);
        //assertTrue(controller.wasIntakeRequested());
    }

    @Ignore("skip until we implement this")
    public void testIntakeWithBallReady() {
        // what should we do here?
    }
  
    @Test
    @Ignore("skip until we implement this")
    public void testIntakeWithNoBallReady() {
        // what should we do here?
    }
}
