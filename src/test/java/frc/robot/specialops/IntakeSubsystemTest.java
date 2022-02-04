package frc.robot.specialops;

import org.junit.Test;

import frc.robot.AbstractTestWithController;

import static org.junit.Assert.assertTrue;

import org.junit.Ignore;

public class IntakeSubsystemTest extends AbstractTestWithController {

    public static final double EPSILON = 0.0001;

    private final SpecialOpsController controller = new SpecialOpsController(0);
    private final IntakeSubsystem intake = new IntakeSubsystem(controller);
  
    @Test
    public void testIntakeControls() {
        setAButton(true);
        assertTrue(controller.wasIntakeRequested());
    }

    @Test
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
