package frc.robot.specialops;

import org.junit.Test;

import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

import static org.junit.Assert.assertEquals;

import org.junit.Ignore;

public class IntakeSubsystemTest {

    public static final double EPSILON = 0.0001;

    private final SpecialOpsController controller = new SpecialOpsController(0);
    private final XboxControllerSim controllerSim = new XboxControllerSim(0);
    private final IntakeSubsystem intake = new IntakeSubsystem(controller, 0, 0);
  
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
