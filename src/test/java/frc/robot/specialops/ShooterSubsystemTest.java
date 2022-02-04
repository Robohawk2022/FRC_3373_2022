package frc.robot.specialops;

import org.junit.Test;

import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

import static org.junit.Assert.assertEquals;

import org.junit.Ignore;

public class ShooterSubsystemTest {

    public static final double EPSILON = 0.0001;

    private final SpecialOpsController controller = new SpecialOpsController(0);
    private final XboxControllerSim controllerSim = new XboxControllerSim(0);
    private final ShooterSubsystem shooter = new ShooterSubsystem(controller, 0, 0, 0);
  
    @Test
    public void testSpeedChanges() {

        // double initialValue = ShooterSubsystem.DEFAULT_LAUNCH_SPEED;
        // double expectedAfterDecrease = initialValue - ShooterSubsystem.SPEED_CHANGE_AMOUNT;

        // initially, the max launch speed should be the default
        // assertEquals(initialValue, subsystem.getMaxLaunchSpeed(), EPSILON);

        // update the max launch speed
        controllerSim.setLeftBumper(true);
        controllerSim.notifyNewData();
        shooter.updateTeleop();
        controllerSim.setLeftBumper(false);

        // after one decrease, it should be lower
        // assertEquals(expectedAfterDecrease, subsystem.getMaxLaunchSpeed(), EPSILON);

        // update the max launch speed
        controllerSim.setRightBumper(true);
        controllerSim.notifyNewData();
        shooter.updateTeleop();
        controllerSim.setRightBumper(false);

        // and it should be back to the initial value
        // assertEquals(initialValue, subsystem.getMaxLaunchSpeed(), EPSILON);
        controllerSim.setRightBumper(false);
    }

    @Test
    @Ignore("skip until we implement this")
    public void testLauncherSpinUp() {
        // what do we do here?
    }

    @Test
    @Ignore("skip until we implement this")
    public void testShotWithBallReady() {
        // what do we do here?
    }

    @Test
    @Ignore("skip until we implement this")
    public void testShotWithNoBallReady() {
        // what do we do here?
    }
}
