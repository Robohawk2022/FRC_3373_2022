package frc.robot;

import org.junit.Test;

import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.shooter.ShooterController;
import frc.robot.shooter.ShooterSubsystem;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ShooterSubsystemTest {

    public static final double EPSILON = 0.0001;

    private ShooterSubsystem subsystem;
    private XboxControllerSim controllerSim;

    @Test
    public void testSuccess() {

        init();

        // initially, the max launch speed should be the default
        assertEquals(ShooterSubsystem.DEFAULT_LAUNCH_SPEED, subsystem.getMaxLaunchSpeed(), EPSILON);

        // update the max launch speed
        controllerSim.setRightBumper(true);
        controllerSim.notifyNewData();
        subsystem.updateControls();

        // what should the new value be?
        // assertEquals(???, subsystem.getMaxLaunchSpeed(), EPSILON);
    }

    private void init() {
        try {
            if (controllerSim == null) {
                subsystem = new ShooterSubsystem();
                controllerSim = new XboxControllerSim(0);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
