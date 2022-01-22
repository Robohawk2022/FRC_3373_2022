package frc.robot;

import edu.wpi.first.wpilibj.simulation.JoystickSim;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Assert;

/**
 * Test using a simulated joystick!
 */
public class SuperJoystickTest {

  @Test
  public void testSuccess() {

    // This is our normal joystick code, reading from the same port
    SuperJoystick joystick = new SuperJoystick(0);

    // The "sim" is our simulated joystick, which is "plugged in" to port 0
    JoystickSim sim = new JoystickSim(0);

    // button #1 is the A button - if we press it, it should register as held
    sim.setRawButton(1, true);
    sim.notifyNewData();
    assertTrue(joystick.isAHeld());

    // if we release it, it should register as released
    sim.setRawButton(1, false);
    sim.notifyNewData();
    assertFalse(joystick.isAHeld());
  }
}