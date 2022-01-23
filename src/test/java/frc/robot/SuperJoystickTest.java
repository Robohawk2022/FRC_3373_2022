package frc.robot;

import edu.wpi.first.wpilibj.simulation.JoystickSim;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Method;

/**
 * This uses the WPILib "sim" concept to test the joystick.
 */
public class SuperJoystickTest {

  /*
   * We declare the joystick we will be testing, and then a corresponding "sim"
   * which lets us simulate joystick interaction. NOTE - both of them have to
   * be using the same port, and order is important:  if you create the sim
   * before you create the joystick, the JVM will crash.
   */

  private final SuperJoystick joystick;

  private final JoystickSim sim;

  public SuperJoystickTest() {
    this.joystick = new SuperJoystick(0);
    this.sim = new JoystickSim(0);
  }

  /**
   * This tests the raw buttons and makes sure the two methods associated with
   * each one work as expected:
   *   - is{button}Held will always report the current status
   *   - is{button}Pushed will only return true if the button was up when
   *     we last cleared buttons, has been pressed since, and has not 
   *     previously been checked with isPushed
   */

  @Test
  public void testRawButtons() {

    String [] names = { "A", "B", "X", "Y", "LB", "RB", "Back", "Start" };
    int [] codes = { 1, 2, 3, 4, 5 , 6, 7, 8 };

    for (int i=0; i<names.length; i++) {

      sim.setRawButton(codes[i], false);
      sim.notifyNewData();
      assertFalse("I cleared "+names[i]+", but it reported as held", isHeld(names[i]));
      assertFalse("I cleared "+names[i]+", but it reported as pushed", isPushed(names[i]));
  
      sim.setRawButton(codes[i], true);
      sim.notifyNewData();
      assertTrue("I pushed "+names[i]+", but it didn't report as held", isHeld(names[i]));
      assertTrue("I pushed "+names[i]+", but it didn't report as held on the second check", isHeld(names[i]));
      assertTrue("I pushed "+names[i]+", but it didn't report as pushed", isPushed(names[i]));
      assertFalse("I already checked "+names[i]+", but it reported as pushed a second time", isPushed(names[i]));

      sim.setRawButton(codes[i], false);
      sim.notifyNewData();
      assertFalse("I cleared "+names[i]+", but it reported as held", isHeld(names[i]));
      assertFalse("I cleared "+names[i]+", but it reported as pushed", isPushed(names[i]));
    }
  }

  /**
   * This tests the dpad and makes sure the two methods associated with
   * each direction as expected:
   *   - is{direction}Held will always report the current status
   *   - is{direction}Pushed will only return true if the dpad was not
   *     in that direction when we last cleared the dpad, has been pressed 
   *     since, and has not previously been checked with isPushed
   */
  
  public void testDpad() {
    String [] names = { "Up", "UpRight", "Right", "DownRight", "Down", "DownLeft", "Left", "UpLeft" };
    int [] povs = { 0, 45, 90, 135, 180, 225, 270, 315 };

    for (int i=0; i<names.length; i++) {

      sim.setPOV(0, -1);
      sim.notifyNewData();
      assertFalse("I cleared the dpad, but "+names[i]+" reported as held", isHeld(names[i]));
      assertFalse("I cleared the dpad, but "+names[i]+" reported as pushed", isPushed(names[i]));
  
      sim.setPOV(0, povs[i]);
      sim.notifyNewData();
      assertTrue("I set the dpad to "+povs[i]+", but "+names[i]+" didn't report as held", isHeld(names[i]));
      assertTrue("I set the dpad to "+povs[i]+", but "+names[i]+" didn't report as held the second time", isHeld(names[i]));
      assertTrue("I set the dpad to "+povs[i]+", but "+names[i]+" didn't report as pushed", isPushed(names[i]));
      assertFalse("I set the dpad to "+povs[i]+", but "+names[i]+" reported as pushed twice", isPushed(names[i]));

      sim.setPOV(0, -1);
      sim.notifyNewData();
      assertFalse("I cleared the dpad, but "+names[i]+" reported as held", isHeld(names[i]));
      assertFalse("I cleared the dpad, but "+names[i]+" reported as pushed", isPushed(names[i]));
    }
  }

  /*
   * This stuff below is some black magic which locates methods on the SuperJoystick
   * class using their name, and then invokes it.
   * 
   * For instance, calling isHeld("A") will invoke isAHeld. Likewise, calling
   * isHeld("B") will call isBHeld.
   * 
   * It's here to save me from having to create a zillion testing methods.
   */

  private boolean isHeld(String which) {
    try {
      Method method = SuperJoystick.class.getMethod("is"+which+"Held");
      return Boolean.class.cast(method.invoke(joystick));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  private boolean isPushed(String which) {
    try {
      Method method = SuperJoystick.class.getMethod("is"+which+"Pushed");
      return Boolean.class.cast(method.invoke(joystick));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }
}