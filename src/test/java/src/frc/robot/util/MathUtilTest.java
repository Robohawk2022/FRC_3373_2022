package frc.robot.util;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * This is a unit test. It's code that gets run by Gradle automatically during a build,
 * that can test your other code. It beats the heck out of either (a) writing a ton of
 * main() methods and running stuff by hand to see if it works, or (b) having to build
 * code and deploy it to the robot just to see if you got stuff right.
 */
public class MathUtilTest {

    /**
     * Every method annotated with @Test is a separate "test" that pokes at
     * something to see if it works. Notice the use of the methods in Assert to
     * declare what the expected output is.
     */
    @Test
    public void testIntClamp() {

        int [][] cases = {
            { 5, 0, 2, 2 },  // test case: outside range on the max side
            { -1, 0, 2, 0 }, // test case: outside range on the min side
            { 1, 0, 2, 1 }   // test case: inside range
        };

        for (int i=0; i<cases.length; i++) {
            int expected = cases[i][3];
            int actual = MathUtil.clamp(cases[i][0], cases[i][1], cases[i][2]);
            assertEquals("Integer clamp failed", expected, actual);
        }
    }
    
    @Test
    public void testFloorMod() {

        double [][] cases = {
            { 10.0, 360.0, 10.0 },    // test case: positive, less than one rotation
            { -10.0, 360.0, 350.0 },   // test case: negative, less than one rotation
            { -370.0, 360.0, 350.0 },  // test case: negative, more than one rotation
            { 370.0, 360.0, 10.0 },   // test case: positive, more than one rotation
        };

        for (int i=0; i<cases.length; i++) {
            double x = cases[i][0];
            double y = cases[i][1];
            double expected = cases[i][2];
            double actual = MathUtil.floorMod(x, y);

            // there are different versions of assertEquals for different data types.
            // since double math can be a little finicky, there's a comparison method
            // that tests if two values are equal within an expected range. see docs at
            // https://junit.org/junit4/javadoc/latest/org/junit/Assert.html#assertEquals(double,%20double,%20double)
            // for information about this specific method, and other types of tests
            // available in Assert
            assertEquals("floorMod failed", expected, actual, 0.00001);
        }
    }
    
    /**
     * This is especially useful with tricky methods like rotation and other
     * calculations - you can use a series of pre-computed values to test 
     * and make sure you've got it right.
     * 
     * While writing this method, I found an apparent bug in the target
     * method (isClockwiseRotationNearer). That's why we write these things!
     */
    @Test
    public void testIsClockwiseRotationNearer() {

        Object [][] cases = {
            { 10.0, 20.0, false },
            { 20.0, 10.0, true }
        };

        for (int i=0; i<cases.length; i++) {
            double angleStart = (Double) cases[i][0];
            double angleFinish = (Double) cases[i][1];
            boolean expected = (Boolean) cases[i][2];
            boolean actual = MathUtil.isClockwiseRotationNearer(angleStart, angleFinish);
            assertEquals("Clockwise rotation comparison failed", expected, actual);
        }
    }
    
}
