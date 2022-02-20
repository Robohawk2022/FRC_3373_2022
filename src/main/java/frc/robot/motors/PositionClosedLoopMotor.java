package frc.robot.motors;

import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.util.PIDConstant;

/**
 * Subclass of {@link AbstractClosedLoopMotor} for motors that will be moved to
 * set positions, or rotated through set angles.
 * 
 * For these motors, you're extremely unlikely to "set them to 0" because that
 * doesn't mean to stop them, it means to unwind them some crazy number of
 * rotations back to a factory initial value.
 */
public class PositionClosedLoopMotor extends AbstractClosedLoopMotor {
    
    // default PID values for position closed loop from here
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/ef73cb1986c5af01a07bb16cb60e3754fffe3ef4/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java#L48
    public static final PIDConstant DEFAULT_PID = new PIDConstant(0.1, 1e-4, 1.0, 0.0, 0.0, -1.0, 1.0);

    public PositionClosedLoopMotor(String name, int port) {
        super(name, port, DEFAULT_PID);
    }

    /**
     * Rotates to an absolute fixed position and holds the motor there
     * @param position the position (specified as rotations from an arbitrary "zero point")
     */
    public void rotateAbsolute(double position) {
        getController().setReference(position, ControlType.kPosition);
    }

    /**
     * Rotates a certain number of rotations relative to current position
     * @param rotations the number of rotations
     */
    public void rotateRelative(double rotations) {
        double newPosition = getEncoder().getPosition() + rotations;
        rotateAbsolute(newPosition);
    }
}
