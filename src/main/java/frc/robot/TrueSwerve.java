package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Attempt to use WPI to implement a true swerve drive.
 * 
 * Their "robot coordinate system" is like so:
 *   - Robot is at the origin, facing down the X axis
 *   - Positive X values move you forward
 *   - Positive Y values move you forward

         | +y
         |
         |
      /-----\   --> front of robot
 -----|--+--|-------
      \-----/     +x
         |
         |
         |

 */
public class TrueSwerve {

    /**
     * We want to cap the maximum linear speed. We use input power to the drive
     * motors, so max will be between 0 and 1.
     */
    public static final double MAX_DRIVE_POWER = 0.3;

    /**
     * We want to cap the maximum turning rate. This is expressed in radians
     * per second.
     */
    public static final double MAX_TURN_SPEED = Math.PI / 2.0;

    /**
     * I measured our wheels as requiring 0.4 rotations to spin the wheel an
     * entire 360 degrees.
     */
    public static final double DEGREES_PER_POSITION = 360 / 0.4;

    /**
     * W is the distance you have to move from the center of the robot to land
     * on the front axle (one-half the wheelbase).
     * Distance between axles along the side of the robot
     */
    public static final double W = 30.5 / 2.0;

    /**
     * T is the distance you have to move from the center of the robot to be
     * aligned with the left wheels (one-half the track width).
     */
    public static final double T = 17.5 / 2.0;

    /**
     * Coordinates of the wheels in the WPI lib coordinate system:
     *   +x is forward
     *   +y is left
     */
    public static final Translation2d FRONT_LEFT_TX = new Translation2d(W, T);
    public static final Translation2d FRONT_RIGHT_TX = new Translation2d(W, -T);
    public static final Translation2d BACK_RIGHT_TX = new Translation2d(-W, -T);
    public static final Translation2d BACK_LEFT_TX = new Translation2d(-W, T);

    /**
     * This is the workhorse of the calculations
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_TX,
        FRONT_RIGHT_TX,
        BACK_RIGHT_TX,
        BACK_LEFT_TX
    );

    /**
     * Calculate the target speed and angle for all the swerve wheels
     * 
     * @param leftX left joystick X position
     * @param leftY left joystick Y position
     * @param rightX right joystick X position
     * @param frontLeftAnglePos current position of FL angle motor
     * @param frontRightAnglePos current position of FR angle motor
     * @param backRightAnglePos current position of BR angle motor
     * @param backLeftAnglePos current position of BL angle motor
     * @return the target module states for all four wheels
     */
    public SwerveModuleState [] computeStates(
        double leftX, double leftY, double rightX, 
        double frontLeftAnglePos, double frontRightAnglePos, double backRightAnglePos, double backLeftAnglePos) {

        // invert joystick directions to get desired speeds and compute desired wheel states
        double forwardSpeed = -leftY * MAX_DRIVE_POWER;
        double strafeSpeed = -leftX * MAX_DRIVE_POWER;
        double rotateSpeed = -rightX * MAX_TURN_SPEED;
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(forwardSpeed, strafeSpeed, rotateSpeed);
        SwerveModuleState [] states = KINEMATICS.toSwerveModuleStates(desiredSpeeds);
        System.err.println("initial states:");
        System.err.println("\t"+states[0]);
        System.err.println("\t"+states[1]);
        System.err.println("\t"+states[2]);
        System.err.println("\t"+states[3]);

        // calculate current position in degrees of the wheels
        Rotation2d frontLeftDegrees = Rotation2d.fromDegrees(frontLeftAnglePos * DEGREES_PER_POSITION);
        Rotation2d frontRightDegrees = Rotation2d.fromDegrees(frontRightAnglePos * DEGREES_PER_POSITION);
        Rotation2d backRightDegrees = Rotation2d.fromDegrees(backRightAnglePos * DEGREES_PER_POSITION);
        Rotation2d backLeftDegrees = Rotation2d.fromDegrees(backLeftAnglePos * DEGREES_PER_POSITION);
        System.err.println("rotations:");
        System.err.println("\t"+frontLeftDegrees);
        System.err.println("\t"+frontRightDegrees);
        System.err.println("\t"+backRightDegrees);
        System.err.println("\t"+backLeftDegrees);

        // adjust output according to current state and motor characteristics
        states[0] = SwerveModuleState.optimize(states[0], frontLeftDegrees);
        states[1] = SwerveModuleState.optimize(states[1], frontRightDegrees);
        states[2] = SwerveModuleState.optimize(states[2], backRightDegrees);
        states[3] = SwerveModuleState.optimize(states[3], backLeftDegrees);
        return states;
    }

    public double degreesToPosition(double degrees) {
        return degrees / DEGREES_PER_POSITION;
    }
}
