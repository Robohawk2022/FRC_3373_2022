package frc.robot.testrobots;

public class RobotPortMap {

    // ===================================================================
    // Camera ports
    // ===================================================================

    public static final int EYES_FRONT = 0;

    public static final int EYES_BACK = 1;

    // ===================================================================
    // Ports on the laptop
    // ===================================================================

    /** Port for shooter controller */
    public static final int SPECIALOPS_CONTROLLER_PORT = 0;

    // ===================================================================
    // Ports on the RoboRIO
    // ===================================================================

    /** Shooter: main launch wheel */
    public static final int SHOOTER_LAUNCH_WHEEL_PORT = 0;

    /** Shooter: switch that says a ball is waiting to be shot */
    public static final int SHOOTER_SHOT_READY_PORT = 1;

    /** Shooter: indexer port */
    public static final int SHOOTER_INDEXER_PORT = 2;

    /** Intake: switch that says a ball is ready for intake */
    public static final int INTAKE_READY_PORT = 3;

    /** Intake: intake wheel */
    public static final int INTAKE_WHEEL_PORT = 4;    

}
