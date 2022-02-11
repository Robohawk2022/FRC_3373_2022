package frc.robot;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hawkbot extends TimedRobot {

    /*
     * These are the port mappings for all the motors, channels, and so forth
     * in the competition bot. We'll keep them all together here so if we need
     * to change them it's easy.
     */

    public static final int DRIVER_JOY_PORT = 0;
    public static final int SPECIAL_OPS_JOY_PORT = 1;
    public static final int INTAKE_MOTOR_PORT = 0;
    public static final int INDEXER_MOTOR_PORT = 1;
    public static final int LAUNCH_MOTOR_PORT = 2;
    public static final int HOOK_MOTOR_PORT = 3;
    public static final int EXTENSION_MOTOR_PORT = 4;
    public static final int ROTATE_MOTOR_PORT = 5;
    public static final int BALL_READY_CHANNEL = 0;
    public static final int FRONT_EYES_PORT = 0;
    public static final int BACK_EYES_PORT = 1;

    private HawkbotControls controls;
    private IntakeSystem intake;
    private ClimberSystem climber;
    private ShooterSystem shooter;
    private EyesSystem eyes;
    private TeleopMode teleopMode;
    private TesterSystem tester;

    /**
     * This is called once at the beginning of the competition to fire everything up.
     * We'll use it to create all of our motors, switches and everything.
     */
    public void robotInit() {

        controls = new HawkbotControls(DRIVER_JOY_PORT, SPECIAL_OPS_JOY_PORT);
        intake = new IntakeSystem(INTAKE_MOTOR_PORT);
        shooter = new ShooterSystem(LAUNCH_MOTOR_PORT, INDEXER_MOTOR_PORT, BALL_READY_CHANNEL);
        climber = new ClimberSystem(HOOK_MOTOR_PORT, EXTENSION_MOTOR_PORT, ROTATE_MOTOR_PORT);

        // reset the climbing arms at the beginning of the competition
        climber.resetMotors();

        // if we're in sim mode, ignore the eyes
        if (!isSimulation()) {
            eyes = new EyesSystem(FRONT_EYES_PORT, BACK_EYES_PORT);
        }

        tester = new TesterSystem();
        intake.collectMotors(tester);
        shooter.collectMotors(tester);
        climber.collectMotors(tester);
    }

    /**
     * This is called every "period" regardless of what phase we're in. We'll use it
     * to send information to the dashboard about all of our motors.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putString("TeleopMode", Objects.toString(teleopMode));
        intake.updateDashboard();
        shooter.updateDashboard();
        climber.updateDashboard();
        if (eyes != null) {
            eyes.updateDashboard();
        }
    }

    /**
     * For now, we're going to use autonomous mode for testing
     */
    @Override
    public void autonomousInit() {
        SmartDashboard.putData("TargetMotor", tester.getChooser());
    }

    /**
     * For now, we're going to use autonomous mode for testing
     */
    @Override
    public void autonomousPeriodic() {
        tester.update(controls);
    }

    /**
     * For now, we're going to use autonomous mode for testing
     */
    @Override
    public void autonomousExit() {
        SmartDashboard.delete("TargetMotor");
    }

    /**
     * This is called at the beginning of teleop. We'll zero out the climbing arms
     * and set ourselves into intake mode.
     */
    public void teleopInit() {
        climber.resetMotors();
        setTeleopMode(TeleopMode.INTAKE);
    }

    /**
     * This is called every "period" during teleop.
     */
    public void teleopPeriodic() {

        if (controls.wasPreviousTeleopModeRequested()) {
            setTeleopMode(teleopMode.previous());
        } else if (controls.wasPreviousTeleopModeRequested()) {
            setTeleopMode(teleopMode.next());
        }

        if (teleopMode == TeleopMode.INTAKE) {
            intake.update(controls);
        }
        else if (teleopMode == TeleopMode.SHOOT) {
            shooter.update(controls);
        }
        else {
            climber.update(controls);
        }
    }

    /**
     * When we're in simulation, we want to create motor simulators for each of the
     * different motors in our robot
     */
    @Override
    public void simulationInit() {
        for (CANSparkMax motor : tester.getAllMotors()) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(motor.getDeviceId()));
        }
    }

    /**
     * This is called every "period" during simulation. We'll use it to run the physics
     * simulation from Rev Robotics
     */
    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    /**
     * This is called when the robot is disabled. We'll use it to stop all of our
     * motors from spinning.
     */
    @Override
    public void disabledInit() {
        intake.stopAll();
        shooter.stopAll();
        climber.stopAll();
    }

    /**
     * Called when we switch from mode to mode within teleop.
     */
    private final void setTeleopMode(TeleopMode newMode) {

        // when we go into climb mode, stop the other motors
        if (newMode == TeleopMode.CLIMB) {
            intake.stopAll();
            shooter.stopAll();
        }

        // make sure to set the eyes right for the mode
        if (eyes != null) {
            eyes.setEyes(newMode);
        }

        teleopMode = newMode;
    }
}
