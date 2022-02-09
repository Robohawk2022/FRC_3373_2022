package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.testrobots.RobotPortMap;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    private final SpecialOpsController controller;
    private final DigitalInput intakeReadySwitch;
    // private final SomethingSomething intakeWheel;
    private int totalBalls;
    
    public IntakeSubsystem(SpecialOpsController controller) {
        this.controller = controller;
        this.intakeReadySwitch = new DigitalInput(RobotPortMap.INTAKE_READY_PORT);
        // this.intakeWheel = ???
    }

    public void robotPeriodic() {
        SmartDashboard.setDefaultNumber("Intake.totalBalls", totalBalls);
    }

    /** We want to stop all motors during climb mode */
    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.CLIMB) {
            disabledInit();
        }
    }

    public void telopPeriodic() {
        if (controller.wasIntakeRequested()) {
            // TODO what should happen here?
        }
    }

    public void disabledInit() {
        // what should happen here?
    }
}