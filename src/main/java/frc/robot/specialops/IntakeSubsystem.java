package frc.robot.specialops;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    private final SpecialOpsController controller;
    private final DigitalInput intakeReadySwitch;
    // private final SomethingSomething intakeWheel;
    private int totalBalls;
    
    public IntakeSubsystem(SpecialOpsController controller, int intakeReadyPort, int intakeWheelPort) {
        this.controller = controller;
        this.intakeReadySwitch = new DigitalInput(intakeReadyPort);
        // this.intakeWheel = ???
    }

    public void updateDashboard() {
        SmartDashboard.setDefaultNumber("Intake.totalBalls", totalBalls);
    }

    public void updateControls() {
        if (controller.wasIntakeRequested()) {
            // TODO what should happen here?
        }
    }

    public void disable() {
        // what should happen here?
    }
}