package frc.robot.subsystems;

import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;

/**
 * Subsystem for climbing
 */
public class ClimberSubsystem {
    
    private final SpecialOpsController controller;

    public ClimberSubsystem(SpecialOpsController specialOpsController) {
        controller = specialOpsController;
    }

    public void robotPeriodic() {
        // what should happen here?
    }

    public void teleopInit(TeleopMode newMode) {
        // what should happen here?
    }

    public void teleopPeriodic() {
        // what should happen here?
    }

    public void disabledInit() {
        // what should happen here?
    }
}
