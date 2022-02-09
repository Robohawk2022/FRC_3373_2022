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

    public void updateDashboard() {
        // what should happen here?
    }

    public void initTeleopMode(TeleopMode newMode) {
        // what should happen here?
    }

    public void updateTeleop() {
        // what should happen here?
    }

    public void disable() {
        // what should happen here?
    }
}
