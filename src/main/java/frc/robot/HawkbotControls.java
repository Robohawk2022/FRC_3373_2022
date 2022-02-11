package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class HawkbotControls {

    private final XboxController driver;
    private final XboxController specialOps;

    public HawkbotControls(int driverPort, int specialOpsPort) {
        driver = new XboxController(driverPort);
        specialOps = new XboxController(specialOpsPort);
    }

    public boolean wasPreviousTeleopModeRequested() {
        return specialOps.getBackButtonPressed();
    }

    public boolean wasNextTeleopModeRequested() {
        return specialOps.getStartButtonPressed();
    }

    public boolean wasLaunchSpeedIncreaseRequested() {
        return specialOps.getRightBumperPressed();
    }

    public boolean wasLaunchSpeedDecreaseRequested() {
        return specialOps.getLeftBumperPressed();
    }

    public boolean wasShotRequested() {
        return specialOps.getBButtonPressed();
    }

    public boolean wasIntakeReverseRequested() {
        return specialOps.getBButtonPressed();
    }

    public double getClimberExtensionRate() {
        return specialOps.getLeftY();
    }

    public double getClimberRotateRate() {
        return specialOps.getRightX();
    }

    public boolean wasHookToggleRequested() {
        return specialOps.getBButtonPressed();
    }

    public double getForward() {
        return driver.getLeftY();
    }

    public double getStrafe() {
        return driver.getLeftX();
    }

    public double getRotate() {
        return driver.getRightX();
    }

    public double getTestSpeed() {
        return driver.getLeftY();
    }

    public boolean wasTestStopRequested() {
        return driver.getBButtonPressed();
    }

    public boolean wasTestSpeedUpRequested() {
        return driver.getXButtonPressed();
    }

    public boolean wasTestSlowDownRequested() {
        return driver.getYButtonPressed();
    }
}
