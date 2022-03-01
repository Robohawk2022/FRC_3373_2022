package frc.robot.config;

import java.io.Serializable;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.Cloner;

/**
 * Subsystem for shooting
 */
public class ShooterConfig implements Serializable {

    public int launcherPort;             // port number for launch wheel
    public double launcherStartingRpm;   // starting RPM for launch wheel (can be adjusted w/ controller)
    public double launcherRpmScale;      // scale factor for launch wheel RPM
    public double launcherFudgeWindow;   // how "close" does launch RPM have to be, to be considered "up to speed"?
    public int indexerPort;               // port number for indexer
    public double indexerGearRatio;       // gear ratio for indexer
    public double indexerLockinDegrees;   // how many degrees to turn for lock in
    public double indexerEjectDegrees;    // how many degrees to turn for shooting
    public double indexerFudgeWindow;     // how "close" does wheel have to be to stop?
    public double indexerMaxOutput;       // motor speed for rotation (1.0 is max)
    public int ballSwitchPort;              // port number for "ball ready" switch
    public boolean ballSwitchPressedValue;  // truth value when it's pressed

    public int startStopButton;  // which button starts/stops the launch wheel?
    public int speedUpButton;    // which button will speed up the launch wheel?
    public int slowDownButton;   // which button will slow down the launch wheel?
    public int resetSpeedButton; // which button will reset the launch wheel speed?
    public int shootButton;      // which button will trigger a shot?

    // launch wheel is wheel 1
    // indexer is wheel 3 (20:1 geared)
    // ball ready is switch #3 (not inverted)
    public static final ShooterConfig TESTBENCH = new ShooterConfig();
    static {
        TESTBENCH.launcherPort = 1;
        TESTBENCH.launcherStartingRpm = 4000;
        TESTBENCH.launcherRpmScale = 3;
        TESTBENCH.launcherFudgeWindow = 0.1;
        TESTBENCH.indexerPort = 1;
        TESTBENCH.indexerGearRatio = 20 / 1;
        TESTBENCH.indexerLockinDegrees = 180;
        TESTBENCH.indexerEjectDegrees = 270;
        TESTBENCH.indexerFudgeWindow = 0.0005;
        TESTBENCH.indexerMaxOutput = 0.7;
        TESTBENCH.ballSwitchPort = 3;
        TESTBENCH.ballSwitchPressedValue = true;
        TESTBENCH.startStopButton = XboxController.Button.kBack.value;
        TESTBENCH.speedUpButton = XboxController.Button.kX.value;
        TESTBENCH.slowDownButton = XboxController.Button.kY.value;
        TESTBENCH.resetSpeedButton = XboxController.Button.kA.value;
        TESTBENCH.shootButton = XboxController.Button.kB.value;
    }

    // launch wheel is wheel 1
    // indexer is wheel 3 (20:1 geared)
    // ball ready is switch #3 (not inverted)
    public static final ShooterConfig COMPETITION = Cloner.copy(TESTBENCH);
    static {
        TESTBENCH.launcherPort = 10;
        TESTBENCH.indexerPort = 11;
        TESTBENCH.ballSwitchPort = -1; // TODO switch port?
    }
}