package frc.robot.config;

import java.io.Serializable;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.Cloner;

/**
 * Configuration for ball intake
 */
public class IntakeConfig implements Serializable {

    public int intakePort;             // port number for intake wheel
    public double intakeStartingRpm;   // starting RPM for intake wheel (can be adjusted w/ controller)
    public double intakeRpmScale;      // scale factor for intake wheel RPM
    public int startStopButton;  // which button starts/stops the intake wheel?
    public int speedUpButton;    // which button will speed up the intake wheel?
    public int slowDownButton;   // which button will slow down the intake wheel?
    public int resetSpeedButton; // which button will reset the intake wheel speed?
    public int reverseTrigger;   // which trigger reverses the intake wheel?

    public static final IntakeConfig TESTBENCH = new IntakeConfig();
    static {
        TESTBENCH.intakePort = 4;
        TESTBENCH.intakeStartingRpm = 3000;
        TESTBENCH.intakeRpmScale = 3.0;
        TESTBENCH.startStopButton = XboxController.Button.kBack.value;
        TESTBENCH.speedUpButton = XboxController.Button.kX.value;
        TESTBENCH.slowDownButton = XboxController.Button.kY.value;
        TESTBENCH.resetSpeedButton = XboxController.Button.kA.value;
        TESTBENCH.reverseTrigger = XboxController.Axis.kLeftTrigger.value;
    }

    public static final IntakeConfig COMPETITION = Cloner.copy(TESTBENCH);
    static {
        COMPETITION.intakePort = 9;
    }
}