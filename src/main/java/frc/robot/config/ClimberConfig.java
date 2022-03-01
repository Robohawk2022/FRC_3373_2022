package frc.robot.config;

import frc.robot.util.Cloner;

import java.io.Serializable;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Subsystem for climbing
 */ 
public class ClimberConfig implements Serializable {

    public int extenderPort;                // port number for extender wheel
    public double extenderMaxOutput;        // how fast should the extender spin?
    public int extenderAxis;                // which control do we use for the extender?
    public double extenderDeadband;         // area around center of sticks to ignore

    public int extenderLimitPort;              // port number for extender limit switch
    public boolean extenderLimitPressedValue;  // value when this switch is pressed

    public int rotatorPort;            // port number for rotator wheel
    public double rotatorMaxOutput;    // how fast should the rotator spin?
    public int rotatorAxis;            // which control do we use for the rotator?
    public double rotatorDeadband;     // area around center of sticks to ignore

    public int rotatorLimitPort;               // port number for rotator limit switch
    public boolean rotatorLimitPressedValue;   // value when this switch is pressed


    public static final ClimberConfig TESTBENCH = new ClimberConfig();
    static {

        TESTBENCH.extenderPort = 1;
        TESTBENCH.extenderMaxOutput = 0.5;
        TESTBENCH.extenderAxis = XboxController.Axis.kLeftY.value;
        TESTBENCH.extenderDeadband = 0.1;
        TESTBENCH.extenderLimitPort = 2;
        TESTBENCH.extenderLimitPressedValue = true;

        TESTBENCH.rotatorPort = 3;
        TESTBENCH.rotatorMaxOutput = 0.5;
        TESTBENCH.rotatorAxis = XboxController.Axis.kRightX.value;
        TESTBENCH.rotatorDeadband = 0.1;
        TESTBENCH.rotatorLimitPort = 3;
        TESTBENCH.rotatorLimitPressedValue = true;
    }

    public static final ClimberConfig COMPETITION = Cloner.copy(TESTBENCH);
    static {
    }
}
