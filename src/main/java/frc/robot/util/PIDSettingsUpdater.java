package frc.robot.util;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This will "wrap" a {@link SparkMaxPIDController} and synch its values with the smart
 * dashboard.
 */
public class PIDSettingsUpdater {
    
    public static final PIDConstant DEFAULT = new PIDConstant(0.1, 1e-4, 1.0, 0.0, 0.0, 1.0, -1.0);

    // these are all the labels for the values we're going to be sending
    // to the dashboard
    private final String pGainKey;
    private final String iGainKey;
    private final String dGainKey;
    private final String iZoneKey;
    private final String feedForwardKey;
    private final String minOutputKey;
    private final String maxOutputKey;

    // these are current values for all of the variables for the PID control
    private double pGainCurrent;
    private double iGainCurrent;
    private double dGainCurrent;
    private double iZoneCurrent;
    private double feedForwardCurrent;
    private double minOutputCurrent;
    private double maxOutputCurrent;

    private final SparkMaxPIDController controller;

    public PIDSettingsUpdater(String name, SparkMaxPIDController target) {
        this(name, target, DEFAULT);
    }

    public PIDSettingsUpdater(String name, SparkMaxPIDController target, PIDConstant values) {

        pGainKey = name + " P Gain";
        iGainKey = name + " I Gain";
        dGainKey = name + " D Gain";
        iZoneKey = name + " I Zone";
        feedForwardKey = name + " Feed Forward";
        minOutputKey = name + " Min Output";
        maxOutputKey = name + " Max Output";

        pGainCurrent = values.getP();
        iGainCurrent = values.getI();
        dGainCurrent = values.getD();
        iZoneCurrent = values.getIZone();
        feedForwardCurrent = values.getFeedForward();
        minOutputCurrent = values.getMinOutput();
        maxOutputCurrent = values.getMaxOutput();

        controller = target;
    }

    /**
     * Updates the current PID control settings from the smart dashboard.
     */
    public void updateFromDashboard() {

        double pGainNew = SmartDashboard.getNumber(pGainKey, pGainCurrent);
        if (pGainNew != pGainCurrent) {
            controller.setP(pGainNew);
        }

        double iGainNew = SmartDashboard.getNumber(iGainKey, iGainCurrent);
        if (iGainNew != iGainCurrent) {
            controller.setI(iGainNew);
        }

        double dGainNew = SmartDashboard.getNumber(dGainKey, dGainCurrent);
        if (dGainNew != dGainCurrent) {
            controller.setD(dGainNew);
        }

        double iZoneNew = SmartDashboard.getNumber(iZoneKey, iZoneCurrent);
        if (iZoneNew != iZoneCurrent) {
            controller.setIZone(iZoneNew);
        }

        double feedForwardNew = SmartDashboard.getNumber(feedForwardKey, feedForwardCurrent);
        if (feedForwardNew != feedForwardCurrent) {
            controller.setFF(feedForwardNew);
        }
        
        double minOutputNew = SmartDashboard.getNumber(minOutputKey, minOutputCurrent);
        double maxOutputNew = SmartDashboard.getNumber(maxOutputKey, maxOutputCurrent);
        if (minOutputNew != minOutputCurrent || maxOutputNew != maxOutputCurrent) {
            controller.setOutputRange(minOutputNew, maxOutputNew);
        }
    }
}
