package frc.robot.motors;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PIDConstant;

/**
 * Base class for "closed loop" motors - these will have their power level managed indirectly 
 * via a PID controller rather than directly. This class takes care of reading the PID algorithm
 * parameters from the SmartDashboard for tuning.
 * 
 * Here's some basic information about PID theory - https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html
 */
public abstract class AbstractClosedLoopMotor extends NamedMotor {

    private final SparkMaxPIDController controller;
    private final String pGainKey;
    private final String iGainKey;
    private final String dGainKey;
    private final String iZoneKey;
    private final String feedForwardKey;
    private final String minOutputKey;
    private final String maxOutputKey;
    private PIDConstant constants;

    public AbstractClosedLoopMotor(String name, int port, PIDConstant constants) {
        super(name, port);

        this.controller = getMotor().getPIDController();
        this.pGainKey = name + " P Gain";
        this.iGainKey = name + " I Gain";
        this.dGainKey = name + " D Gain";
        this.iZoneKey = name + " I Zone";
        this.feedForwardKey = name + " Feed Forward";
        this.minOutputKey = name + " Min Output";
        this.maxOutputKey = name + " Max Output";

        SmartDashboard.putNumber(pGainKey, constants.getP());
        SmartDashboard.putNumber(iGainKey, constants.getI());
        SmartDashboard.putNumber(dGainKey, constants.getD());
        SmartDashboard.putNumber(iZoneKey, constants.getIZone());
        SmartDashboard.putNumber(feedForwardKey, constants.getFeedForward());
        SmartDashboard.putNumber(minOutputKey, constants.getMinOutput());
        SmartDashboard.putNumber(maxOutputKey, constants.getMaxOutput());

        setConstants(constants);
    }

    public SparkMaxPIDController getController() {
        return controller;
    }

    public PIDConstant getConstants() {
        return constants;
    }

    private void setConstants(PIDConstant constants) {
        this.constants = constants;
        constants.configPID(controller);
    }

    /**
     * Halts the motor (i.e. brings it to a stop) by putting it in "brake"
     * mode and then applying a 0 duty cycle (basically saying "turn it off")
     */
    public void halt() {
        getMotor().setIdleMode(IdleMode.kBrake);
        getController().setReference(0.0, ControlType.kDutyCycle);
    }

    /**
     * Lets the motor coast by putting it in "coast" mode and then applying a
     * 0 duty cycle (basically saying "turn it off")
     */
    public void coast() {
        getMotor().setIdleMode(IdleMode.kCoast);
        getController().setReference(0.0, ControlType.kDutyCycle);
    }

    public void updateDashboard() {

        super.updateDashboard();

        boolean hasChanged = false;

        double pGainNew = SmartDashboard.getNumber(pGainKey, constants.getP());
        hasChanged = hasChanged || pGainNew != constants.getP();

        double iGainNew = SmartDashboard.getNumber(iGainKey, constants.getI());
        hasChanged = hasChanged || iGainNew != constants.getI();

        double dGainNew = SmartDashboard.getNumber(dGainKey, constants.getD());
        hasChanged = hasChanged || dGainNew != constants.getD();

        double iZoneNew = SmartDashboard.getNumber(iZoneKey, constants.getIZone());
        hasChanged = hasChanged || iZoneNew != constants.getIZone();

        double feedForwardNew = SmartDashboard.getNumber(feedForwardKey, constants.getFeedForward());
        hasChanged = hasChanged || feedForwardNew != constants.getFeedForward();
        
        double minOutputNew = SmartDashboard.getNumber(minOutputKey, constants.getMinOutput());
        hasChanged = hasChanged || minOutputNew != constants.getMinOutput();

        double maxOutputNew = SmartDashboard.getNumber(maxOutputKey, constants.getMaxOutput());
        hasChanged = hasChanged || maxOutputNew != constants.getMaxOutput();

        // we'll only update parameters if something's changed
        if (hasChanged) {
            PIDConstant newConstants = new PIDConstant(pGainNew, iGainNew, dGainNew, feedForwardNew, iZoneNew, minOutputNew, maxOutputNew);
            System.err.println("setting PID constants for "+getName()+" from "+constants+" to "+newConstants);
            setConstants(newConstants);
        }
    }
}
