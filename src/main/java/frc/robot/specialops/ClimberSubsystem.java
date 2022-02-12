package frc.robot.specialops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotPortMap;
/**
 * Subsystem for climbing
 * @param <motorOne>
 */ 
public class ClimberSubsystem<motorOne> {
    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;
    private CANSparkMax motorThree; 

    private final SpecialOpsController controller;
   

    public ClimberSubsystem(SpecialOpsController specialOpsController) {
        controller = specialOpsController;
        motorOne = new CANSparkMax(0, MotorType.kBrushless);
        motorTwo = new CANSparkMax(1, MotorType.kBrushless);
        motorThree = new CANSparkMax(2, MotorType.kBrushless);
    }

    public void updateDashboard() {
        // what should happen here?
    }

    public void updateTeleop() {
        if (controller.wasHookRquested()) {
            System.out.println("Logger: Hook Requested");
        }
    }

    

    public void disable() {
        // what should happen here?
    }
    private MotorController makeLaunchWheel(int port) {
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.setClosedLoopRampRate(port);
        motor.setOpenLoopRampRate(port);
        //motor.setIdleMode(port);
        return motor;
    }
}