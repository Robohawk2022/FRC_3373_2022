package frc.robot.subsystems;

import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Subsystem for climbing
 * @param <motorOne>
 */ 
public class ClimberSubsystem<motorOne> {
    private CANSparkMax chainDriveMotor;
    private CANSparkMax rotatorMotor;
    private CANSparkMax motorThree; 

    private final SpecialOpsController controller;
   

    public ClimberSubsystem(SpecialOpsController specialOpsController) {
        controller = specialOpsController;
        chainDriveMotor = new CANSparkMax(0, MotorType.kBrushless);
        rotatorMotor = new CANSparkMax(1, MotorType.kBrushless);
        motorThree = new CANSparkMax(2, MotorType.kBrushless);
    }

    public void robotPeriodic() {
        // what should happen here?
    }

    public void teleopInit(TeleopMode newMode) {
        // what should happen here?
    }

    public void teleopPeriodic() {
        if (controller.getXButtonPressed()) {
            System.out.println("Logger: Hook Requested");
            
        }
        
    }

    public void disabledInit() {
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
