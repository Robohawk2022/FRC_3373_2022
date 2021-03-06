package frc.robot;
import frc.robot.util.PIDConstant;

public class Constants{
    //* Robot Config
    //TODO change robot dimensions to match this years robot
    public static final double robotLength = 26.25; //The larger dimension, measure from center of wheels
    public static final double robotWidth = 20.3125;


    public static final double wheelCircumference = 18.8495559215387594307759; //TODO measure this value
    //public static final double relativeEncoderRatio= 17.999954;
    public static final double relativeEncoderRatio= 17.9992371;

    public static final double ROTATIONAL_CORRECTION_FACTOR = 4; 


    public static final int BLDriveMotorID = 1;
    public static final int BLRotateMotorID = 2;

    public static final int BRDriveMotorID = 3;
    public static final int BRRotateMotorID = 4;

    public static final int FRDriveMotorID = 5;
    public static final int FRRotateMotorID = 6;

    public static final int FLDriveMotorID = 7;
    public static final int FLRotateMotorID = 8;

    public static final byte numberOfControlSegments = 8;

    //* Drive motors PID
    public static final PIDConstant DRIVE_FL_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant DRIVE_FR_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant DRIVE_BL_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant DRIVE_BR_PID = new PIDConstant(5e-5, 1e-6, 0, 1.56e-4, 0, -1, 1);

    //* Rotation motors PID
    public static final PIDConstant ROTATE_FL_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant ROTATE_FR_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant ROTATE_BL_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);
    public static final PIDConstant ROTATE_BR_PID = new PIDConstant(1, 1e-6, 0, 1.56e-4, 0, -1, 1);

    //* Encoder Positions 
    // Front left
    public static final double FLEncMin = 0.01171875; //0.0078125; //0.015625;
    public static final double FLEncMax = 3.3125;
    public static final double FLEncHome = 2.742188;//0.464844; //1.519531;

    // Back left
    public static final double BLEncMin = 0.4453125;
    public static final double BLEncMax = 3.2578125;
    public static final double BLEncHome = 2.406250;

    // Back right
    public static final double BREncMin = 0.44140625;
    public static final double BREncMax = 3.2890625;
    public static final double BREncHome = 2.406250;

    // Front right
    public static final double FREncMin = 0.45703125;
    public static final double FREncMax = 3.26953125;
    public static final double FREncHome = 1.414063;

    // Indexer constructor values
    public static final int INTAKE_INDEX = 13; // Testbench: 4
    public static final int CONVEYOR_INDEX = 14; // 1
    public static final int PRELOAD_INDEX = 15; // 2
    public static final int LOAD_INDEX = 16; // 3
    
    public static final int INTAKE_BALL_SENSOR_INDEX = 2;
    public static final int CONVEYOR_BALL_SENSOR_INDEX = 1;
    public static final int PRELOAD_BALL_SENSOR_INDEX = 0;

    public static final double INTAKE_SPEED = -1;

    //Climber values
	public static final double CLIMBER_PP=0.5;
    public static final double CLIMBER_PI=0;
    public static final double CLIMBER_PD=0;

    public static final double CLIMBER_WP=0.5;
    public static final double CLIMBER_WI=0;
    public static final double CLIMBER_WD=0;

    public static final int POLE_MOTOR_ID = 10;
    public static final int WINCH_MOTOR_ID = 9;
    public static final int INCLINE_MOTOR_ID = 11;
    public static final int PCM_ID = 19;
    public static final int POLE_SOLENOID_ID = 0;
    public static final int WINCH_SOLENOID_ID = 1;
    
} 
