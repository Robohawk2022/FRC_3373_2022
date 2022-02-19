package frc.robot.motors;

public class MotorSettings {
    
    public static enum Type {
        OPEN_LOOP,
        VELOCITY,
        POSITION
    }

    public String name;
    public Type type = Type.OPEN_LOOP;
    public int port = -1;
    public double rampRate = 1.0;
    public boolean allowCoasting = false;

}
