package frc.robot.motors;

public interface Motor {

    public void set(double setPoint);

    public default void stop() { set(0.0); }

    public double get();
    
}
