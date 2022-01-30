package frc.robot.util;

public class Logger {

    public static final Logger SWERVE_WHEEL = new Logger(true);
    public static final Logger SWERVE_CONTROLLER = new Logger(false);
    public static final Logger SHOOTER = new Logger(true);
    public static final Logger INTAKE = new Logger(true);

    private final boolean show;

    public Logger(boolean show) {
        this.show = show;
    }

    public void log(String message, Object... args) {
        if (show) {
            System.out.println(String.format(message, args));
        }
    }
}