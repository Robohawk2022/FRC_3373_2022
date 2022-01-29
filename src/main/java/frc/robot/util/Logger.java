public class Logger{

    public static final Logger SWERVE_WHEEL = new Logger(true);
    public static final Logger SWERVE_CONTROLLER = new Logger(false);
    public static final Logger SHOOTER = new Logger(true);
    public static final Logger INTAKE = new Logger(true);

    boolean show = true;

    public Logger(boolean show) {
        this.show = show;
    }

    public void log(String args) {
        if (show) {
            System.out.println(args);
        }
    }
}