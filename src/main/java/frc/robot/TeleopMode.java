package frc.robot;

/**
 * Represents the different "modes" our 2022 robot can be in during the teleop phase,
 * and includes logic for looping between them.
 */
public enum TeleopMode {

    INTAKE,
    SHOOT,
    CLIMB;

    public TeleopMode next() {
        switch (this) {
            case INTAKE: return SHOOT;
            case SHOOT: return CLIMB;
            default: return INTAKE;
        }
    }

    public TeleopMode previous() {
        switch (this) {
            case INTAKE: return CLIMB;
            case SHOOT: return INTAKE;
            default: return SHOOT;
        }
    }    
}
