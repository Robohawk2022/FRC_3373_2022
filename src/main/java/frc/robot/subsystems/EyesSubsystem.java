package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.TeleopMode;
import frc.robot.testrobots.RobotPortMap;

public class EyesSubsystem {
    
    private final UsbCamera frontEyes;
    private final UsbCamera backEyes;
    private VideoSink cameraServer;

    public EyesSubsystem() {
        frontEyes = startCapture(RobotPortMap.EYES_FRONT);
        backEyes = startCapture(RobotPortMap.EYES_BACK);
        cameraServer = CameraServer.getServer();
    }

    public void robotPeriodic() {
        if (cameraServer.getSource() == frontEyes) {
            SmartDashboard.setDefaultString("Camera.View", "Drive");
        } else {
            SmartDashboard.setDefaultString("Camera.View", "Shoot");
        }
    }

    public void teleopInit(TeleopMode newMode) {
        if (newMode == TeleopMode.SHOOT) {
            cameraServer.setSource(backEyes);
        } else {
            cameraServer.setSource(frontEyes);
        }
    }

    public void disabledInit() {
        // what should happen?
    }

    private UsbCamera startCapture(int port) {
        UsbCamera camera = CameraServer.startAutomaticCapture(port);
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        return camera;
    } 
}