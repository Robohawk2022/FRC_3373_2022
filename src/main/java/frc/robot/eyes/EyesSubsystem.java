package frc.robot.eyes;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpecialOpsController;
import frc.robot.testrobots.RobotPortMap;

public class EyesSubsystem {
    
    private final SpecialOpsController controller;
    private final UsbCamera frontEyes;
    private final UsbCamera backEyes;
    private VideoSink cameraServer;

    public EyesSubsystem(SpecialOpsController specialOpsController) {
        controller = specialOpsController;
        frontEyes = startCapture(RobotPortMap.EYES_FRONT);
        backEyes = startCapture(RobotPortMap.EYES_BACK);
        cameraServer = CameraServer.getServer();
    }

    public void updateDashboard() {
        if (cameraServer.getSource() == frontEyes) {
            SmartDashboard.setDefaultString("Camera.View", "Drive");
        } else {
            SmartDashboard.setDefaultString("Camera.View", "Shoot");
        }
    }

    public void updateTeleop() {
        if (controller.isLaunchWheelActive()) {
            cameraServer.setSource(backEyes);
        } else {
            cameraServer.setSource(frontEyes);
        }
    }

    public void disable() {
        // what should happen?
    }

    private UsbCamera startCapture(int port) {
        UsbCamera camera = CameraServer.startAutomaticCapture(port);
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        return camera;
    } 
}
