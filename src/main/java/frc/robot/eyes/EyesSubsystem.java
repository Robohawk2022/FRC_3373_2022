package frc.robot.eyes;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EyesSubsystem {
    
    private UsbCamera frontEyes;
    private UsbCamera backEyes;
    private VideoSink cameraServer;

    public EyesSubsystem() {
        frontEyes = startCapture(0);
        backEyes = startCapture(1);
        cameraServer = CameraServer.getServer();
    }

    public void updateDashboard() {
        if (cameraServer.getSource() == frontEyes) {
            SmartDashboard.setDefaultString("Camera.View", "Front");
        } else {
            SmartDashboard.setDefaultString("Camera.View", "Back");
        }
    }

    public void setDriveView() {
        cameraServer.setSource(frontEyes);
    }

    public void setShootView() {
        cameraServer.setSource(backEyes);
    }

    private UsbCamera startCapture(int port) {
        UsbCamera camera = CameraServer.startAutomaticCapture(port);
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        return camera;
    }
  
}
