package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EyesSystem {
    
    private final UsbCamera frontEyes;
    private final UsbCamera backEyes;
    private final VideoSink cameraServer;

    public EyesSystem(int frontPort, int backPort) {
        frontEyes = startCapture(frontPort);
        backEyes = startCapture(backPort);
        cameraServer = CameraServer.getServer();
    }

    public void updateDashboard() {
        if (cameraServer.getSource() == frontEyes) {
            SmartDashboard.setDefaultString("Camera/View", "Drive");
        } else {
            SmartDashboard.setDefaultString("Camera/View", "Shoot");
        }
    }

    public void setEyes(TeleopMode newMode) {
        if (newMode == TeleopMode.SHOOT) {
            cameraServer.setSource(backEyes);
        } else {
            cameraServer.setSource(frontEyes);
        }
    }

    private UsbCamera startCapture(int port) {
        UsbCamera camera = CameraServer.startAutomaticCapture(port);
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        return camera;
    } 
}
