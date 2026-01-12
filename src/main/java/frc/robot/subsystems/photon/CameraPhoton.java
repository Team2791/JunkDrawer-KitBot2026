package frc.robot.subsystems.photon;

import java.util.List;

import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraPhoton extends CameraIO {

    final PhotonCamera camera;

    public CameraPhoton(VisionConstants.CameraConfig config) {
        super(config);
        camera = new PhotonCamera(config.name);
    }

    @Override
    protected List<PhotonPipelineResult> results() {
        return camera.getAllUnreadResults();
    }

    @Override
    public void setDriverMode(boolean enabled) {
        camera.setDriverMode(enabled);
    }
}
