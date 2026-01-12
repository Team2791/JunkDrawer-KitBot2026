package frc.robot.subsystems.photon;

import frc.robot.constants.VisionConstants;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraReplay extends CameraIO {

    public CameraReplay(VisionConstants.CameraConfig config) {
        super(config);
    }

    @Override
    protected List<PhotonPipelineResult> results() {
        return List.of();
    }

    @Override
    public void setDriverMode(boolean enabled) {}
}
