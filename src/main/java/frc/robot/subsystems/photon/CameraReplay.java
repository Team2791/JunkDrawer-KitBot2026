package frc.robot.subsystems.photon;

import frc.robot.constants.VisionConstants;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Replay implementation of {@link CameraIO} for simulation/log replay.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all QuestNav data is
 * read directly from the logged {@link CameraIO.CameraData} values.
 *
 * <p>All methods are no-ops because all replayed data is immutable.
 */
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
