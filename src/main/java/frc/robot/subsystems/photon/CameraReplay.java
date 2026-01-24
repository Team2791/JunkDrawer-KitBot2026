package frc.robot.subsystems.photon;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Replay implementation of {@link CameraIO} for log playback.
 *
 * <p>All methods are no-ops or return empty results, as vision measurements are populated from
 * logs by AdvantageKit's replay infrastructure.
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
