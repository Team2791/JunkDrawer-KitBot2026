package frc.robot.subsystems.photon;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Replay implementation of {@link CameraIO} for log playback.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs. It does not
 * interact with any real camera hardware or PhotonVision - all vision measurements are read
 * directly from logged {@link CameraIO.CameraData}.
 *
 * <p>Methods return empty/no-op values because:
 *
 * <ul>
 *   <li>{@link #results()}: No new results needed (logged data is used instead)
 *   <li>{@link #setDriverMode(boolean)}: Camera mode changes are ignored in replay
 * </ul>
 */
public class CameraReplay extends CameraIO {

  /**
   * Constructs a replay camera for a specific camera configuration.
   *
   * @param config The camera configuration including name and transform
   */
  public CameraReplay(VisionConstants.CameraConfig config) {
    super(config);
  }

  /**
   * Returns empty results list for replay mode.
   *
   * <p>Vision measurements are automatically populated from logs by AdvantageKit's replay
   * infrastructure, so no new pipeline results are needed.
   *
   * @return Empty list (no new results)
   */
  @Override
  protected List<PhotonPipelineResult> results() {
    return List.of();
  }

  /**
   * No-op driver mode setter for replay mode.
   *
   * <p>Camera mode changes are not executed during replay.
   *
   * @param enabled Ignored in replay mode
   */
  @Override
  public void setDriverMode(boolean enabled) {}
}
