package frc.robot.subsystems.photon;

import frc.robot.alerter.Alerter;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Concrete implementation of {@link CameraIO} for PhotonVision-compatible cameras.
 *
 * <p>This class interfaces with physical USB or network-connected vision cameras running
 * PhotonVision firmware. It handles:
 * <ul>
 *   <li>Camera initialization and configuration
 *   <li>Reading pipeline results from the camera
 *   <li>Setting driver mode for dashboard view
 * </ul>
 *
 * <p>Works with any camera supported by PhotonVision, including standard USB webcams.
 */
public class CameraPhoton extends CameraIO {

    /** The PhotonVision camera interface. */
    final PhotonCamera camera;

    /**
     * Constructs a PhotonVision camera instance.
     *
     * Creates a PhotonCamera using the name from the configuration and initializes
     * the parent class with pose estimation setup.
     *
     * @param config Camera configuration including name and robot-to-camera transform
     */
    public CameraPhoton(VisionConstants.CameraConfig config) {
        super(config);
        camera = new PhotonCamera(config.name);
        camera.setDriverMode(config.passthrough);

        Alerter.getInstance().register(
            String.format("Camera `%s`", config.name()),
            camera,
            PhotonCamera::isConnected
        );
    }

    @Override
    public void update() {
        data.connected = camera.isConnected();
        data.results = camera
            .getAllUnreadResults()
            .toArray(PhotonPipelineResult[]::new);
    }
}
