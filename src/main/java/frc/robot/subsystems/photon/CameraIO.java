package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import frc.robot.util.VisionMeasurement;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Abstract base class for vision camera implementations.
 *
 * <p>This class provides the core vision processing pipeline:
 * <ul>
 *   <li>Queries camera results for detected AprilTags
 *   <li>Uses {@link PhotonPoseEstimator} for multi-tag pose estimation
 *   <li>Calculates measurement uncertainty based on number and distance of detected tags
 *   <li>Provides pose measurements to the drivetrain for odometry correction
 * </ul>
 */
public abstract class CameraIO {

    /** Auto-logged data structure for camera vision measurements. */
    public static class CameraData {

        CameraData() {}

        /** Whether or not the camera is connected */
        public boolean connected = false;

        /** The current list of vision results */
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];
    }

    /** Logged data from this camera. */
    public final CameraData data = new CameraData();

    /** Configuration for this camera (name, position, etc.). */
    public final VisionConstants.CameraConfig config;

    /** PhotonVision pose estimator for AprilTag-based localization. */
    private final PhotonPoseEstimator estimator;

    /**
     * Constructs a camera IO instance.
     *
     * <p>Initializes the {@link PhotonPoseEstimator} with the AprilTag field layout and
     * multi-tag PnP strategy for robust pose estimation.
     *
     * @param config Configuration including camera name and robot-to-camera transform
     */
    public CameraIO(VisionConstants.CameraConfig config) {
        this.config = config;
        this.estimator = new PhotonPoseEstimator(
            VisionConstants.AprilTag.kLayout,
            config.bot2cam
        );
    }

    /**
     * Sets whether the camera should display driver view or processing view.
     *
     * @param enabled true for driver view, which exposes a camera feed
     */
    public abstract void update();

    /**
     * Computes the {@code VisionMeasurement} of a set of results.
     */
    public VisionMeasurement measurementOf(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimation = Optional.empty();

        // Process all available results
        estimation = estimator.estimateCoprocMultiTagPose(result);
        Matrix<N3, N1> stdDevs = stdDevsOf(estimation, result.getTargets());

        // Store measurement if pose estimation succeeded
        if (estimation.isEmpty()) return null;

        EstimatedRobotPose est = estimation.get();

        return new VisionMeasurement(
            est.estimatedPose,
            stdDevs,
            est.timestampSeconds
        );
    }

    /**
     * Updates measurement standard deviations based on pose estimation quality.
     *
     * <p><strong>Strategy:</strong>
     * <ul>
     *   <li>No estimation: Use single-tag uncertainty (high)
     *   <li>Multiple tags: Use lower multi-tag uncertainty
     *   <li>Single tag: Scale uncertainty by distance (farther = less certain)
     *   <li>Single tag {@literal >} 4m: Mark as completely unreliable
     * </ul>
     *
     * @param estimation The estimated robot pose, if available
     * @param targets The detected targets used for estimation
     */
    private final Matrix<N3, N1> stdDevsOf(
        Optional<EstimatedRobotPose> estimation,
        List<PhotonTrackedTarget> targets
    ) {
        if (estimation.isEmpty()) return VisionConstants.StdDevs.kSingleTag;

        Matrix<N3, N1> devsEst = VisionConstants.StdDevs.kSingleTag;
        int numTags = 0;
        double avgDist = 0;

        // Calculate average distance to detected tags
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = estimator
                .getFieldTags()
                .getTagPose(target.getFiducialId());

            if (tagPose.isEmpty()) continue;

            numTags++;
            double dist = tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(
                    estimation.get().estimatedPose.toPose2d().getTranslation()
                );

            avgDist += dist;
        }

        if (numTags == 0) {
            return VisionConstants.StdDevs.kSingleTag;
        }

        avgDist /= numTags;
        final double untrusted = Double.MAX_VALUE;

        // Use multi-tag devs if we have multiple tags
        if (numTags > 1) return VisionConstants.StdDevs.kMultiTag;

        // Single tag detection at long range is unreliable
        if (numTags == 1 && avgDist > 4) return VecBuilder.fill(
            untrusted,
            untrusted,
            untrusted
        );
        // Scale uncertainty by distance squared
        else return devsEst.times(1 + ((avgDist * avgDist) / 30));
    }
}
