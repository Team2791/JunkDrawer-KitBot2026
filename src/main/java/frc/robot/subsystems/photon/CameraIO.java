package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.VisionMeasurement;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.AutoLog;
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
 *
 * <p>Subclasses must implement {@link #results()} and {@link #setDriverMode(boolean)} to handle camera-specific operations.
 * Measurement uncertainty is automatically adjusted based on:
 * <ul>
 *   <li>Number of tags detected (single vs multi-tag)
 *   <li>Distance to tags (farther tags = less confidence)
 * </ul>
 */
public abstract class CameraIO {

    /**
     * Data structure for logging vision measurements with AdvantageKit.
     */
    @AutoLog
    public static class CameraData {

        /** The current vision measurement, if available. */
        public VisionMeasurement measurement = null;
    }

    /** Logged data from this camera. */
    public final CameraData data = new CameraData();

    /** Configuration for this camera (name, position, etc.). */
    public final VisionConstants.CameraConfig config;

    /** Standard deviations for measurement uncertainty. */
    private Matrix<N3, N1> stdDevs = VisionConstants.StdDevs.kSingleTag;

    /** PhotonVision pose estimator for AprilTag-based localization. */
    private final PhotonPoseEstimator estimator;

    /** Latest pipeline result from the camera. */
    private PhotonPipelineResult latestResult;

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
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.bot2cam
        );
    }

    /**
     * Gets the latest pipeline results from the camera.
     *
     * <p>Implemented by subclasses to handle camera-specific result retrieval
     * (e.g., USB cameras vs coprocessor cameras).
     *
     * @return List of pipeline results since last call
     */
    protected abstract List<PhotonPipelineResult> results();

    /**
     * Sets whether the camera should display driver view or processing view.
     *
     * @param enabled true for driver view (colorful), false for processing view (grayscale)
     */
    public abstract void setDriverMode(boolean enabled);

    /**
     * Updates the camera and processes vision results.
     *
     * <p>This method:
     * <ul>
     *   <li>Queries the camera for new pipeline results
     *   <li>Updates pose estimation with each result
     *   <li>Calculates measurement uncertainty based on number/distance of tags
     *   <li>Stores the latest measurement for use by the drivetrain
     * </ul>
     */
    public void update() {
        List<PhotonPipelineResult> results = results();
        Optional<EstimatedRobotPose> estimation = Optional.empty();

        // Process all available results
        for (PhotonPipelineResult result : results) {
            estimation = estimator.update(result);
            updateStdDevs(estimation, result.getTargets());
        }

        // Store measurement if pose estimation succeeded
        estimation.ifPresent(estimatedRobotPose -> {
            data.measurement = new VisionMeasurement(
                estimatedRobotPose.estimatedPose,
                stdDevs,
                estimatedRobotPose.timestampSeconds
            );
        });
        latestResult = results.isEmpty() ? null : results.get(0);
    }

    /**
     * Calculates the distance to the nearest detected AprilTag.
     *
     * <p>Used to determine if tags are too far away to be reliable.
     *
     * @return Distance in meters, or {@link Double#MAX_VALUE} if no targets visible
     */
    double nearestTarget() {
        if (latestResult == null) return Double.MAX_VALUE;

        OptionalDouble min = latestResult.targets
            .stream()
            .mapToDouble(c ->
                c.bestCameraToTarget
                    .getTranslation()
                    .toTranslation2d()
                    .getDistance(new Translation2d())
            )
            .min();

        if (min.isEmpty()) return Double.MAX_VALUE;
        else return min.getAsDouble();
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
    void updateStdDevs(
        Optional<EstimatedRobotPose> estimation,
        List<PhotonTrackedTarget> targets
    ) {
        if (estimation.isEmpty()) {
            stdDevs = VisionConstants.StdDevs.kSingleTag;
        } else {
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
                        estimation
                            .get()
                            .estimatedPose.toPose2d()
                            .getTranslation()
                    );

                avgDist += dist;
            }

            if (numTags == 0) {
                // No valid tags detected
                stdDevs = VisionConstants.StdDevs.kSingleTag;
            } else {
                avgDist /= numTags;
                double untrusted = Double.MAX_VALUE;

                // Use multi-tag strategy if we have multiple tags
                if (numTags > 1) devsEst = VisionConstants.StdDevs.kMultiTag;

                // Single tag detection at long range is unreliable
                if (numTags == 1 && avgDist > 4) devsEst = VecBuilder.fill(
                    untrusted,
                    untrusted,
                    untrusted
                );
                // Scale uncertainty by distance squared
                else devsEst = devsEst.times(1 + ((avgDist * avgDist) / 30));
                stdDevs = devsEst;
            }
        }
    }

    /**
     * Gets the latest result from the camera.
     *
     * @return The latest pipeline result, or null if no result is available
     */
    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }
}
