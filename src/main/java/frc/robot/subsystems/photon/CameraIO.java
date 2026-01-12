package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public abstract class CameraIO {

    public record VisionMeasurement(
        Pose3d estimate,
        Matrix<N3, N1> stdDevs,
        double timestamp
    ) {
        public Pose2d estimate2() {
            return estimate.toPose2d();
        }
    }

    @AutoLog
    public static class CameraData {

        public VisionMeasurement measurement = null;
    }

    public final CameraData data = new CameraData();
    public final VisionConstants.CameraConfig config;
    private Matrix<N3, N1> stdDevs = VisionConstants.StdDevs.kSingleTag;

    private final PhotonPoseEstimator estimator;
    private PhotonPipelineResult latestResult;

    public CameraIO(VisionConstants.CameraConfig config) {
        this.config = config;
        this.estimator = new PhotonPoseEstimator(
            VisionConstants.AprilTag.kLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.bot2cam
        );
    }

    protected abstract List<PhotonPipelineResult> results();

    public abstract void setDriverMode(boolean enabled);

    public void update() {
        List<PhotonPipelineResult> results = results();
        Optional<EstimatedRobotPose> estimation = Optional.empty();

        for (PhotonPipelineResult result : results) {
            estimation = estimator.update(result);
            updateStdDevs(estimation, result.getTargets());
        }

        estimation.ifPresent(estimatedRobotPose -> {
            data.measurement = new VisionMeasurement(
                estimatedRobotPose.estimatedPose,
                stdDevs,
                estimatedRobotPose.timestampSeconds
            );
        });
        latestResult = results.isEmpty() ? null : results.get(0);
    }

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
                stdDevs = VisionConstants.StdDevs.kSingleTag;
            } else {
                avgDist /= numTags;
                double untrusted = Double.MAX_VALUE;
                if (numTags > 1) devsEst = VisionConstants.StdDevs.kMultiTag;
                if (numTags == 1 && avgDist > 4) devsEst = VecBuilder.fill(
                    untrusted,
                    untrusted,
                    untrusted
                );
                else devsEst = devsEst.times(1 + ((avgDist * avgDist) / 30));
                stdDevs = devsEst;
            }
        }
    }

    /**
     * Get the latest result from the camera. May be null
     * @return the latest result from the camera
     */
    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }
}
