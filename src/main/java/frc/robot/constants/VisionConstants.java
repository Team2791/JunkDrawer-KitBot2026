package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Vision system constants for AprilTag-based localization.
 *
 * <p>Contains camera configurations, measurement uncertainty parameters,
 * and field layout definitions for both PhotonVision and Quest Pro cameras.
 *
 * <p>Used by {@link frc.robot.subsystems.photon.Photon} and
 * {@link frc.robot.subsystems.quest.Quest} for pose estimation.
 */
public final class VisionConstants {

    /**
     * Vision measurement data structure.
     *
     * <p>Bundles a robot pose estimate with uncertainty (standard deviations)
     * and timestamp for fusion with odometry.
     *
     * @param estimate 3D robot pose from vision
     * @param stdDevs Standard deviations [x, y, theta] for pose estimator weighting
     * @param timestamp FPGA timestamp when image was captured (seconds)
     */
    public record VisionMeasurement(
        Pose3d estimate,
        Matrix<N3, N1> stdDevs,
        double timestamp
    ) {
        /**
         * Convert 3D pose to 2D for odometry fusion.
         *
         * @return 2D robot pose (ignoring Z-axis)
         */
        public Pose2d estimate2() {
            return estimate.toPose2d();
        }
    }

    /**
     * Camera hardware configurations.
     *
     * <p>Each camera requires a name (for NetworkTables) and transform
     * from robot center to camera optical frame.
     */
    public enum CameraConfig {
        /**
         * Primary AprilTag camera.
         *
         * <p>TODO: Define actual {@link Transform3d} from robot center to camera.
         */
        kCamera("camera", new Transform3d());

        /** Camera name for PhotonVision NetworkTables. */
        public final String name;

        /** Transform from robot center to camera (robot coords → camera coords). */
        public final Transform3d bot2cam;

        CameraConfig(String name, Transform3d bot2cam) {
            this.name = name;
            this.bot2cam = bot2cam;
        }
    }

    /**
     * Transform from robot center to Quest Pro headset.
     *
     * <p>TODO: Measure and configure actual transform.
     */
    public static Transform3d kBotToQuest = new Transform3d();

    /**
     * Quest Pro measurement standard deviations.
     *
     * <p>Uncertainty in X, Y (2 cm each) and rotation (0.035 degrees).
     * Lower values = more trust in Quest measurements.
     */
    public static Matrix<N3, N1> kQuestDevs = VecBuilder.fill(
        Centimeters.of(2).in(Meters),
        Centimeters.of(2).in(Meters),
        Degrees.of(0.035).in(Radians)
    );

    /**
     * Vision alignment constraints.
     *
     * <p>Limits vision-based control commands to reasonable ranges.
     */
    public static final class Align {

        /**
         * Maximum distance for vision-based alignment (1.5 meters).
         *
         * <p>Beyond this range, AprilTag detection becomes unreliable.
         */
        public static final double kMaxDistance = 1.50;
    }

    /**
     * AprilTag field layout configuration.
     *
     * <p>Defines tag positions and IDs for the game field.
     */
    public static final class AprilTag {

        /**
         * Field layout for 2025 Reefscape (Welded variant).
         *
         * <p>TODO: Update for 2026 field when released.
         */
        public static final AprilTagFieldLayout kLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    /**
     * Measurement uncertainty standard deviations.
     *
     * <p>Used by {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     * to weight vision measurements vs. odometry.
     */
    public static final class StdDevs {

        /**
         * Single AprilTag measurement uncertainty.
         *
         * <p>Higher uncertainty (4m, 4m, 8 rad) due to ambiguity with one tag.
         * <ul>
         *   <li>X standard deviation: 4.0 meters
         *   <li>Y standard deviation: 4.0 meters
         *   <li>Theta standard deviation: 8.0 radians
         * </ul>
         */
        public static final Matrix<N3, N1> kSingleTag = VecBuilder.fill(
            4,
            4,
            8
        );

        /**
         * Multiple AprilTag measurement uncertainty.
         *
         * <p>Lower uncertainty (0.5m, 0.5m, 1 rad) with multiple tags visible.
         * <ul>
         *   <li>X standard deviation: 0.5 meters
         *   <li>Y standard deviation: 0.5 meters
         *   <li>Theta standard deviation: 1.0 radian
         * </ul>
         */
        public static final Matrix<N3, N1> kMultiTag = VecBuilder.fill(
            0.5,
            0.5,
            1
        );
    }
}
