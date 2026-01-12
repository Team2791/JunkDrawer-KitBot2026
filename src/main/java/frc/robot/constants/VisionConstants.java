package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.simulation.SimCameraProperties;

public final class VisionConstants {

    public enum CameraConfig {
        // TODO: we can load the camera properties from a json file at some point
        kCamera(
            "camera",
            new Transform3d(),
            SimCameraProperties.PERFECT_90DEG()
        );

        public final String name;
        public final Transform3d bot2cam;
        public final SimCameraProperties props;

        CameraConfig(
            String name,
            Transform3d bot2cam,
            SimCameraProperties props
        ) {
            this.name = name;
            this.bot2cam = bot2cam;
            this.props = props;
        }
    }

    public static final class Align {

        public static final double kMaxDistance = 1.50;
    }

    public static final class AprilTag {

        // TODO: Season: Field Layout Constants
        public static final AprilTagFieldLayout kLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public static final class StdDevs {

        public static final Matrix<N3, N1> kSingleTag = VecBuilder.fill(
            4,
            4,
            8
        );
        public static final Matrix<N3, N1> kMultiTag = VecBuilder.fill(
            0.5,
            0.5,
            1
        );
    }
}
