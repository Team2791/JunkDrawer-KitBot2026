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

public final class VisionConstants {

    public record VisionMeasurement(
        Pose3d estimate,
        Matrix<N3, N1> stdDevs,
        double timestamp
    ) {
        public Pose2d estimate2() {
            return estimate.toPose2d();
        }
    }

    public enum CameraConfig {
        kCamera("camera", new Transform3d());

        public final String name;
        public final Transform3d bot2cam;

        CameraConfig(String name, Transform3d bot2cam) {
            this.name = name;
            this.bot2cam = bot2cam;
        }
    }

    public static Transform3d kBotToQuest = new Transform3d(); // TODO:
    public static Matrix<N3, N1> kQuestDevs = VecBuilder.fill(
        Centimeters.of(2).in(Meters),
        Centimeters.of(2).in(Meters),
        Degrees.of(0.035).in(Radians)
    );

    public static final class Align {

        public static final double kMaxDistance = 1.50;
    }

    public static final class AprilTag {

        // TODO: Season: update for 2026 field
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
