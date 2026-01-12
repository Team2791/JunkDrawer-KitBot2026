package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.GameConstants;
import java.util.Arrays;
import java.util.Optional;

/**
 * Coordinate system basics for alliance symmetry
 *   - Blue alliance origin is at field origin
 *   - Red alliance origin is at opposite corner of field
 */
public class AllianceUtil {

    public static class AllianceCell {

        private Alliance last = Alliance.Blue;
        private boolean wasInit = false;

        AllianceCell() {}

        public DriverStation.Alliance get() {
            Optional<Alliance> current = DriverStation.getAlliance();

            if (current.isPresent() || wasInit) {
                last = current.orElse(last);
                wasInit = true;
                return last;
            }

            System.err.println(
                "WARNING: Tried alliance before FMS report. Defaulting to Blue."
            );
            Thread.dumpStack();

            return Alliance.Blue;
        }

        public boolean invert() {
            return get() == DriverStation.Alliance.Red;
        }

        public Pose2d autoflip(Pose2d pose) {
            return invert() ? AllianceUtil.flip(pose) : pose;
        }

        public Pose2d[] autoflip(Pose2d[] poses) {
            return Arrays.stream(poses)
                .map(this::autoflip)
                .toArray(Pose2d[]::new);
        }

        public Rotation2d autoflip(Rotation2d rotation) {
            return invert() ? AllianceUtil.flip(rotation) : rotation;
        }

        public Rotation2d[] autoflip(Rotation2d[] rotations) {
            return Arrays.stream(rotations)
                .map(this::autoflip)
                .toArray(Rotation2d[]::new);
        }

        public int sign() {
            return invert() ? -1 : 1;
        }
    }

    /**
     * SAFETY: this assumes FMS was/has been connected.
     * When that has not happened, we assume Blue alliance and log a warning.
     */
    public static final AllianceCell unsafe = new AllianceCell();

    /** Returns true if the current alliance is Red, false if Blue */
    public static Optional<Boolean> invert() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.<Boolean>map(x -> x == DriverStation.Alliance.Red);
    }

    /** Flips a Pose2d from one alliance's coordinate system to the other */
    public static Pose2d flip(Pose2d pose) {
        return pose.relativeTo(GameConstants.kRedOrigin);
    }

    /** Flips a Rotation2d from one alliance's coordinate system to the other */
    public static Rotation2d flip(Rotation2d rotation) {
        return MathUtil.normalizeAngle(rotation.plus(Rotation2d.kPi));
    }

    /** Automatically flips a Pose2d if the current alliance is Red */
    public static Optional<Pose2d> autoflip(Pose2d pose) {
        return invert().map(x -> x ? flip(pose) : pose);
    }

    /** Automatically flips a Rotation2d if the current alliance is Red */
    public static Optional<Rotation2d> autoflip(Rotation2d rotation) {
        return invert().map(x -> x ? flip(rotation) : rotation);
    }

    /** @return -1 if red, 1 if blue, None if disconnected */
    public static Optional<Integer> sign() {
        return invert().map(x -> x ? -1 : 1);
    }
}
