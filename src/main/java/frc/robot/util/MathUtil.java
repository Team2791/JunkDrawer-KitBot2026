package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class MathUtil {

    public static final double kTau = 2.0 * Math.PI;

    /**
     * @param a a pose
     * @param b another pose
     * @return the transformation from `a` to `b` in the coordinate frame of `a`. In other words, an
     * object at `a` moves by the returned transform, relative to itself, to reach position `b`.
     */
    public static Transform2d transformationOf(Pose2d a, Pose2d b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        double omega = b.getRotation().minus(a.getRotation()).getRadians();

        double cosA = Math.cos(a.getRotation().getRadians());
        double sinA = Math.sin(a.getRotation().getRadians());

        double dxA = cosA * dx + sinA * dy;
        double dyA = -sinA * dx + cosA * dy;
        return new Transform2d(dxA, dyA, new Rotation2d(omega));
    }

    /**
     * Normalizes an angle to be within the range [0, 2π).
     *
     * @param angle the angle to normalize
     * @return the normalized angle
     */
    public static double normalizeAngle(double angle) {
        return ((angle % kTau) + kTau) % kTau; // fix negative angles
    }

    /**
     * Normalizes angle but with a Rotation instead
     *
     * @param rotation the rotation to normalize
     * @return the normalized rotation
     */
    public static Rotation2d normalizeAngle(Rotation2d rotation) {
        return new Rotation2d(normalizeAngle(rotation.getRadians()));
    }
}
