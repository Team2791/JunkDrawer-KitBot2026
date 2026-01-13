package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * Mathematical utility functions for robotics calculations.
 *
 * This class provides helper methods for common operations involving poses,
 * rotations, and transformations in 2D space.
 */
public class MathUtil {

    /** Tau (2π) constant for angle calculations. */
    public static final double kTau = 2.0 * Math.PI;

    /**
     * Calculates the transformation from one pose to another in the first pose's frame.
     *
     * <p>Given two poses, returns the transformation that would move an object from the
     * first pose to the second pose, as viewed from the first pose's perspective.
     * In other words, the returned transform represents how something at pose 'a' should
     * move (relative to itself) to reach pose 'b'.
     *
     * @param a The starting pose (reference frame)
     * @param b The target pose
     * @return The transformation from pose a to pose b in a's coordinate frame
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
     * <p>This is useful for standardizing angles so they can be easily compared.
     * Negative angles are wrapped to their positive equivalents.
     *
     * @param angle The angle to normalize (in radians)
     * @return The normalized angle in the range [0, 2π)
     */
    public static double normalizeAngle(double angle) {
        return ((angle % kTau) + kTau) % kTau; // Fix negative angles
    }

    /**
     * Normalizes a Rotation2d to be within the range [0, 2π).
     *
     * @param rotation The rotation to normalize
     * @return A new Rotation2d with the normalized angle
     */
    public static Rotation2d normalizeAngle(Rotation2d rotation) {
        return new Rotation2d(normalizeAngle(rotation.getRadians()));
    }
}
