package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Control loop constants for robot motion control.
 *
 * <p>Contains PID gains, feedforward coefficients, and control limits for:
 *
 * <ul>
 *   <li>Swerve module drive motors (velocity control)
 *   <li>Swerve module turn motors (position control)
 *   <li>Autonomous trajectory following
 *   <li>Vision-based alignment
 *   <li>Joystick input rate limiting
 * </ul>
 */
public final class ControlConstants {

    private ControlConstants() {}

    /**
     * Autonomous trajectory following PID constants.
     *
     * <p>Used by {@link frc.robot.autos.AutoManager} for Choreo trajectory following.
     */
    public static final class Auto {

        private Auto() {}

        /** Proportional gain for X and Y position errors (orthogonal axes). */
        public static final double kOrthoP = 1.25;
        /** Integral gain for orthogonal position errors. */
        public static final double kOrthoI = 0.00;
        /** Derivative gain for orthogonal position errors. */
        public static final double kOrthoD = 0.00;

        /** Proportional gain for rotation error. */
        public static final double kTurnP = 0.00;
        /** Integral gain for rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for rotation error. */
        public static final double kTurnD = 0.00;

        /** Threshold to switch to nearby control. */
        public static final double kNearbyThreshold = 0.25;
    }

    /**
     * Vision-based alignment PID constants.
     *
     * <p>Used by {@link frc.robot.commands.pathfind.Nearby} for fine alignment
     */
    public static final class Nearby {

        private Nearby() {}

        /** Proportional gain for X and Y alignment errors. */
        public static final double kOrthoP = 4.75;
        /** Integral gain for alignment position errors. */
        public static final double kOrthoI = 0.00;
        /** Derivative gain for alignment position errors. */
        public static final double kOrthoD = 0.00;

        /** Proportional gain for alignment rotation error. */
        public static final double kTurnP = 2.60;
        /** Integral gain for alignment rotation error. */
        public static final double kTurnI = 0.00;
        /** Derivative gain for alignment rotation error. */
        public static final double kTurnD = 0.04;

        /** Maximum angular velocity during alignment (radians/second). TODO: Needs tuning. */
        public static final double kMaxTurnVelocity = 2 * Math.PI;
        /** Maximum angular acceleration during alignment (radians/second²). */
        public static final double kMaxTurnAcceleration = 2 * Math.PI;

        /**
         * Position and rotation tolerance for "at target" detection. TODO: May need adjustment
         */
        public static final Pose2d kTolerance = new Pose2d(
            0.03,
            0.03,
            new Rotation2d(0.05)
        );
    }
}
