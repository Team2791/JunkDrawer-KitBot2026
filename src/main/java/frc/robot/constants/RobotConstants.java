package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import frc.robot.util.Vec2;

/**
 * Global robot physical properties and dimensions.
 *
 * <p>Contains mass, moment of inertia, and drive base geometry used for
 * dynamics modeling, path planning, and collision avoidance.
 */
public class RobotConstants {

    /** Robot mass including battery and bumpers. */
    public static final double kMass = 68.023;

    /** Robot moment of inertia about vertical axis. */
    public static final double kMoI = 4.235;

    /**
     * Swerve drive base geometric dimensions.
     *
     * <p>Defines wheelbase, track width, bumper extents, and radii for
     * kinematics calculations and obstacle clearance checks.
     */
    public static final class DriveBase {

        /**
         * Distance between swerve modules (21.5 inches = 0.5461 meters).
         *
         * <p>Square configuration: X and Y distances are equal.
         */
        public static final Vec2 kDimensions = Vec2.fill(
            Inches.of(21.5).in(Meters)
        );

        /** Wheelbase: front-to-back distance between modules (meters). */
        public static final double kWheelBase = kDimensions.x;

        /** Track width: side-to-side distance between modules (meters). */
        public static final double kTrackWidth = kDimensions.y;

        /** Bumper width: side-to-side outer dimension (31.5 inches = 0.8001 meters). */
        public static final double kBumperWidth = Inches.of(31.5).in(Meters);

        /** Bumper length: front-to-back outer dimension (31.5 inches = 0.8001 meters). */
        public static final double kBumperLength = Inches.of(31.5).in(Meters);

        /**
         * Bumper corner radius (diagonal half-distance).
         *
         * <p>Used for circular collision detection. Calculated as:
         * {@code 0.5 * sqrt(length² + width²) = 0.5661 meters}
         */
        public static final double kBumperRadius =
            0.5 * Math.hypot(kBumperLength, kBumperWidth);

        /**
         * Drive base radius (module diagonal half-distance).
         *
         * <p>Used for angular velocity calculations. Calculated as:
         * {@code 0.5 * sqrt(wheelbase² + trackWidth²) = 0.3863 meters}
         */
        public static final double kDriveRadius =
            0.5 * Math.hypot(kWheelBase, kTrackWidth);
    }
}
