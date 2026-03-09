package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import frc.robot.subsystems.drive.DriveConstants;

/**
 * Global robot physical properties and dimensions.
 *
 * TODO: Update values based on actual measurements and Ross
 * TODO: Also, maybe define these values here instead of extracting from DriveConstants, where they don't belong
 */
public class RobotConstants {

    private RobotConstants() {}

    /** Robot mass including battery and bumpers. TODO: measure */
    public static final double kMass = DriveConstants.robotMassKg;

    /**
     * Robot moment of inertia about vertical axis.
     * TODO: Ross will calculate this
     */
    public static final double kMoI = DriveConstants.robotMOI;

    /**
     * Swerve drive base geometric dimensions.
     *
     * <p>Defines wheelbase, track width, bumper extents, and radii for
     * kinematics calculations and obstacle clearance checks.
     */
    public static final class DriveBase {

        private DriveBase() {}

        /**
         * Wheel base: front-to-back distance between modules (meters).
         * @see #kBumperLength for front-to-back outer dimension
         */
        public static final double kWheelBase = DriveConstants.wheelBase;

        /**
         * Track width: side-to-side distance between modules (meters).
         * @see #kBumperWidth for side-to-side outer dimension
         */
        public static final double kTrackWidth = DriveConstants.trackWidth;

        /**
         * Bumper width: side-to-side outer dimension
         * @see #kTrackWidth for distance between wheels in the same axis
         */
        public static final double kBumperWidth = Inches.of(33.5).in(Meters);

        /**
         * Bumper length: front-to-back outer dimension
         * @see #kWheelBase for distance between wheels in the same axis
         */
        public static final double kBumperLength = Inches.of(31.5).in(Meters);

        /**
         * Radius of circle bumpers fit within.
         */
        public static final double kBumperRadius =
            0.5 * Math.hypot(kBumperLength, kBumperWidth);

        /**
         * Drive base radius (module diagonal half-distance).
         */
        public static final double kDriveRadius =
            DriveConstants.driveBaseRadius;
    }

    /**
     * Wheel physical properties.
     */
    public static final class Wheel {

        private Wheel() {}

        /** Wheel radius in meters. */
        public static final double kRadius = DriveConstants.wheelRadiusMeters;

        /** Coefficient of friction between wheel and carpet. */
        public static final double kFriction = DriveConstants.wheelCOF;
    }
}
