package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.MathUtil.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Swerve drive configuration constants.
 *
 * <p>Contains all parameters required to configure a 4-module swerve drivetrain:
 * <ul>
 *   <li>Module positions and CAN IDs
 *   <li>Motor gear ratios and inversions
 *   <li>Encoder conversion factors
 *   <li>Wheel specifications (radius, friction coefficient)
 *   <li>Maximum speeds (linear and angular)
 *   <li>SwerveDriveKinematics for odometry and control
 * </ul>
 *
 * <p>All gear ratios follow the convention {@code 1 motorRotation : kReduction wheelRotations}.
 */
public final class SwerveConstants {

    /** Swerve drive kinematics object for odometry and chassis speed conversion. */
    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(
            Module.kFrontLeft.translation(),
            Module.kFrontRight.translation(),
            Module.kRearLeft.translation(),
            Module.kRearRight.translation()
        );

    /**
     * Swerve module hardware configuration.
     *
     * <p>Each module is assigned:
     * <ul>
     *   <li>A unique index value (0-3) for CAN ID calculation
     *   <li>Drive and turn motor CAN IDs (calculated from index)
     *   <li>Absolute encoder angular offset (zero = wheels forward)
     *   <li>Position relative to robot center (X-forward, Y-left)
     * </ul>
     *
     * <p>TODO: Season - Ensure controls team sets CAN IDs properly.
     * Should be FrontLeft=1x and go clockwise from top-down view.
     */
    public enum Module {
        // TODO: Season: make controls set these IDs. Should be FrontLeft=1x and go clockwise from top-down view
        kFrontLeft(0),
        kFrontRight(1),
        kRearLeft(3),
        kRearRight(2);

        public final int value;

        Module(int i) {
            this.value = i;
        }

        public int driveId() {
            return this.value * 10;
        }

        public int turnId() {
            return this.driveId() + 5;
        }

        public double angularOffset() {
            return switch (this) {
                case kFrontLeft -> -Math.PI / 2;
                case kFrontRight -> 0;
                case kRearLeft -> Math.PI;
                case kRearRight -> Math.PI / 2;
            };
        }

        public Translation2d translation() {
            double absX = RobotConstants.DriveBase.kWheelBase / 2;
            double absY = RobotConstants.DriveBase.kTrackWidth / 2;

            return switch (this) {
                case kFrontLeft -> new Translation2d(absX, absY);
                case kFrontRight -> new Translation2d(absX, -absY);
                case kRearLeft -> new Translation2d(-absX, absY);
                case kRearRight -> new Translation2d(-absX, -absY);
            };
        }
    }

    /**
     * Drive motor (NEO) configuration.
     *
     * <p>MAXSwerve module uses a multi-stage gearing system:
     * <ul>
     *   <li>Pinion gear (14T) on motor shaft
     *   <li>Bevel gear (45T) and bevel pinion (15T) for 90° turn
     *   <li>Spur gear (22T) final reduction to wheel
     * </ul>
     *
     * <p>Total reduction: {@code (14 * 15) / (45 * 22) = 0.2121}
     */
    public static final class DriveMotor {

        /**
         * Number of teeth on the pinion gear.
         *
         * <p>According to REV docs, can be 12T, 13T, or 14T.
         * This robot uses 14T for higher top speed.
         */
        public static final double kPinionTeeth = 14.0;

        /** Number of teeth on the wheel's bevel gear (45T). */
        public static final double kBevelGearTeeth = 45.0;

        /** Number of teeth on the first-stage spur gear (22T). */
        public static final double kSpurTeeth = 22.0;

        /** Number of teeth on the bevel pinion (15T). */
        public static final double kBevelPinionTeeth = 15.0;

        /**
         * Gear reduction from motor to wheel ({@code ~0.2121}).
         *
         * <p>Calculated as: {@code (pinion * bevel_pinion) / (bevel_gear * spur)}
         */
        public static final double kReduction =
            (kPinionTeeth * kBevelPinionTeeth) / (kBevelGearTeeth * kSpurTeeth);

        /** Moment of inertia of drive system (1.91e-4 kg·m²). */
        public static final double kMoI = 1.91e-4;

        /**
         * Static friction voltage threshold (0.1 volts).
         *
         * <p>Minimum voltage required to overcome static friction and turn the wheel.
         */
        public static final double kStaticFriction = 0.1;

        /** Idle mode set to brake (prevents coasting when disabled). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    /**
     * Turn motor (NEO 550) configuration.
     *
     * <p>Controls module orientation via position closed-loop with absolute encoder feedback.
     */
    public static final class TurnMotor {

        /** Moment of inertia of steering system (2.17e-5 kg·m²). */
        public static final double kMoI = 2.17e-5;

        /**
         * Gear reduction from turn motor to module rotation ({@code ~46.42}).
         *
         * <p>Calculated as: {@code 9424 / 203 = 46.42}
         */
        public static final double kReduction = 9424. / 203.;

        /** Idle mode set to brake (holds module angle when stopped). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    /**
     * Drive encoder conversion factors.
     *
     * <p>Converts NEO internal encoder readings to wheel motion units.
     */
    public static final class DriveEncoder {

        /**
         * Position conversion factor (motor rotations → wheel radians).
         *
         * <p>Calculated as: {@code 2π * gear_reduction = 1.171 rad/motor-rotation}
         */
        public static final double kPositionFactor =
            kTau * DriveMotor.kReduction;

        /**
         * Velocity conversion factor (motor RPM → wheel rad/s).
         *
         * <p>Calculated as: {@code position_factor / 60 = 0.01952 (rad/s)/RPM}
         */
        public static final double kVelocityFactor = kPositionFactor / 60.0;
    }

    /**
     * Turn encoder conversion factors.
     *
     * <p>Converts NEO 550 absolute encoder readings to module angle units.
     */
    public static final class TurnEncoder {

        /**
         * Position conversion factor (motor rotations → motor radians).
         *
         * <p>Absolute encoder directly measures motor shaft, not geared output.
         */
        public static final double kPositionFactor = kTau;

        /**
         * Velocity conversion factor (motor RPM → motor rad/s).
         *
         * <p>Calculated as: {@code 2π / 60 = 0.1047 (rad/s)/RPM}
         */
        public static final double kVelocityFactor = kPositionFactor / 60.0;

        /**
         * Invert the turn encoder.
         *
         * <p><strong>Never change this. Ever.</strong> Encoder inversion is critical
         * for correct module orientation control.
         */
        public static final boolean kInverted = true;
    }

    /**
     * Wheel physical specifications.
     *
     * <p>Determines maximum achievable speeds and traction limits.
     */
    public static final class Wheel {

        /** Radius of the wheel (1.5 inches = 0.0381 meters). */
        public static final double kRadius = Inches.of(1.5).in(Meters);

        /**
         * Angular free speed of the wheel (radians/second).
         *
         * <p>Calculated from NEO free speed and drive reduction:
         * {@code 594.7 rad/s * (1/5.36) = 110.9 rad/s}
         */
        public static final double kFreeSpeedAngular =
            MotorConstants.Neo.kFreeSpeed * DriveMotor.kReduction;

        /**
         * Linear free speed of the wheel (meters/second).
         *
         * <p>Calculated as: {@code angular_speed * radius = 4.23 m/s}
         */
        public static final double kFreeSpeedLinear =
            kFreeSpeedAngular * kRadius;

        /** Estimated coefficient of friction (1.3 for rubber on carpet). */
        public static final double kFrictionCoefficient = 1.3;
    }

    /**
     * Maximum robot speeds.
     *
     * <p>Defines velocity limits for trajectory planning and driver input scaling.
     */
    public static final class MaxSpeed {

        /** Maximum linear speed (4.804 m/s, ~15.8 ft/s). */
        public static final double kLinear = 4.804;

        /** Maximum angular speed (12.440 rad/s, ~2.0 rev/s). */
        public static final double kAngular = 12.440;
    }
}
