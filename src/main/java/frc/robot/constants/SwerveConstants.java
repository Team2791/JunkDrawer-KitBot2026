package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.MathUtil.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class SwerveConstants {

    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(
            Module.kFrontLeft.translation(),
            Module.kFrontRight.translation(),
            Module.kRearLeft.translation(),
            Module.kRearRight.translation()
        );

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

    public static final class DriveMotor {

        /** Number of teeth on the pinion gear. According to docs, either 12, 13, or 14T */
        public static final double kPinionTeeth = 14.0;

        /** Number of teeth on the wheel's bevel gear */
        public static final double kBevelGearTeeth = 45.0;

        /** Number of teeth on the first-stage spur gear */
        public static final double kSpurTeeth = 22.0;

        /** Number of teeth on the bevel pinion */
        public static final double kBevelPinionTeeth = 15.0;

        /** Reduction factor from motor to wheel */
        public static final double kReduction =
            (kPinionTeeth * kBevelPinionTeeth) / (kBevelGearTeeth * kSpurTeeth);

        /** Moment of inertia */
        public static final double kMoI = 1.91e-4;

        /** The minimum amount of voltage that can turn the drive motor */
        public static final double kStaticFriction = 0.1;

        /** Idle mode, can be either brake or coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public static final class TurnMotor {

        /** Moment of inertia */
        public static final double kMoI = 2.17e-5;

        /** Reduction factor */
        public static final double kReduction = 9424. / 203.;

        /** Idle mode, can be either brake or coast */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public static final class DriveEncoder {

        /** Convert from motor-rotations to wheel-radians */
        public static final double kPositionFactor =
            kTau * DriveMotor.kReduction;

        /** Convert from motor rotations per minute to wheel radians per second */
        public static final double kVelocityFactor = kPositionFactor / 60.0;
    }

    public static final class TurnEncoder {

        /** Convert from motor-rotations to motor-radians */
        public static final double kPositionFactor = kTau;

        /** Convert from motor rotations per minute to motor radians per second */
        public static final double kVelocityFactor = kPositionFactor / 60.0;

        /** Invert the turn encoder. Never change this. Ever. */
        public static final boolean kInverted = true;
    }

    public static final class Wheel {

        /** Radius of the wheel, per design */
        public static final double kRadius = Inches.of(1.5).in(Meters);

        /** Angular free speed of the wheel */
        public static final double kFreeSpeedAngular =
            MotorConstants.Neo.kFreeSpeed * DriveMotor.kReduction;

        /** Linear free speed of the wheel */
        public static final double kFreeSpeedLinear =
            kFreeSpeedAngular * kRadius;

        /** Estimated wheel CoF, per design */
        public static final double kFrictionCoefficient = 1.3;
    }

    public static final class MaxSpeed {

        public static final double kLinear = 4.804;
        public static final double kAngular = 12.440;
    }
}
