package frc.robot.constants;

import static frc.robot.util.MathUtil.kTau;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Control loop constants for robot motion control.
 *
 * <p>Contains PID gains, feedforward coefficients, and control limits for:
 * <ul>
 *   <li>Swerve module drive motors (velocity control)
 *   <li>Swerve module turn motors (position control)
 *   <li>Autonomous trajectory following
 *   <li>Vision-based alignment
 *   <li>Joystick input rate limiting
 * </ul>
 */
public final class ControlConstants {

    /** Gyroscope direction multiplier ({@code -1.0} inverts gyro readings). */
    public static final double kGyroFactor = -1.0;

    /**
     * Drive motor velocity control PID constants.
     *
     * <p>These gains control the closed-loop velocity regulation for each
     * swerve module's drive motor.
     */
    public static final class DrivetrainDrive {

        /** Proportional gain for velocity error. */
        public static final double kP = 0.004;
        /** Integral gain for accumulated velocity error. */
        public static final double kI = 1e-10;
        /** Derivative gain for velocity error rate of change. */
        public static final double kD = 0.0002;

        /** Static friction feedforward (volts). TODO: Tune with characterization. */
        public static final double kS = 0.0;
        /** Velocity feedforward (volts per meter/second). TODO: Tune with characterization. */
        public static final double kV = 0.0;
        /** Acceleration feedforward (volts per meter/second²). TODO: Tune with characterization. */
        public static final double kA = 0.0;

        /** Minimum motor output (duty cycle). */
        public static final double kMin = -1.0;
        /** Maximum motor output (duty cycle). */
        public static final double kMax = 1.0;
    }

    /**
     * Turn motor position control PID constants.
     *
     * <p>These gains control the closed-loop position regulation for each
     * swerve module's steering motor.
     */
    public static final class DrivetrainTurn {

        /** Proportional gain for angular position error. */
        public static final double kP = 2.00;
        /** Integral gain for accumulated angular error. */
        public static final double kI = 0.00;
        /** Derivative gain for angular error rate of change. */
        public static final double kD = 0.00;

        /** Minimum controller output. */
        public static final double kMinOutput = -1.0;
        /** Maximum controller output. */
        public static final double kMaxOutput = 1.0;

        /** Minimum angle for continuous input (radians). */
        public static final double kMinInput = 0;
        /** Maximum angle for continuous input (radians, wraps at 2π). */
        public static final double kMaxInput = kTau;
    }

    /**
     * Autonomous trajectory following PID constants.
     *
     * <p>Used by {@link frc.robot.autos.AutoManager} for Choreo trajectory following.
     */
    public static final class Auto {

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
    }

    /**
     * Vision-based alignment PID constants.
     *
     * <p>Used by {@link frc.robot.commands.align.Navigate} and
     * {@link frc.robot.commands.align.TagAlign} for precise positioning.
     */
    public static final class Align {

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

        /** Maximum angular velocity during alignment (radians/second). TODO: May be increased. */
        public static final double kMaxTurnVelocity = kTau;
        /** Maximum angular acceleration during alignment (radians/second²). */
        public static final double kMaxTurnAcceleration = kTau;

        /** Position and rotation tolerance for "at target" detection. TODO: May need adjustment for shooter vs P&P. */
        public static final Pose2d kTolerance = new Pose2d(
            0.03,
            0.03,
            new Rotation2d(0.05)
        );
    }

    /**
     * Slew rate limiters for driver input smoothing.
     *
     * <p>Limits acceleration of joystick commands to prevent abrupt movements
     * and reduce mechanical stress on the drivetrain.
     */
    public static final class SlewRateLimit {

        /** Maximum translational acceleration (meters/second²). TODO: Can be increased (less top-heavy than previous design). */
        public static final double kOrthogonal = 1.667;
        /** Maximum rotational acceleration (radians/second²). */
        public static final double kRotation = 3.87;
    }
}
