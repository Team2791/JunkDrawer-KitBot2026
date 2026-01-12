package frc.robot.constants;

import static frc.robot.util.MathUtil.kTau;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** PID constants, mostly */
public final class ControlConstants {
    public static final double kGyroFactor = -1.0;

    public static final class DrivetrainDrive {
        public static final double kP = 0.004;
        public static final double kI = 1e-10;
        public static final double kD = 0.0002;
        public static final double kS = 0.0; // TODO: season: tune (claude will help with FF constants)
		public static final double kV = 0.0;
		public static final double kA = 0.0;

        public static final double kMin = -1.0;
        public static final double kMax = 1.0;
    }

    public static final class DrivetrainTurn {
        public static final double kP = 2.00;
        public static final double kI = 0.00;
        public static final double kD = 0.00;

        public static final double kMinOutput = -1.0;
        public static final double kMaxOutput = 1.0;

        public static final double kMinInput = 0;
        public static final double kMaxInput = kTau;
    }

    public static final class Auto {
        public static final double kOrthoP = 1.25;
        public static final double kOrthoI = 0.00;
        public static final double kOrthoD = 0.00;

        public static final double kTurnP = 0.00;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.00;
    }

    public static final class Align {
        public static final double kOrthoP = 4.75;
        public static final double kOrthoI = 0.00;
        public static final double kOrthoD = 0.00;

        public static final double kTurnP = 2.60;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.04;

        public static final double kMaxTurnVelocity = kTau; // TODO: season: we may be able to increase this
        public static final double kMaxTurnAcceleration = kTau;

        public static final Pose2d kTolerance = new Pose2d(0.03, 0.03, new Rotation2d(0.05)); // TODO: season: may want to increase because shooter, not P&P
    }

    public static final class SlewRateLimit {
        public static final double kOrthogonal = 1.667; // TODO: Season: we can raise this due to no top-heavyness
        public static final double kRotation = 3.87;
    }
}