package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import frc.robot.util.Vec2;

public class RobotConstants {

    public static final double kMass = 68.023;
    public static final double kMoI = 4.235;

    public static final class DriveBase {

        /** length and width between swerve modules */
        public static final Vec2 kDimensions =  Vec2.fill(Inches.of(21.5).in(Meters));

        public static final double kWheelBase = kDimensions.x;
        public static final double kTrackWidth = kDimensions.y;

        /** Length and width between bumpers */
        public static final double kBumperWidth = Inches.of(31.5).in(Meters);

        public static final double kBumperLength = Inches.of(31.5).in(Meters);

        public static final double kBumperRadius =
            0.5 * Math.hypot(kBumperLength, kBumperWidth);

        /** Drive base radius */
        public static final double kDriveRadius = 0.5 * Math.hypot(kWheelBase, kTrackWidth);
    }
}
