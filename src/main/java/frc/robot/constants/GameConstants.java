package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

@SuppressWarnings("SuspiciousNameCombination")
public class GameConstants {
    public static final double kFieldWidth = Inches.of(317).in(Meters);
    public static final double kFieldLength = Inches.of(690.875).in(Meters);
    public static final Pose2d kRedOrigin = new Pose2d(kFieldLength, kFieldWidth, Rotation2d.kPi);
}
