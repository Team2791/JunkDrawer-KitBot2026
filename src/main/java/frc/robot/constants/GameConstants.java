package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * FRC 2026 game-specific field dimensions and reference points.
 *
 * <p>Contains field geometry constants derived from the official game manual.
 * All dimensions are converted from inches to meters for consistency with WPILib.
 *
 * <p>These values define the coordinate system used for autonomous navigation
 * and alliance-aware positioning.
 */
@SuppressWarnings("SuspiciousNameCombination")
public class GameConstants {

    /** Field width (short dimension, 317 inches = 8.0518 meters). */
    public static final double kFieldWidth = Inches.of(317).in(Meters);

    /** Field length (long dimension, 690.875 inches = 17.548225 meters). */
    public static final double kFieldLength = Inches.of(690.875).in(Meters);

    /**
     * Red alliance coordinate system origin.
     *
     * <p>Located at the far corner of the field with {@code 180°} rotation.
     * Used by {@link frc.robot.util.AllianceUtil} for coordinate flipping.
     */
    public static final Pose2d kRedOrigin = new Pose2d(
        kFieldLength,
        kFieldWidth,
        Rotation2d.kPi
    );
}
