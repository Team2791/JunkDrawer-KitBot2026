package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * FRC 2026 game-specific field dimensions and reference points.
 *
 * <p>Contains field geometry constants derived from the official game manual.
 * All dimensions are converted from inches to meters.
 *
 * <p>These values define the coordinate system used for autonomous navigation
 * and alliance-aware positioning.
 *
 * <p>Sources:
 * <ul>
 *   <li><a href="https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf">2026 Field Dimension Drawings</a>
 * </ul>
 */
@SuppressWarnings("SuspiciousNameCombination")
public class GameConstants {

    private GameConstants() {}

    /** Field width (short dimension) */
    public static final double kFieldWidth = Inches.of(317.69).in(Meters);

    /** Field length (long dimension) */
    public static final double kFieldLength = Inches.of(651.22).in(Meters);

    /**
     * Red alliance coordinate system origin.
     *
     * <p>Located at the far corner of the field with {@code 180°} rotation.
     */
    public static final Pose2d kRedOrigin = new Pose2d(
        kFieldLength,
        kFieldWidth,
        Rotation2d.kPi
    );
}
