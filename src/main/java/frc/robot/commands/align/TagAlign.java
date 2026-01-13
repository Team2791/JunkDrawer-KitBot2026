package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;
import frc.robot.util.Elastic;
import frc.robot.util.MathUtil;
import java.util.List;

/**
 * Abstract command group for two-stage AprilTag-based alignment.
 *
 * <p>This {@link SequentialCommandGroup} performs a two-stage navigation approach:
 * <ol>
 *   <li>Stage 1 ({@code targetY}): Align Y position only - move to same Y as tag target
 *   <li>Stage 2 ({@code targetFinal}): Full alignment - move to final desired position and rotation
 * </ol>
 *
 * <p>The two-stage approach ensures smooth, stable alignment by first eliminating
 * one degree of freedom, then completing the approach.
 *
 * <p>Subclasses must implement {@link #getTagIds()} to provide the list of target
 * AprilTag IDs for this alignment (e.g., {@code [1, 2, 3]} for different game elements).
 *
 * <p><strong>Flow:</strong>
 * <ol>
 *   <li>InstantCommand: Find nearest tag and calculate target poses
 *   <li>{@link Navigate.Supplied}: Move to Stage 1 target (Y alignment)
 *   <li>{@link Navigate.Supplied}: Move to Stage 2 target (full alignment)
 *   <li>InstantCommand: Rumble controller to indicate success
 * </ol>
 *
 * <p><strong>Safety checks:</strong>
 * <ul>
 *   <li>Maximum distance threshold (default 4m): Fails if target too far
 *   <li>Tag existence check: Aborts if no visible tags
 * </ul>
 */
public abstract class TagAlign extends SequentialCommandGroup {

    /** Robot's desired offset from tag (where to position relative to the tag). */
    final Transform2d offset;
    /** Drivetrain for movement control. */
    final Drivetrain drivetrain;

    /** Stage 1 target: Only Y position aligned, robot X still adjusting. */
    Pose2d targetY;
    /** Stage 2 target: Full pose aligned (X, Y, rotation). */
    Pose2d targetFinal;

    /** Most recently selected AprilTag ID (-1 if none found). */
    int tagId = -1;

    /**
     * Constructs a TagAlign command group.
     *
     * <p>Sets up the command sequence:
     * <ol>
     *   <li>Update target pose based on current tag visibility
     *   <li>Navigate to Y-aligned position (Stage 1)
     *   <li>Navigate to final position (Stage 2)
     *   <li>Rumble controller on success
     * </ol>
     *
     * @param drivetrain Drivetrain subsystem for movement
     * @param offset Desired robot offset from tag center
     */
    public TagAlign(Drivetrain drivetrain, Transform2d offset) {
        this.offset = offset;
        this.drivetrain = drivetrain;

        addCommands(
            // Calculate target poses based on nearest visible tag
            new InstantCommand(this::updateTargetPose),
            // Stage 1: Navigate to Y-aligned position
            new Navigate.Supplied(drivetrain, () -> targetY),
            // Stage 2: Navigate to full final position
            new Navigate.Supplied(drivetrain, () -> targetFinal),
            // Success feedback
            new InstantCommand(Alerter.getInstance()::rumble)
        );
    }

    /**
     * Gets the list of AprilTag IDs that are valid targets for this alignment.
     *
     * <p>Subclasses should override to provide tag IDs specific to their alignment
     * goal (e.g., {@code [1, 2]} for left/right goals).
     *
     * @return List of valid AprilTag IDs, or null if none available
     */
    protected abstract List<Integer> getTagIds();

    /**
     * Updates target poses based on nearest visible AprilTag.
     *
     * <p>Called at command start. Process:
     * <ol>
     *   <li>Get list of valid target tag IDs
     *   <li>Find nearest tag from the valid list
     *   <li>Calculate Stage 1 target (Y-aligned only)
     *   <li>Calculate Stage 2 target (full pose aligned)
     *   <li>Check distance threshold
     *   <li>Abort with notification if too far
     * </ol>
     *
     * <p><strong>Coordinate Transformation:</strong>
     * <ul>
     *   <li>{@code tagToRobot}: Current robot position relative to nearest tag
     *   <li>{@code tagToRobotX}: Intermediate pose (only adjust X from current, Y matches tag)
     *   <li>{@code tagToPose}: Final pose (apply full offset from tag)
     * </ul>
     */
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    private void updateTargetPose() {
        List<Integer> targetIds = getTagIds();

        if (targetIds == null) return;

        Pose2d robotPose = drivetrain.getPose();
        Pose2d tagPose = null;

        // Find the nearest tag from the valid target list
        double distance = Double.POSITIVE_INFINITY;
        for (int id : targetIds) {
            Pose2d pose = VisionConstants.AprilTag.kLayout
                .getTagPose(id)
                .get()
                .toPose2d();
            double d = robotPose
                .getTranslation()
                .getDistance(pose.getTranslation());

            if (d < distance) {
                distance = d;
                tagPose = pose;
                tagId = id;
            }
        }

        if (tagPose == null) {
            System.out.println("AlignClosest: no provided apriltags exist");
            tagId = -1;
            return;
        }

        // Calculate target poses by applying transformations relative to the tag

        // Current position relative to the tag
        Transform2d tagToRobot = MathUtil.transformationOf(tagPose, robotPose);

        // Stage 1 target: Keep robot's current X distance from tag, align Y
        // Uses current X distance but tag's desired Y offset
        Transform2d tagToRobotX = new Transform2d(
            tagToRobot.getX(), // Keep current X distance
            offset.getY(), // Use tag's Y offset
            offset.getRotation() // Use tag's rotation offset
        );

        // Stage 2 target: Apply full offset transformation from tag
        Transform2d tagToPose = new Transform2d(
            offset.getX(),
            offset.getY(),
            offset.getRotation()
        );

        // Calculate actual target poses by transforming from tag frame
        targetY = tagPose.transformBy(tagToRobotX);
        targetFinal = tagPose.transformBy(tagToPose);

        // Check if target is within maximum distance threshold
        double dist = robotPose
            .getTranslation()
            .getDistance(targetFinal.getTranslation());

        if (dist >= VisionConstants.Align.kMaxDistance) {
            // Target too far - send warning notification and abort
            Elastic.sendNotification(
                new Elastic.Notification(
                    Elastic.Notification.NotificationLevel.WARNING,
                    "Alignment: Too far away",
                    "%fm > %fm.".formatted(
                        dist,
                        VisionConstants.Align.kMaxDistance
                    )
                )
            );

            // Reset state - command will not proceed
            tagId = -1;
            targetY = null;
            targetFinal = null;
        }
    }

    /**
     * Gets the ID of the most recently selected AprilTag.
     *
     * @return Tag ID that was selected, or -1 if no tag found/selected
     */
    public int getTagId() {
        return tagId;
    }
}
