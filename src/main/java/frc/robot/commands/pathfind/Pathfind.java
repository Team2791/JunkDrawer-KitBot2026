package frc.robot.commands.pathfind;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract command for pose-based robot navigation using holonomic drive control.
 *
 * <p>This is an abstract command that subclasses override via {@link Supplied} or by implementing
 * {@link #target()}.
 *
 * <p>Default concrete implementation: {@link Supplied}
 */
public abstract class Pathfind extends SequentialCommandGroup {

  /**
   * Concrete {@link Pathfind} implementation that accepts a target pose via Supplier.
   *
   * <p>Useful for dynamic targets that may change during the command, or for creating {@link
   * Pathfind} instances with a fixed target pose.
   */
  public static class Supplied extends Pathfind {

    /** Supplier for the target pose (can be fixed or dynamic). */
    final Supplier<Pose2d> target;

    /**
     * Constructs {@link Pathfind} with a dynamic target supplier.
     *
     * @param drivetrain Drivetrain subsystem for movement control
     * @param target Supplier providing target pose each execute cycle
     */
    public Supplied(Drive drivetrain, Supplier<Pose2d> target) {
      super(drivetrain);
      this.target = target;
    }

    /**
     * Constructs {@link Pathfind} with a fixed target pose.
     *
     * @param drivetrain Drivetrain subsystem for movement control
     * @param target Fixed target pose for this navigation
     */
    public Supplied(Drive drivetrain, Pose2d target) {
      this(drivetrain, () -> target);
    }

    @Override
    protected Pose2d target() {
      return target.get();
    }
  }

  /** Cached target pose for this navigation. */
  Pose2d cache;

  final Drive drivetrain;

  /**
   * Constructs a Navigate command.
   *
   * <ol>
   *   <li>Calls {@link #target()} to get the target pose and caches it.
   *   <li>Logs the target pose to the field for visualization.
   *   <li>Runs {@link Repulse} for faraway pathfinding.
   *   <li>Runs {@link Nearby} for close-range precision movement.
   *   <li>Clears the target pose from the field logs.
   * </ol>
   *
   * @param drivetrain Drivetrain subsystem for movement control
   */
  public Pathfind(Drive drivetrain) {
    this.drivetrain = drivetrain;

    addCommands(
        new InstantCommand(this::prepare),
        new Repulse(drivetrain, this::currentTarget),
        new Nearby(drivetrain, this::currentTarget),
        new InstantCommand(this::release));
  }

  /**
   * Gets the target pose for this navigation.
   *
   * <p>Subclasses must implement this to provide the target, which may be constant or dynamic
   * (e.g., based on vision measurements).
   *
   * @return Target pose, or null if no valid target
   */
  protected abstract Pose2d target();

  /** Caches and logs the target pose */
  protected final void prepare() {
    cache = target();

    // drivetrain.getField().getObject("Pathfind/Target").setPose(cache);
    Logger.recordOutput("Pathfind/Target", cache);
  }

  /** Deletes the target pose from the logs */
  protected final void release() {
    Pose2d invalid = new Pose2d(-1, -1, new Rotation2d());
    // drivetrain.getField().getObject("Pathfind/Target").setPose(invalid);
    Logger.recordOutput("Pathfind/Target", invalid);
  }

  /**
   * Gets the currently cached target pose.
   *
   * @return The cached target pose
   */
  protected final Pose2d currentTarget() {
    return cache;
  }
}
