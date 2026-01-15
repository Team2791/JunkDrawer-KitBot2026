package frc.robot.commands.pathfind;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * Command for nearby pose-targeted navigation using PIDs.
 *
 * <p>This command is used as a helper for {@link Pathfind}. It is used as the final step: close to
 * the target, switch to PID control.
 *
 * <p>Navigate uses a {@link HolonomicDriveController} with three independent PID controllers to
 * move the robot to a target pose. The controller calculates the required chassis speeds (x, y,
 * rotation) to reach the target.
 */
class Nearby extends Command {

  /** Get the supplier on {@link #initialize()} */
  final Supplier<Pose2d> target;

  /** PID for X position (field-relative). */
  PIDController xController =
      new PIDController(
          ControlConstants.Nearby.kOrthoP,
          ControlConstants.Nearby.kOrthoI,
          ControlConstants.Nearby.kOrthoD);
  /** PID for Y position (field-relative). */
  PIDController yController =
      new PIDController(
          ControlConstants.Nearby.kOrthoP,
          ControlConstants.Nearby.kOrthoI,
          ControlConstants.Nearby.kOrthoD);
  /** Profiled PID for rotation with velocity/acceleration constraints. */
  ProfiledPIDController rotController =
      new ProfiledPIDController(
          ControlConstants.Nearby.kTurnP,
          ControlConstants.Nearby.kTurnI,
          ControlConstants.Nearby.kTurnD,
          new TrapezoidProfile.Constraints(
              ControlConstants.Nearby.kMaxTurnVelocity,
              ControlConstants.Nearby.kMaxTurnAcceleration));

  /** High-level holonomic controller combining all three PID loops. */
  final HolonomicDriveController controller =
      new HolonomicDriveController(xController, yController, rotController);

  /** Drivetrain subsystem for motion control. */
  final Drive drivetrain;
  /** Current target pose being navigated to. */
  Pose2d currentTarget;

  /** Flag to exit early if target pose is invalid. */
  boolean exit = false;

  /**
   * Constructs a Nearby command.
   *
   * <p>Initializes the HolonomicDriveController with the three PID controllers, enables continuous
   * input for rotation (angles wrap around), and sets the position tolerance for determining when
   * the target is reached.
   *
   * @param drivetrain Drivetrain subsystem for movement control
   * @param target The target supplier
   */
  Nearby(Drive drivetrain, Supplier<Pose2d> target) {
    this.drivetrain = drivetrain;
    this.target = target;

    // Enable continuous input: angles wrap at 2π so -0.1 rad = 2π - 0.1 rad
    rotController.enableContinuousInput(0, 2 * Math.PI);
    // Set tolerance threshold for "at reference" check
    controller.setTolerance(ControlConstants.Nearby.kTolerance);

    addRequirements(drivetrain);
  }

  /**
   * Command initialization: Get target pose and prepare for navigation.
   *
   * <p>Called once when the command starts. Fetches the target pose from {@link #target}, logs it,
   * and displays it on the field view for debugging.
   */
  @Override
  public final void initialize() {
    exit = false;
    currentTarget = target.get();

    if (currentTarget == null) {
      exit = true;
      return;
    }
  }

  /**
   * Command execution: Calculate and send chassis speeds to reach target.
   *
   * <p>Uses the {@link HolonomicDriveController} to calculate the required chassis speeds (vx, vy,
   * ω) based on current pose, target pose, and controller gains.
   */
  @Override
  public final void execute() {
    if (currentTarget == null) return;

    Pose2d robot = drivetrain.getPose();
    // Calculate speeds using holonomic controller, with a target velocity of 1 cm/s (0.01 m/s)
    ChassisSpeeds speeds =
        controller.calculate(robot, currentTarget, 0.01, currentTarget.getRotation());
    // Send robot-relative speeds to drivetrain
    drivetrain.runVelocity(speeds);
  }

  /**
   * Command end: Stop robot and clean up target visualization.
   *
   * <p>Called when the command finishes (either by {@link #isFinished()} or interruption). Removes
   * the target from the field visualization and stops the robot.
   *
   * @param interrupted true if command was interrupted, false if finished normally
   */
  @Override
  @OverridingMethodsMustInvokeSuper
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.runVelocity(new ChassisSpeeds());
  }

  /**
   * Checks if the robot has reached the target pose.
   *
   * <p>Uses the {@link HolonomicDriveController}'s tolerance checking to determine if both position
   * and rotation are within acceptable thresholds.
   *
   * @return true if at target within tolerance, false otherwise
   */
  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
