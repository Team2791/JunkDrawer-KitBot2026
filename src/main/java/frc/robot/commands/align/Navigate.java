package frc.robot.commands.align;

import static frc.robot.util.MathUtil.kTau;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract command for pose-based robot navigation using holonomic drive control.
 *
 * <p>Navigate uses a {@link HolonomicDriveController} with three independent PID controllers
 * to move the robot to a target pose. The controller calculates the required
 * chassis speeds (x, y, rotation) to reach the target.
 *
 * <p>This is an abstract command that subclasses override via {@link Supplied} or
 * by implementing {@link #getTargetPose()}.
 *
 * <p><strong>PID Controllers:</strong>
 * <ul>
 *   <li>X Controller (orthogonal): Aligns robot X position
 *   <li>Y Controller (orthogonal): Aligns robot Y position
 *   <li>Rotation Controller (profiled): Aligns robot heading with velocity/acceleration constraints
 * </ul>
 *
 * <p>The rotation controller uses continuous input (0 to 2π) so angles wrap correctly
 * (e.g., -0.1 rad is treated as 2π - 0.1).
 *
 * <p>Default concrete implementation: {@link Supplied}
 */
public abstract class Navigate extends Command {

    /**
     * Concrete Navigate implementation that accepts a target pose via Supplier.
     *
     * Useful for dynamic targets that may change during the command, or for
     * creating Navigate instances with a fixed target pose.
     */
    public static class Supplied extends Navigate {

        /** Supplier for the target pose (can be fixed or dynamic). */
        final Supplier<Pose2d> target;

        /**
         * Constructs Navigate with a dynamic target supplier.
         *
         * @param drivetrain Drivetrain subsystem for movement control
         * @param target Supplier providing target pose each execute cycle
         */
        public Supplied(Drivetrain drivetrain, Supplier<Pose2d> target) {
            super(drivetrain);
            this.target = target;
        }

        /**
         * Constructs Navigate with a fixed target pose.
         *
         * @param drivetrain Drivetrain subsystem for movement control
         * @param target Fixed target pose for this navigation
         */
        public Supplied(Drivetrain drivetrain, Pose2d target) {
            this(drivetrain, () -> target);
        }

        @Override
        protected Pose2d getTargetPose() {
            return target.get();
        }
    }

    /** PID for X position (field-relative). */
    PIDController xController = new PIDController(
        ControlConstants.Align.kOrthoP,
        ControlConstants.Align.kOrthoI,
        ControlConstants.Align.kOrthoD
    );
    /** PID for Y position (field-relative). */
    PIDController yController = new PIDController(
        ControlConstants.Align.kOrthoP,
        ControlConstants.Align.kOrthoI,
        ControlConstants.Align.kOrthoD
    );
    /** Profiled PID for rotation with velocity/acceleration constraints. */
    ProfiledPIDController rotController = new ProfiledPIDController(
        ControlConstants.Align.kTurnP,
        ControlConstants.Align.kTurnI,
        ControlConstants.Align.kTurnD,
        new TrapezoidProfile.Constraints(
            ControlConstants.Align.kMaxTurnVelocity,
            ControlConstants.Align.kMaxTurnAcceleration
        )
    );

    /** High-level holonomic controller combining all three PID loops. */
    final HolonomicDriveController controller;

    /** Drivetrain subsystem for motion control. */
    final Drivetrain drivetrain;
    /** Current target pose being navigated to. */
    Pose2d currentTarget;

    /** Flag to exit early if target pose is invalid. */
    boolean exit = false;

    /**
     * Constructs a Navigate command.
     *
     * Initializes the HolonomicDriveController with the three PID controllers,
     * enables continuous input for rotation (angles wrap around), and sets the
     * position tolerance for determining when the target is reached.
     *
     * @param drivetrain Drivetrain subsystem for movement control
     */
    public Navigate(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.controller = new HolonomicDriveController(
            xController,
            yController,
            rotController
        );

        // Enable continuous input: angles wrap at 2π so -0.1 rad = 2π - 0.1 rad
        rotController.enableContinuousInput(0, kTau);
        // Set tolerance threshold for "at reference" check
        controller.setTolerance(ControlConstants.Align.kTolerance);

        addRequirements(drivetrain);
    }

    /**
     * Gets the target pose for this navigation.
     *
     * Subclasses must implement this to provide the target, which may be
     * constant or dynamic (e.g., based on vision measurements).
     *
     * @return Target pose, or null if no valid target
     */
    protected abstract Pose2d getTargetPose();

    /**
     * Command initialization: Get target pose and prepare for navigation.
     *
     * Called once when the command starts. Fetches the target pose from
     * {@link #getTargetPose()}, logs it, and displays it on the field view
     * for debugging.
     */
    @Override
    public final void initialize() {
        exit = false;
        currentTarget = getTargetPose();

        if (currentTarget == null) {
            exit = true;
            return;
        }

        Logger.recordOutput("Nearby/Target", currentTarget);
        drivetrain.getField().getObject("Nearby/Target").setPose(currentTarget);
    }

    /**
     * Command execution: Calculate and send chassis speeds to reach target.
     *
     * <p>Called periodically (~50 Hz) while the command is running. Uses the
     * {@link HolonomicDriveController} to calculate the required chassis speeds (vx, vy, ω)
     * based on current pose, target pose, and controller gains.
     */
    @Override
    public final void execute() {
        if (currentTarget == null) return;

        Pose2d robot = drivetrain.pose();
        // Calculate speeds using holonomic controller
        // Period (0.01s) is approximate for WPILib timing
        // Target rotation is used as the desired angular velocity direction
        ChassisSpeeds speeds = controller.calculate(
            robot,
            currentTarget,
            0.01,
            currentTarget.getRotation()
        );
        // Send field-relative speeds (controller output is in field coordinates)
        drivetrain.set(speeds);
    }

    /**
     * Command end: Stop robot and clean up target visualization.
     *
     * <p>Called when the command finishes (either by {@link #isFinished()} or interruption).
     * Removes the target from the field visualization and stops the robot.
     *
     * @param interrupted true if command was interrupted, false if finished normally
     */
    @Override
    @OverridingMethodsMustInvokeSuper
    public void end(boolean interrupted) {
        // Remove target from field visualization by moving it off-field
        drivetrain
            .getField()
            .getObject("Nearby/Target")
            .setPose(new Pose2d(-1, -1, new Rotation2d()));
        Logger.recordOutput(
            "Nearby/Target",
            new Pose2d(-1, -1, new Rotation2d())
        );
        // Stop the robot
        drivetrain.drive(new ChassisSpeeds());
    }

    /**
     * Checks if the robot has reached the target pose.
     *
     * <p>Uses the {@link HolonomicDriveController}'s tolerance checking to determine if
     * both position and rotation are within acceptable thresholds.
     *
     * @return true if at target within tolerance, false otherwise
     */
    @Override
    public boolean isFinished() {
        return controller.atReference();
    }
}
