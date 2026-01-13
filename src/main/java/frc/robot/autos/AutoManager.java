package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.AllianceUtil;
import java.io.File;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

/**
 * Manages autonomous routines using Choreo trajectory following.
 *
 * <p>This class integrates with the Choreo library to create and execute autonomous
 * routines with precise trajectory following. It handles:
 * <ul>
 *   <li>Loading pre-generated trajectories from the deploy directory
 *   <li>Following trajectories with PID-based feedback control
 *   <li>Alliance-aware coordinate transformation for symmetric routines
 *   <li>Dashboard visualization of active trajectories
 * </ul>
 *
 * <p>The manager uses three independent PID controllers (X, Y, rotation) to correct
 * deviations from the desired trajectory path. Trajectories are automatically flipped
 * based on alliance color to support symmetric autonomous routines.
 */
public class AutoManager {

    /** PID controller for X-axis position tracking. */
    final PIDController xController = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );
    /** PID controller for Y-axis position tracking. */
    final PIDController yController = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );
    /** PID controller for rotation tracking. */
    final PIDController rotController = new PIDController(
        ControlConstants.Auto.kTurnP,
        ControlConstants.Auto.kTurnI,
        ControlConstants.Auto.kTurnD
    );

    /** Choreo auto factory for creating and managing autonomous routines. */
    final AutoFactory factory;
    /** Reference to the drivetrain subsystem. */
    final Drivetrain drivetrain;

    /**
     * Constructs an AutoManager with trajectory loading and factory setup.
     *
     * <p>This constructor:
     * <ul>
     *   <li>Initializes the Choreo {@link AutoFactory} with trajectory following callbacks
     *   <li>Loads all {@code .traj} files from the deploy/choreo directory
     *   <li>Configures continuous input for the rotation controller (wrapping at ±π)
     *   <li>Sets up dashboard visualization for the active trajectory
     * </ul>
     *
     * <p>The factory is configured to automatically flip trajectories based on alliance
     * color and visualize them on the field widget.
     *
     * @param drivetrain The drivetrain subsystem for trajectory following
     */
    public AutoManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.factory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose,
            this::follow,
            true,
            drivetrain,
            (traj, starting) -> {
                if (starting) {
                    Pose2d[] poses = traj.getPoses();
                    // SAFETY: this callback should only be run once we are in autonomous
                    // which can't happen unless FMS is connected
                    Pose2d[] recentered = AllianceUtil.unsafe.autoflip(poses);
                    Logger.recordOutput("Auto/CurrentTrajectory", recentered);
                    drivetrain
                        .getField()
                        .getObject("Auto/CurrentTrajectory")
                        .setPoses(recentered);
                } else {
                    Logger.recordOutput(
                        "Auto/CurrentTrajectory",
                        new Pose2d[0]
                    );
                    drivetrain
                        .getField()
                        .getObject("Auto/CurrentTrajectory")
                        .setPoses();
                }
            }
        );

        String choreo = Filesystem.getDeployDirectory().getPath() + "/choreo";
        for (File f : Objects.requireNonNull(new File(choreo).listFiles())) {
            if (f.isFile() && f.getName().endsWith(".traj")) {
                factory
                    .cache()
                    .loadTrajectory(f.getName().replaceAll(".traj", ""));
            }
        }

        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Follows a single trajectory sample with PID correction.
     *
     * <p>Called by the Choreo library for each sample point in the trajectory.
     * Combines feedforward velocities from the trajectory with PID feedback
     * corrections based on the current pose error.
     *
     * <p>The resulting chassis speeds are sent to the drivetrain in field-relative
     * mode with a fixed origin (blue alliance perspective).
     *
     * @param sample The desired trajectory sample containing pose and velocities
     */
    public void follow(SwerveSample sample) {
        // get current pose
        Pose2d pose = drivetrain.getPose();
        Pose2d wants = sample.getPose();

        Logger.recordOutput("Auto/CurrentPose", pose);

        // generate speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), wants.getX()),
            sample.vy + yController.calculate(pose.getY(), wants.getY()),
            sample.omega +
                rotController.calculate(
                    pose.getRotation().getRadians(),
                    wants.getRotation().getRadians()
                )
        );

        // field-relative drive
        drivetrain.drive(speeds, Drivetrain.FieldRelativeMode.kFixedOrigin);
    }

    /**
     * Creates a new autonomous routine.
     *
     * <p>Currently returns an empty routine skeleton. Trajectories and commands
     * should be added to this routine to define the autonomous behavior.
     *
     * @return A new {@link AutoRoutine} for autonomous execution
     */
    public AutoRoutine routine() {
        AutoRoutine routine = factory.newRoutine("Main Routine");

        return routine;
    }
}
