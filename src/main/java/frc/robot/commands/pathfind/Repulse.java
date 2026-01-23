package frc.robot.commands.pathfind;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Repulsor;
import frc.robot.util.Vec2;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Faraway pathfinding command.
 *
 * <p>Uses {@link Repulsor} for faraway pathfinding to a target pose. Repulsor works by generating a
 * potential field around obstacles and the goal, then using the laws of physics to "push" the robot
 * away from obstacles and "pull" it toward the goal.
 *
 * <p>This command is used as the first step in {@link Pathfind}: far from the target, use Repulsor
 * to pathfind around obstacles. When close to the target, switch to {@link Nearby} for precision
 * movement.
 */
class Repulse extends Command {

    /** Supplier for the target pose. */
    final Supplier<Pose2d> target;

    /** Repulsor instance for pathfinding calculations. */
    final Repulsor repulsor = new Repulsor();

    final Drive drivetrain;

    /** PID controller for orthogonal movement correction. */
    final PIDController orthoController = new PIDController(
        ControlConstants.Auto.kOrthoP,
        ControlConstants.Auto.kOrthoI,
        ControlConstants.Auto.kOrthoD
    );

    /** PID controller for rotational movement correction. */
    final PIDController rotController = new PIDController(
        ControlConstants.Auto.kTurnP,
        ControlConstants.Auto.kTurnI,
        ControlConstants.Auto.kTurnD
    );

    /** Currently targeted pose, reset every {@link #initialize()} */
    Pose2d currentTarget;

    /**
     * Constructor for Repulse command.
     *
     * @param drivetrain Drivetrain subsystem for motion control
     * @param target Supplier providing the target pose
     */
    Repulse(Drive drivetrain, Supplier<Pose2d> target) {
        this.target = target;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    /**
     * Initializes the Repulse command.
     *
     * <ol>
     *   <li>Gets the current target pose from the supplier.
     *   <li>Sets the Repulsor goal to the target translation.
     *   <li>Logs the planned trajectory for visualization.
     * </ol>
     */
    @Override
    public void initialize() {
        currentTarget = target.get();
        repulsor.setGoal(currentTarget.getTranslation());

        List<Translation2d> traj = repulsor.getTrajectory(
            drivetrain.getPose().getTranslation(),
            currentTarget.getTranslation(),
            20
        );
        Logger.recordOutput(
            "Pathfind/RepulseTrajectory",
            traj.stream().toArray(Translation2d[]::new)
        );
    }

    /**
     * Runs the Repulse pathfinding logic each cycle.
     *
     * <ol>
     *   <li>Gets the next SwerveSample from Repulsor based on current pose and velocity.
     *   <li>Calculates the distance vector to the target.
     *   <li>Computes PID correction power based on distance.
     *   <li>Adds PID correction to Repulsor's suggested velocities.
     *   <li>Drives the drivetrain with the combined velocities and rotational PID output.
     * </ol>
     */
    @Override
    public void execute() {
        SwerveSample sample = repulsor.getCmd(
            drivetrain.getPose(),
            drivetrain.getChassisSpeeds(),
            DriveConstants.maxSpeedMetersPerSec,
            true
        );

        Logger.recordOutput("Pathfind/RepulseTarget", sample.getPose());

        Vec2 pose = new Vec2(drivetrain.getPose());
        Vec2 target = new Vec2(sample.getPose());
        Vec2 dist = target.sub(pose);
        Vec2 pidpower = dist.norm().mul(orthoController.calculate(dist.mag()));
        Vec2 result = new Vec2(sample.vx, sample.vy).add(pidpower);

        drivetrain.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                result.x,
                result.y,
                rotController.calculate(
                    drivetrain.getPose().getRotation().getRadians(),
                    currentTarget.getRotation().getRadians()
                ),
                drivetrain.getPose().getRotation()
            )
        );
    }

    /**
     * Cleans up after the Repulse command ends.
     *
     * <p>Clears the target and trajectory logs from the field visualization.
     */
    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(
            "Pathfind/RepulseTarget",
            new Pose2d(-1, -1, new Rotation2d())
        );
        Logger.recordOutput("Pathfind/RepulseTrajectory", new Translation2d[0]);
    }

    /**
     * Determines if the Repulse command is finished.
     *
     * <p>Finishes when the robot is within the nearby threshold distance of the target. See {@link
     * ControlConstants.Auto#kNearbyThreshold}.
     */
    @Override
    public boolean isFinished() {
        Vec2 pose = new Vec2(drivetrain.getPose());
        Vec2 target = new Vec2(currentTarget);
        Vec2 dist = target.sub(pose);

        return dist.mag() < ControlConstants.Auto.kNearbyThreshold;
    }
}
