package frc.robot.auto;

import choreo.auto.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.generated.ChoreoTraj;
import frc.robot.commands.pathfind.Pathfind;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import java.util.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * AutoSelector provides a three-stage autonomous routine selection system for FRC robots.
 *
 * This class creates a hierarchical selection interface using three dashboard choosers that
 * dynamically update based on previous selections. The first chooser allows selection of
 * starting positions, the second chooser populates with valid transitions from the first
 * selection, and the third chooser populates with valid transitions from the second selection.
 *
 * The selector uses an AutoGraph to determine valid transitions between AutoNodes and
 * automatically loads all available Choreo trajectories into the factory cache for
 * efficient path planning.
 *
 * Key Features:
 * - Dynamic chooser population based on graph transitions
 * - Automatic trajectory caching from ChoreoTraj
 * - Support for canceling autonomous sequences at any stage
 * - Integration with Elastic notifications for error reporting
 * - Alliance-aware pose handling with automatic field flipping
 *
 * Usage:
 * 1. Create an AutoSelector instance with a Drive subsystem
 * 2. Call build() to generate an AutoRoutine based on current selections
 * 3. The routine will execute the selected path with appropriate transitions
 *
 * The selector handles null selections gracefully and provides feedback through
 * Elastic notifications when trajectory transitions are not available.
 *
 * @see AutoFactory
 * @see AutoNode
 * @see AutoGraph
 * @see ChoreoTraj
 */
public class AutoSelector {

    final AutoFactory factory;
    final AutoGraph graph = new AutoGraph();
    final List<AutoNode> current = new ArrayList<>();

    List<LoggedDashboardChooser<AutoNode>> choosers = new ArrayList<>();
    static final int NUM_CHOOSERS = 4;

    /**
     * Constructs an AutoSelector with the given Drive subsystem.
     * @param drive Drive subsystem
     * @see AutoSelector
     */
    public AutoSelector(Drive drive) {
        factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive.follower::follow,
            true,
            drive,
            (traj, starting) -> {
                // this function is called if a trajectory starts or finished
                // the `starting` parameter is true if the trajectory is starting, false if it is ending
                // if we are ending a trajectory, do cleanup...
                if (!starting) {
                    Pose2d[] empty = new Pose2d[0];
                    Logger.recordOutput("Auto/Trajectory", empty);
                    drive.field.getObject("Auto/Trajectory").setPoses(empty);
                    return;
                }

                // otherwise, log+visualize the trajectory
                Pose2d[] trj = traj.getPoses();
                Pose2d[] flipped = AllianceUtil.autoflip(trj).orElse(trj);
                Logger.recordOutput("Auto/Trajectory", flipped);
                drive.field.getObject("Auto/Trajectory").setPoses(flipped);
            }
        );

        for (ChoreoTraj traj : ChoreoTraj.ALL_TRAJECTORIES.values()) {
            factory.cache().loadTrajectory(traj.name());
        }

        for (int i = 0; i < NUM_CHOOSERS; i++) {
            LoggedDashboardChooser<AutoNode> c = chooser(i);
            choosers.add(c);
        }

        LoggedDashboardChooser<AutoNode> s0 = choosers.get(0);
        add(s0, AutoNode.POS1);
        add(s0, AutoNode.POS2);
        add(s0, AutoNode.POS3);
        add(s0, AutoNode.CANCEL);

        s0.onChange(node -> update(0, node));
    }

    /**
     * Helper method to add an AutoNode option to a chooser.
     * @param c The chooser to add the option to
     * @param n The AutoNode to add as an option
     */
    private void add(LoggedDashboardChooser<AutoNode> c, AutoNode n) {
        c.addOption(n.label(), n);
    }

    /**
     * Called when chooser {@code n} changes to {@code change}.
     *
     * <ol>
     *   <li>Trims all selections and choosers after index {@code n}.
     *   <li>Records the new selection at index {@code n}.
     *   <li>Populates the next chooser with valid graph transitions from {@code change},
     *       plus a {@link AutoNode#CANCEL} stop option.
     *   <li>Registers this method recursively as the next chooser's change listener.
     * </ol>
     *
     * @param n      The index of the chooser that changed (0-based)
     * @param change The newly selected {@link AutoNode}
     */
    public void update(int n, AutoNode change) {
        if (n >= NUM_CHOOSERS - 1) return; // if last chooser changed, no next chooser to update

        if (current.size() > n) {
            cutoff(n + 1); // reset subsequent choosers and selections
            current.remove(n); // remove current selection at index n (last index)
        }

        // current.size() currently equal to n. add new selection at index n
        current.add(change);

        // update next chooser based on new selection
        Set<AutoNode> next = graph.transitionsFrom(change);
        LoggedDashboardChooser<AutoNode> s = choosers.get(n + 1);

        for (AutoNode node : next) add(s, node); // populate next chooser with valid transitions from new selection
        add(s, AutoNode.CANCEL); // add cancel/stop option to next chooser
        s.onChange(node -> update(n + 1, node)); // set up next chooser to update after selection
    }

    /**
     * Recursively cuts off choosers and current selections starting from index n.
     * This method is called when a selection is changed in one of the choosers, and it
     * ensures that all subsequent choosers are reset and repopulated based on the new selection.
     *
     * @param n The index of the chooser that has changed and from which to start cutting off subsequent choosers
     */
    private void cutoff(int n) {
        current.subList(n, current.size()).clear();

        while (n < choosers.size()) {
            choosers.get(n).getSendableChooser().close();
            LoggedDashboardChooser<AutoNode> next = chooser(n);
            choosers.set(n, next);
            n++;
        }
    }

    /**
     * Helper method to create a LoggedDashboardChooser for AutoNode selections.
     * @param n The index of the chooser (used for naming and identification)
     * @return A new LoggedDashboardChooser instance
     */
    private LoggedDashboardChooser<AutoNode> chooser(int n) {
        if (n == 0) return new LoggedDashboardChooser<>("Auto/StartPos");
        else return new LoggedDashboardChooser<>("Auto/Task" + n);
    }

    /**
     * Builds an AutoRoutine based on the current selections in the choosers.
     *
     * <p>This method retrieves the selected AutoNodes from the three choosers and constructs a command sequence
     * that follows the selected path through the AutoGraph. It handles null selections and "Cancel" options
     * gracefully, allowing for flexible autonomous routine construction.
     *
     * @param drive The Drive subsystem to use for trajectory following
     * @return An AutoRoutine representing the selected autonomous path, or null if no valid selection was made
     */
    public AutoRoutine build(Drive drive) {
        AutoRoutine routine = factory.newRoutine("Auto");
        if (current.isEmpty()) return routine; // no selection made, return empty routine

        Command seq = current.get(0).onEnter(routine);

        for (int i = 0; i < current.size() - 1; i++) {
            AutoNode node = current.get(i);
            AutoNode next = current.get(i + 1);

            if (next == AutoNode.CANCEL) break; // if next selection is "Cancel", stop building further commands

            ChoreoTraj trj = graph.transition(node, next);
            Pose2d initial = trj.initialPoseBlue();

            if (i == 0) {
                // SAFETY: Dashboard set current auto, FMS/DS is connected.
                // note: will be overridden if cameras get a good read.
                drive.setPose(AllianceUtil.unsafe.autoflip(initial));
            }

            if (node == AutoNode.CANCEL) break; // if any selection is "Cancel", stop building further commands
            seq = Commands.sequence(
                seq,
                node.onEnter(routine),
                new Pathfind.Supplied(drive, () -> {
                    // SAFETY: robot driving, therefore FMS/DS connection OK
                    return AllianceUtil.unsafe.autoflip(initial);
                }),
                trj.asAutoTraj(routine).cmd()
            );
        }

        routine.active().onTrue(seq);
        return routine;
    }
}
