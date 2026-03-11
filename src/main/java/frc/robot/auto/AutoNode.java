package frc.robot.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.generated.ChoreoTraj;

/**
 * Enumeration representing different autonomous routine nodes/states for the robot.
 * Each node defines a specific action or sequence of actions that the robot should
 * perform during autonomous mode, such as shooting, intaking, moving to positions,
 * or climbing.
 */
public enum AutoNode {
    POS1,
    POS2,
    POS3,
    DEPOT,
    OUTPOST,
    CLIMB,
    CENTER,
    CANCEL;

    /**
     * Returns a human-readable label for this autonomous node.
     * Used for display purposes in dashboards or selection interfaces.
     *
     * @return A descriptive string label for this node
     */
    String label() {
        return switch (this) {
            case POS1 -> "Position 1";
            case POS2 -> "Position 2 and Shoot";
            case POS3 -> "Position 3 and Shoot";
            case DEPOT -> "Depot, Intake, and Shoot";
            case OUTPOST -> "Outpost, Intake, and Shoot";
            case CLIMB -> "Climb";
            case CENTER -> "Center and Collection";
            case CANCEL -> "Stop/Cancel";
        };
    }

    /**
     * Defines the command sequence that should be executed when entering this autonomous node.
     * Creates and returns the appropriate command based on the node type, which may include
     * shooting sequences, intake operations, trajectory following, aiming, and climbing.
     *
     * @param routine The autonomous routine context that provides access to trajectory commands
     * @return The command to execute when entering this node, or null if no action is required
     */
    Command onEnter(AutoRoutine routine) {
        return switch (this) {
            case POS3, POS2 -> Commands.sequence(
                // TODO: shoot all balls
                new InstantCommand()
            );
            case DEPOT -> Commands.sequence(
                // TODO: start intaking while following next trj
                new InstantCommand(),
                ChoreoTraj.seq_depot_intake.asAutoTraj(routine).cmd(),
                // TODO: stop intake
                new InstantCommand(),
                // TODO: aim and shoot all balls
                new InstantCommand()
            );
            case OUTPOST -> Commands.sequence(
                // wait for human player to load balls
                new WaitCommand(3),
                // TODO: aim and shoot all balls
                new InstantCommand()
            );
            case CLIMB -> Commands.sequence(
                // TODO: climb
                new InstantCommand()
            );
            case CENTER -> Commands.sequence(
                // TODO: start intaking while following next trj
                new InstantCommand(),
                ChoreoTraj.seq_center_intake.asAutoTraj(routine).cmd(),
                // TODO: stop intake
                new InstantCommand()
            );
            case POS1, CANCEL -> Commands.none();
        };
    }
}
