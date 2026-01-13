package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

/**
 * A simple command wrapper that executes arbitrary functions.
 *
 * <p>This class exists because most commands are simply thin wrappers around functions
 * that already exist elsewhere in the codebase. FunctionWrapper allows you to turn
 * any Runnable into a Command without creating a new class.
 *
 * <p>Provides flexibility to:
 * <ul>
 *   <li>Run a function on command start
 *   <li>Optionally run another function on command end
 *   <li>Specify when the command should finish
 *   <li>Define subsystem requirements
 * </ul>
 */
public class FunctionWrapper extends Command {

    /** The function to execute when the command initializes. */
    final Runnable function;

    /** Condition that determines when the command should finish. */
    final BooleanSupplier finished;

    /** Optional function to execute when the command ends. */
    final Runnable callback;

    /**
     * Creates a command that executes a function and finishes immediately.
     *
     * The function runs once at the start, then the command ends.
     * No end callback is executed.
     *
     * @param function The function to execute at command start
     * @param requirements Subsystems that this command requires
     */
    public FunctionWrapper(Runnable function, Subsystem... requirements) {
        this(function, () -> true, requirements);
    }

    /**
     * Creates a command that executes a function with custom finish condition.
     *
     * The function runs at start, and the command finishes when the condition becomes true.
     * No end callback is executed.
     *
     * @param function The function to execute at command start
     * @param finished Condition that determines when to finish the command
     * @param requirements Subsystems that this command requires
     */
    public FunctionWrapper(
        Runnable function,
        BooleanSupplier finished,
        Subsystem... requirements
    ) {
        this(function, finished, () -> {}, requirements);
    }

    /**
     * Creates a command that executes a function and runs a callback on end.
     *
     * The function runs at start and finishes immediately, then the callback runs.
     *
     * @param function The function to execute at command start
     * @param callback The function to execute when the command ends
     * @param requirements Subsystems that this command requires
     */
    public FunctionWrapper(
        Runnable function,
        Runnable callback,
        Subsystem... requirements
    ) {
        this(function, () -> true, callback, requirements);
    }

    /**
     * Creates a command with all customization options.
     *
     * @param function The function to execute at command start
     * @param finished Condition that determines when to finish the command
     * @param callback The function to execute when the command ends (runs regardless of interruption)
     * @param requirements Subsystems that this command requires
     */
    public FunctionWrapper(
        Runnable function,
        BooleanSupplier finished,
        Runnable callback,
        Subsystem... requirements
    ) {
        this.function = function;
        this.finished = finished;
        this.callback = callback;
        addRequirements(requirements);
    }

    /**
     * Initializes the command by executing the wrapped function.
     */
    @Override
    public void initialize() {
        function.run();
    }

    /**
     * Cleans up the command by executing the callback function.
     *
     * @param interrupted Whether the command was interrupted (vs finished normally)
     */
    @Override
    public void end(boolean interrupted) {
        callback.run();
    }

    /**
     * Checks if the command should finish.
     *
     * @return true if the command should end, false to continue running
     */
    @Override
    public boolean isFinished() {
        return finished.getAsBoolean();
    }
}
