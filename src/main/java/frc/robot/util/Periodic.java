package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.ArrayList;
import java.util.Collections;

/**
 * A scheduler for executing periodic callbacks outside the command system.
 *
 * This class provides a mechanism for executing arbitrary code on every robot tick without
 * requiring it to be wrapped in a Command or part of a Subsystem. Callbacks registered here
 * will be executed repeatedly by the command scheduler.
 *
 * This is useful for one-off periodic logic that doesn't fit neatly into the command-based
 * paradigm, such as updating sensors, logging data, or managing state machines.
 *
 * The scheduler is initialized statically, so it starts automatically when the class is first loaded.
 */
public class Periodic {

    /** List of callbacks to execute each periodic cycle. */
    private static final ArrayList<Runnable> callbacks = new ArrayList<>();

    /**
     * Static initializer - registers a periodic command with the CommandScheduler
     * that will execute all registered callbacks.
     */
    static {
        CommandScheduler.getInstance().schedule(new RunCommand(Periodic::run));
    }

    /**
     * Registers one or more callbacks to be executed every robot tick.
     *
     * Callbacks are executed in the order they are provided.
     *
     * @param callbacks The callback functions to register for periodic execution
     */
    public static void schedule(Runnable... callbacks) {
        Collections.addAll(Periodic.callbacks, callbacks);
    }

    /**
     * Executes all registered callbacks in sequence.
     *
     * This method is called by the CommandScheduler every robot tick.
     * If a callback throws an exception, execution continues with the next callback.
     */
    private static void run() {
        for (Runnable callback : callbacks) {
            callback.run();
        }
    }
}
