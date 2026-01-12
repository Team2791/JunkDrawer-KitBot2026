package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

/** 99% of commands are just functions that already exist, change my mind. */
public class FunctionWrapper extends Command {
    final Runnable function;
    final BooleanSupplier finished;
    final Runnable callback;

    /**
     * Functional Command Wrapper. Will always finish instantly
     *
     * @param function     what to run when the command starts
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable function, Subsystem... requirements) {
        this(function, () -> true, requirements);
    }

    /**
     * Functional Command Wrapper
     *
     * @param function     what to run when the command starts
     * @param finished     whether the command is finished
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable function, BooleanSupplier finished, Subsystem... requirements) {
        this(function, finished, () -> { }, requirements);
    }

    /**
     * Functional Command Wrapper. Will always finish instantly
     *
     * @param callback     what to run when the command ends
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable function, Runnable callback, Subsystem... requirements) {
        this(function, () -> true, callback, requirements);
    }

    /**
     * Functional Command Wrapper
     *
     * @param function     what to run when the command starts
     * @param finished     whether the command is finished
     * @param callback     what to run when the command ends
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable function, BooleanSupplier finished, Runnable callback, Subsystem... requirements) {
        this.function = function;
        this.finished = finished;
        this.callback = callback;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        function.run();
    }

    @Override
    public void end(boolean interrupted) {
        callback.run();
    }

    @Override
    public boolean isFinished() {
        return finished.getAsBoolean();
    }
}
