package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.AdvantageConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class providing convenient methods for working with AdvantageKit and mode-dependent code.
 *
 * <p>This class is never instantiated and provides static methods for:
 * <ul>
 *   <li>Selecting implementations based on the current robot mode (real vs replay)
 *   <li>Logging active commands for debugging and analysis
 * </ul>
 *
 * <p>All methods here should use {@link AdvantageConstants#kCurrentMode} to determine the runtime environment.
 */
public final class AdvantageUtil {

    /** Private constructor to prevent instantiation. */
    private AdvantageUtil() {}

    /**
     * Selects between two implementations based on the current robot mode.
     *
     * <p>Used to choose between real robot implementations and replay/simulation alternatives.
     * For example, this could select between a hardware-based gyro and a logged gyro replay.
     *
     * @param real The implementation to use when running on a real robot
     * @param replay The implementation to use when replaying from logs
     * @return The selected implementation based on the current mode
     * @param <T> The type of the implementation objects
     * @throws IllegalStateException if the mode is Sim (which is not supported)
     */
    public static <T> T match(T real, T replay) {
        return switch (AdvantageConstants.kCurrentMode) {
            case Real -> real;
            case Sim -> throw new IllegalStateException(
                "No simulation this year :("
            );
            case Replay -> replay;
        };
    }

    /**
     * Selects between two implementations provided as suppliers based on the current robot mode.
     *
     * <p>This version is useful when the implementations are expensive to create and should only be
     * instantiated when selected. The appropriate supplier is called to create the instance.
     *
     * @param real A supplier that creates the real robot implementation
     * @param replay A supplier that creates the replay implementation
     * @return The selected implementation instance based on the current mode
     * @param <T> The type of the implementation objects
     */
    public static <T> T match(Supplier<T> real, Supplier<T> replay) {
        return AdvantageUtil.<Supplier<T>>match(real, replay).get();
    }

    /**
     * Logs the currently active command for a given subsystem.
     *
     * <p>This is useful for debugging to understand which command is controlling a subsystem
     * at any given time. The active command name is logged to AdvantageKit under the subsystem's
     * "ActiveCommand" property.
     *
     * @param self The subsystem to log the active command for
     */
    public static void logActiveCommand(Subsystem self) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Command command = scheduler.requiring(self);

        if (command != null) {
            Logger.recordOutput(
                self.getName() + "/ActiveCommand",
                command.getName()
            );
        } else {
            Logger.recordOutput(self.getName() + "/ActiveCommand", "None");
        }
    }
}
