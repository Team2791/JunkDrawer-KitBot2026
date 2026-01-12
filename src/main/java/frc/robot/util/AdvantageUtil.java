package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.AdvantageConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class AdvantageUtil {

    private AdvantageUtil() {}

    public static <T> T match(T real, T replay) {
        return switch (AdvantageConstants.kCurrentMode) {
            case Real -> real;
            case Sim -> throw new IllegalStateException("No simulation this year :(");
            case Replay -> replay;
        };
    }

    public static <T> T match(Supplier<T> real, Supplier<T> replay) {
        return AdvantageUtil.<Supplier<T>>match(real, replay).get();
    }

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
