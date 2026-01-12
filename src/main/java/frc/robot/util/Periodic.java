package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.ArrayList;
import java.util.Collections;

public class Periodic {

    private static final ArrayList<Runnable> callbacks = new ArrayList<>();

    static {
        CommandScheduler.getInstance().schedule(new RunCommand(Periodic::run));
    }

    public static void schedule(Runnable... callbacks) {
        Collections.addAll(Periodic.callbacks, callbacks);
    }

    private static void run() {
        for (Runnable callback : callbacks) {
            callback.run();
        }
    }
}
