package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.ArrayList;
import java.util.Collections;

public class Scheduler {

    private static final ArrayList<Runnable> callbacks = new ArrayList<>();

    static {
        CommandScheduler.getInstance().schedule(new RunCommand(Scheduler::run));
    }

    public static void onPeriodic(Runnable... callbacks) {
        Collections.addAll(Scheduler.callbacks, callbacks);
    }

    private static void run() {
        for (Runnable callback : callbacks) {
            callback.run();
        }
    }
}
