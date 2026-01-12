package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class AdvantageConstants {
    public static final AdvantageMode kCurrentMode = RobotBase.isReal() ? AdvantageMode.Real : AdvantageMode.Sim;

    public enum AdvantageMode {
        Real, Sim, Replay,
    }
}
