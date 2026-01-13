package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constants for AdvantageKit logging mode selection.
 *
 * <p>This class determines which operating mode the robot code runs in, affecting
 * how data is logged and which hardware/simulation implementations are used.
 */
public class AdvantageConstants {

    /**
     * The current robot operating mode.
     *
     * <p>Automatically detected based on the robot environment:
     * <ul>
     *   <li>{@link AdvantageMode#Real}: Running on actual robot hardware
     *   <li>{@link AdvantageMode#Sim}: Running in simulation (currently not supported)
     * </ul>
     *
     * <p>For replay mode, this must be manually set to {@link AdvantageMode#Replay}
     * in the code before starting.
     */
    public static final AdvantageMode kCurrentMode = RobotBase.isReal()
        ? AdvantageMode.Real
        : AdvantageMode.Sim;

    /**
     * Robot operating modes for AdvantageKit.
     *
     * <ul>
     *   <li>{@code Real}: Running on physical robot hardware with real sensors/actuators
     *   <li>{@code Sim}: Running in simulation (not currently supported)
     *   <li>{@code Replay}: Replaying from previously recorded log files for analysis
     * </ul>
     */
    public enum AdvantageMode {
        Real,
        Sim,
        Replay,
    }
}
