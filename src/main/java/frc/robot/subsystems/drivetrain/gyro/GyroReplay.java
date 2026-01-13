package frc.robot.subsystems.drivetrain.gyro;

/**
 * Replay implementation of {@link GyroIO} for log playback.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all gyro data is
 * read directly from the logged {@link GyroIO.GyroData} values.
 *
 * <p>Both {@link #reset()} and {@link #update()} are no-ops because:
 * <ul>
 *   <li>Reset commands are ignored (replayed data is immutable)
 *   <li>Updates are unnecessary (logged data is automatically populated by AdvantageKit)
 * </ul>
 */
public class GyroReplay extends GyroIO {

    /**
     * No-op reset for replay mode.
     *
     * <p>Gyro resets are not executed during replay since the gyro state
     * is determined entirely by logged data.
     */
    @Override
    public void reset() {}

    /**
     * No-op update for replay mode.
     *
     * <p>Sensor readings are automatically populated from logs by AdvantageKit's
     * replay infrastructure, so no manual updates are needed.
     */
    @Override
    public void update() {}
}
