package frc.robot.subsystems.drivetrain.gyro;

/**
 * Replay implementation of {@link GyroIO} for log playback.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all gyro data is
 * read directly from the logged {@link GyroIO.GyroData} values.
 *
 * <p>All methods are no-ops because all replayed data is immutable.
 */
public class GyroReplay extends GyroIO {

    @Override
    public void reset() {}

    @Override
    public void update() {}
}
