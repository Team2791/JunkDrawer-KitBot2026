package frc.robot.subsystems.drivetrain.module;

import frc.robot.constants.SwerveConstants;

/**
 * Replay implementation of {@link ModuleIO} for log playback.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all module data
 * is read directly from logged {@link ModuleData}.
 *
 * <p>All methods are no-ops because all replayed data is immutable.
 */
public class ModuleReplay extends ModuleIO {

    /**
     * Constructs a replay module for a specific swerve module position.
     *
     * @param id The module identifier (FL, FR, RL, or RR)
     */
    public ModuleReplay(SwerveConstants.Module id) {
        super(id);
    }

    /**
     * No-op update for replay mode.
     *
     * <p>Module sensor readings are automatically populated from logs by
     * AdvantageKit's replay infrastructure.
     */
    @Override
    public void update() {}

    /**
     * No-op setpoint command for replay mode.
     *
     * <p>Motor commands are not executed during replay since module state
     * is determined entirely by logged data.
     *
     * @param driveVelocity Ignored in replay mode
     * @param turnPosition Ignored in replay mode
     */
    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) {}
}
