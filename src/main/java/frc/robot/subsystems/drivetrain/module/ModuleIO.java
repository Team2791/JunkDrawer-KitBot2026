package frc.robot.subsystems.drivetrain.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.MathUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class defining the interface for swerve drive module implementations.
 *
 * <p>Each swerve module independently controls:
 * <ul>
 *   <li>Drive velocity (forward/backward wheel speed)
 *   <li>Turn position (wheel orientation/direction)
 * </ul>
 *
 * <p>This class defines the contract that all swerve module implementations must follow.
 * Concrete implementations (e.g., {@link ModuleSpark}) handle specific hardware configurations
 * (motor types, encoders, controllers).
 *
 * <p>The module tracks state via {@link ModuleData} which includes:
 * <ul>
 *   <li>connection status
 *   <li>position
 *   <li>velocity
 *   <li>voltage
 *   <li>current
 * </ul>
 *
 * <p>The Drivetrain subsystem uses module implementations via dependency injection
 * to support different hardware configurations or simulation.
 */
public abstract class ModuleIO {

    /**
     * Data container for swerve module sensor readings and system state.
     *
     * Tracks both drive and turn motor/encoder states. Automatically logged
     * by AdvantageKit via the @AutoLog annotation, enabling data recording
     * and replay for testing and analysis.
     */
    @AutoLog
    public static class ModuleData {

        /** Drive motor connection status (true if motor responds to commands). */
        public boolean driveConnected = false;
        /** Drive motor output shaft position (integrated encoder ticks). */
        public Distance drivePosition = Meters.of(0);
        /** Drive motor current velocity. */
        public LinearVelocity driveVelocity = MetersPerSecond.of(0);
        /** Drive motor supply voltage. */
        public Voltage driveVoltage = Volts.of(0);
        /** Drive motor current draw. */
        public Current driveCurrent = Amps.of(0);

        /** Turn motor connection status (true if motor responds to commands). */
        public boolean turnConnected = false;
        /** Turn motor output shaft angle (absolute position from encoder). */
        public Angle turnPosition = Radians.of(0);
        /** Turn motor current angular velocity. */
        public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
        /** Turn motor supply voltage. */
        public Voltage turnVoltage = Volts.of(0);
        /** Turn motor current draw. */
        public Current turnCurrent = Amps.of(0);
    }

    /** Current sensor readings for this module. */
    public final ModuleDataAutoLogged data = new ModuleDataAutoLogged();

    /** Unique identifier for this module (FL, FR, RL, or RR). */
    public final SwerveConstants.Module id;

    /**
     * Constructs a module IO instance.
     *
     * @param id Unique identifier for this module's position on the chassis
     */
    protected ModuleIO(SwerveConstants.Module id) {
        this.id = id;
    }

    /**
     * Reads sensor data from motors and encoders, updating {@link ModuleData}.
     *
     * This method should be called periodically (typically in subsystem periodic)
     * to refresh all sensor readings in the module data. Drive position and turn
     * position must be continuously integrated from encoder values.
     */
    public abstract void update();

    /**
     * Sends desired module state commands to the hardware.
     *
     * <p>This method takes a desired swerve module state (velocity and angle) and sends
     * the appropriate commands to the drive and turn motors. The state includes:
     * <ul>
     *   <li>{@code speedMetersPerSecond}: Target drive motor velocity
     *   <li>{@code angle}: Target turn motor direction/orientation
     * </ul>
     *
     * <p>The Drivetrain subsystem calls this method with optimized and cosine-scaled
     * states to minimize wheel slip and ensure smooth motion. This method should:
     * <ol>
     *   <li>Apply any hardware-specific control loop gains
     *   <li>Clamp commands to safe ranges
     *   <li>Send setpoints to motor controllers
     * </ol>
     *
     * @param desired The target state including velocity and wheel angle
     */
    public void setDesiredState(SwerveModuleState desired) {
        // Optimize the state to minimize wheel rotation (always take shorter path)
        desired.optimize(new Rotation2d(data.turnPosition));
        // Scale drive velocity by cosine of angle error to reduce slip during turning
        desired.cosineScale(new Rotation2d(data.turnPosition));

        double commanded = desired.speedMetersPerSecond;
        double turn = MathUtil.normalizeAngle(desired.angle.getRadians());

        // Send the optimized commands to motor controllers
        setStateSetpoint(commanded, turn);

        // Log desired values for debugging and analysis
        Logger.recordOutput(
            "Drivetrain/Module/%d/DesiredSpeed".formatted(id.value),
            commanded
        );
        Logger.recordOutput(
            "Drivetrain/Module/%d/DesiredAngle".formatted(id.value),
            turn
        );
    }

    /**
     * Sends direct velocity and position setpoints to motor controllers.
     *
     * This is the low-level command method called by {@link #setDesiredState(SwerveModuleState)}.
     * Implementations should send these values to the appropriate motor controllers
     * (velocity control for drive, position control for turn).
     *
     * @param driveVelocity Target drive motor velocity in meters per second
     * @param turnPosition Target turn motor position in radians
     */
    public abstract void setStateSetpoint(
        double driveVelocity,
        double turnPosition
    );

    /**
     * Gets the current position of this module (drive distance and turn angle).
     *
     * <p>This position is used by the Drivetrain's SwerveDrivePoseEstimator for
     * tracking robot pose based on wheel movement (odometry). The position includes:
     * <ul>
     *   <li>Drive distance: Total distance wheel has traveled (meters)
     *   <li>Angle: Current orientation of the wheel (radians)
     * </ul>
     *
     * @return Current swerve module position from sensor data
     */
    public final SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            data.drivePosition,
            new Rotation2d(data.turnPosition)
        );
    }

    /**
     * Gets the current velocity state of this module (speed and orientation).
     *
     * @return Current swerve module state (velocity and angle) from sensor data
     */
    public final SwerveModuleState getState() {
        return new SwerveModuleState(
            data.driveVelocity,
            new Rotation2d(data.turnPosition)
        );
    }
}
