package frc.robot.subsystems.drivetrain.module;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.Alerter;
import frc.robot.util.MathUtil;

/**
 * REV Robotics SparkMax implementation of a swerve drive module.
 *
 * <p>This concrete {@link ModuleIO} implementation uses two SparkMax brushless motors:
 * <ul>
 *   <li>Drive motor: Rotates the wheel to control forward/backward movement
 *   <li>Turn motor: Rotates the module to control wheel direction
 * </ul>
 *
 * <p><strong>Encoder Setup:</strong>
 * <ul>
 *   <li>Drive: RelativeEncoder (integrated into SparkMax) for position and velocity
 *   <li>Turn: SparkAbsoluteEncoder (separate CANcoder) for absolute wheel direction
 * </ul>
 *
 * <p><strong>Control Approach:</strong>
 * <ul>
 *   <li>Drive: Velocity-mode closed-loop control via PID
 *   <li>Turn: Position-mode closed-loop control via PID
 * </ul>
 *
 * <p>The SparkMax motor controllers run their own PID loops on-motor, with gains
 * configured in {@link SparkConfigConstants.Drivetrain}.
 *
 * <p><strong>Hardware monitoring:</strong>
 * <ul>
 *   <li>Both motors registered with Alerter for fault detection
 *   <li>Connection status tracked via SparkMax error codes
 *   <li>Current draw monitored for stall/jam detection
 * </ul>
 */
public class ModuleSpark extends ModuleIO {

    /** Drive motor - rotates the wheel for linear movement. */
    final SparkMax driveMotor;
    /** Turn motor - rotates the module for directional control. */
    final SparkMax turnMotor;

    /** Drive encoder: integrated relative encoder measuring wheel rotations. */
    final RelativeEncoder driveEncoder;
    /** Turn encoder: absolute encoder (CANcoder) for wheel direction. */
    final SparkAbsoluteEncoder turnEncoder;

    /** Closed-loop controller for drive velocity setpoints. */
    final SparkClosedLoopController driveController;
    /** Closed-loop controller for turn position setpoints. */
    final SparkClosedLoopController turnController;

    /** Last desired state received (for reference). */
    SwerveModuleState desiredState;

    /**
     * Constructs a SparkMax-based swerve module.
     *
     * Initializes both motor controllers with their CAN IDs, attaches encoders,
     * applies REV Spark configurations (PID gains, idle mode, limits), and
     * registers motors with the Alerter for monitoring.
     *
     * @param id Module identifier containing CAN IDs and configuration for this module
     */
    public ModuleSpark(SwerveConstants.Module id) {
        super(id);
        // Initialize drive and turn motors using CAN IDs from module config
        driveMotor = new SparkMax(id.driveId(), MotorType.kBrushless);
        turnMotor = new SparkMax(id.turnId(), MotorType.kBrushless);

        // Get encoder references from motors
        // Drive uses relative encoder built into SparkMax
        // Turn uses external absolute encoder (CANcoder) for persistent direction
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        // Get closed-loop controllers for setting velocity/position commands
        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();

        desiredState = new SwerveModuleState();

        // Apply SparkMax configurations with PID gains, output limits, and modes
        // kResetMode: Reset hardware before applying config for clean state
        // kPersistMode: Burn config to flash so it persists through power cycles
        driveMotor.configure(
            SparkConfigConstants.Drivetrain.kDrive,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );
        turnMotor.configure(
            SparkConfigConstants.Drivetrain.kTurn,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        // Register motors with fault detection system for runtime monitoring
        Alerter.getInstance().register(
            "Module%dDrive".formatted(id.value),
            driveMotor
        );
        Alerter.getInstance().register(
            "Module%dTurn".formatted(id.value),
            turnMotor
        );
    }

    /**
     * Updates sensor readings from both motors and encoders.
     *
     * <p>Called periodically by the Drivetrain subsystem to read current state:
     * <ul>
     *   <li>Drive position: Integrated from relative encoder (meters)
     *   <li>Drive velocity: Current wheel speed from encoder (meters/second)
     *   <li>Turn position: Absolute direction from CANcoder (radians) minus module offset
     *   <li>Turn velocity: Current rotation rate from encoder (radians/second)
     *   <li>Connection status: REVLibError check for each motor
     *   <li>Power draw: Bus voltage and applied output scaled to actual current
     * </ul>
     *
     * <p>The turn position is offset-corrected because the absolute encoder's zero
     * point may not align with the module's forward direction.
     */
    @Override
    public void update() {
        // Update drive motor data
        this.data.driveConnected = driveMotor.getLastError() == REVLibError.kOk;
        // Convert encoder rotations to meters using wheel radius
        this.data.drivePosition = Meters.of(
            driveEncoder.getPosition() * SwerveConstants.Wheel.kRadius
        );
        // Convert encoder RPM to linear velocity in meters/second
        this.data.driveVelocity = MetersPerSecond.of(
            driveEncoder.getVelocity() * SwerveConstants.Wheel.kRadius
        );
        // Actual voltage = bus voltage * duty cycle (applied output)
        this.data.driveVoltage = Volts.of(
            driveMotor.getBusVoltage() * driveMotor.getAppliedOutput()
        );
        this.data.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

        // Update turn motor data
        this.data.turnConnected = turnMotor.getLastError() == REVLibError.kOk;
        // Get absolute encoder position and subtract the module's angular offset
        // (offset calibrates encoder zero to desired forward direction)
        this.data.turnPosition = Radians.of(
            turnEncoder.getPosition() - id.angularOffset()
        );
        this.data.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
        this.data.turnVoltage = Volts.of(
            turnMotor.getBusVoltage() * turnMotor.getAppliedOutput()
        );
        this.data.turnCurrent = Amps.of(turnMotor.getOutputCurrent());
    }

    /**
     * Sends velocity and position setpoints to motor controllers.
     *
     * <p><strong>Drive motor:</strong> Velocity setpoint for wheel speed (m/s {@literal →} rotations/minute conversion)
     * <p><strong>Turn motor:</strong> Position setpoint for wheel angle (radians, offset-corrected)
     *
     * <p>The turn position is offset-corrected by adding back the module's {@code angularOffset}
     * so the motor controller receives the actual absolute position to target.
     *
     * @param driveVelocity Target drive wheel speed in meters per second
     * @param turnPosition Target wheel direction in radians
     */
    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) {
        // Offset correction: add back the angular offset so motor controller
        // receives actual absolute position (undoes the offset from update())
        double turnSetpoint = MathUtil.normalizeAngle(
            turnPosition + id.angularOffset()
        );

        // Send turn position setpoint using position control
        turnController.setSetpoint(turnSetpoint, ControlType.kPosition);

        // Convert meters/second to radians/second for wheel angular velocity
        driveController.setSetpoint(
            driveVelocity / SwerveConstants.Wheel.kRadius,
            ControlType.kVelocity
        );
    }
}
