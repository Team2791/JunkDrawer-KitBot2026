package frc.robot.subsystems.drivetrain;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.VisionMeasurement;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.module.ModuleIO;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.AllianceUtil;
import frc.robot.util.IterUtil;
import frc.robot.util.RateLimiter;
import frc.robot.util.Vec2;
import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The swerve drivetrain subsystem.
 *
 * <p>This subsystem handles all robot movement through four independent swerve modules.
 * It manages:
 * <ul>
 *   <li>Module velocity and angle control
 *   <li>Gyroscope-based rotation tracking
 *   <li>Pose estimation using swerve kinematics and vision measurements
 *   <li>Multiple drive control modes (field-relative, robot-relative, alliance-relative)
 *   <li>Dashboard visualization via the Field2d widget and Elastic dashboard
 * </ul>
 *
 * <p>The drivetrain uses a dependency injection pattern to support both real hardware
 * (NavX gyro, Spark motor controllers) and replay/simulation alternatives.
 */
public class Drivetrain extends SubsystemBase {

    /** Front-left swerve module. */
    final ModuleIO frontLeft;

    /** Front-right swerve module. */
    final ModuleIO frontRight;

    /** Rear-left swerve module. */
    final ModuleIO rearLeft;

    /** Rear-right swerve module. */
    final ModuleIO rearRight;

    /** Gyroscope for absolute rotation tracking. */
    final GyroIO gyro;

    /** Pose estimator that combines encoder and gyro data with vision measurements. */
    final SwerveDrivePoseEstimator odometry;

    /** Field visualization widget for SmartDashboard and Elastic dashboard. */
    final Field2d field;

    /** Rate limiter for controlling acceleration on drive inputs. */
    final RateLimiter slew;

    /**
     * Constructs a new Drivetrain subsystem.
     *
     * Creates all four swerve modules, initializes the pose estimator, and sets up
     * the gyroscope. The actual hardware implementations are provided via dependency injection.
     *
     * @param gyro The gyroscope implementation (NavX or GyroReplay)
     * @param moduleFactory A factory function that creates swerve modules based on their configuration
     */
    public Drivetrain(
        GyroIO gyro,
        Function<SwerveConstants.Module, ModuleIO> moduleFactory
    ) {
        this.gyro = gyro;

        // Create all four swerve modules using the factory
        frontLeft = moduleFactory.apply(SwerveConstants.Module.kFrontLeft);
        frontRight = moduleFactory.apply(SwerveConstants.Module.kFrontRight);
        rearLeft = moduleFactory.apply(SwerveConstants.Module.kRearLeft);
        rearRight = moduleFactory.apply(SwerveConstants.Module.kRearRight);

        // Initialize the pose estimator with current module positions
        odometry = new SwerveDrivePoseEstimator(
            SwerveConstants.kKinematics,
            gyro.heading(),
            modulePositions(),
            new Pose2d()
        );

        // Create rate limiter for smooth acceleration control
        slew = new RateLimiter(
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kRotation
        );

        field = new Field2d();

        // Reset the gyro to zero
        this.gyro.reset(new Rotation2d());

        // Configure the Elastic dashboard swerve drive widget
        SmartDashboard.putData("SwerveDrive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            // Display each module's angle and velocity
            IterUtil.zipThen(
                Arrays.stream(modules()),
                Stream.of(
                    "Front Left",
                    "Front Right",
                    "Back Left",
                    "Back Right"
                ),
                (module, label) -> {
                    builder.addDoubleProperty(
                        label + " Angle",
                        () -> module.getState().angle.getRadians(),
                        null
                    );
                    builder.addDoubleProperty(
                        label + " Velocity",
                        () -> module.getState().speedMetersPerSecond,
                        null
                    );
                }
            );

            // Display robot heading (corrected for alliance if applicable)
            builder.addDoubleProperty(
                "Robot Angle",
                () -> {
                    Rotation2d ang = getHeading();
                    Rotation2d fixed = AllianceUtil.autoflip(ang).orElse(ang);
                    return fixed.getRadians();
                },
                null
            );
        });

        // Register with AdvantageKit for automatic logging
        AutoLogOutputManager.addObject(this);
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_RobotDrive,
            FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit
        );
    }

    /**
     * Gets all swerve modules on the robot.
     *
     * @return An array of all modules in order: [frontLeft, frontRight, rearLeft, rearRight]
     */
    public ModuleIO[] modules() {
        return new ModuleIO[] { frontLeft, frontRight, rearLeft, rearRight };
    }

    /**
     * Gets the current position of each swerve module.
     *
     * Used by the odometry system to track robot movement.
     *
     * @return An array of module positions in the same order as modules()
     */
    @AutoLogOutput
    public SwerveModulePosition[] modulePositions() {
        return Arrays.stream(modules())
            .map(ModuleIO::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Gets the current velocity and angle of each swerve module.
     *
     * @return An array of module states in the same order as modules()
     */
    @AutoLogOutput
    public SwerveModuleState[] moduleStates() {
        return Arrays.stream(modules())
            .map(ModuleIO::getState)
            .toArray(SwerveModuleState[]::new);
    }

    /**
     * Gets the overall velocity of the robot's chassis.
     *
     * @return The current chassis speeds (vx, vy, omega)
     */
    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kKinematics.toChassisSpeeds(moduleStates());
    }

    /** @param speeds The desired speeds for the robot to move at */
    private void setDesiredSpeeds(ChassisSpeeds speeds) {
        // Discretize speeds for more accurate control (matches 20ms robot tick)
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, 0.02);

        // Convert chassis speeds to individual module states
        SwerveModuleState[] states =
            SwerveConstants.kKinematics.toSwerveModuleStates(discrete);

        // Ensure all wheels are commanded within maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            SwerveConstants.MaxSpeed.kLinear
        );

        // Log for debugging and analysis
        Logger.recordOutput("Drivetrain/StateSetpoints", states);
        Logger.recordOutput("Drivetrain/SpeedSetpoint", discrete);

        // Command each module to its desired state
        IterUtil.zipThen(
            Arrays.stream(modules()),
            Arrays.stream(states),
            ModuleIO::setDesiredState
        );
    }

    /**
     * Gets the estimated position of the robot on the field.
     *
     * This is updated using encoder/gyro data and enhanced with vision measurements.
     *
     * @return The robot's current pose (x, y, rotation)
     */
    @AutoLogOutput
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets the robot's pose to a specific value.
     *
     * Useful at the start of autonomous or when absolute position knowledge is needed.
     *
     * @param pose The new pose for the robot
     */
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.heading(), modulePositions(), pose);
    }

    /**
     * Gets the robot's current heading (rotation).
     *
     * Equivalent to getPose().getRotation() but slightly more convenient.
     *
     * @return The robot's rotation angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Commands swerve drive with specified speeds and frame of reference.
     *
     * <p>Supports multiple coordinate frames:
     * <ul>
     *   <li>{@code kFixedOrigin}: Field-relative with fixed coordinate system
     *   <li>{@code kAllianceOrigin}: Field-relative with alliance-aware coordinate system (recommended)
     *   <li>{@code kOff}: Robot-relative (no field rotation applied)
     * </ul>
     *
     * @param speeds The desired chassis speeds
     * @param fieldRelative The reference frame for the speeds
     */
    public void drive(ChassisSpeeds speeds, FieldRelativeMode fieldRelative) {
        switch (fieldRelative) {
            case kFixedOrigin -> setDesiredSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading())
            );
            case kAllianceOrigin -> setDesiredSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vxMetersPerSecond * AllianceUtil.sign().orElse(1),
                    speeds.vyMetersPerSecond * AllianceUtil.sign().orElse(1),
                    speeds.omegaRadiansPerSecond,
                    getHeading()
                )
            );
            case kOff -> setDesiredSpeeds(speeds);
        }
    }

    /**
     * Commands field-relative swerve drive with alliance-aware coordinates.
     *
     * This is the default drive mode and accounts for alliance-specific field orientation.
     *
     * @param speeds The desired chassis speeds
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, FieldRelativeMode.kAllianceOrigin);
    }

    /**
     * Commands swerve drive with manual power inputs (normalized to [-1, 1]).
     *
     * @param pow The X and Y power inputs ([-1, 1])
     * @param rotPower The rotational power input ([-1, 1])
     * @param fieldRelative The reference frame for the powers
     */
    public void drive(
        Vec2 pow,
        double rotPower,
        FieldRelativeMode fieldRelative
    ) {
        // Normalize inputs if magnitude exceeds 1.0
        if (pow.mag() > 1) pow = pow.norm();

        // Scale to maximum velocities
        Vec2 vel = pow.mul(SwerveConstants.MaxSpeed.kLinear);
        double rot = rotPower * SwerveConstants.MaxSpeed.kAngular;

        drive(new ChassisSpeeds(vel.x, vel.y, rot), fieldRelative);
    }

    /**
     * Commands field-relative swerve drive with manual power inputs.
     *
     * @param pow The X and Y power inputs ([-1, 1])
     * @param rotPower The rotational power input ([-1, 1])
     */
    public void drive(Vec2 pow, double rotPower) {
        drive(pow, rotPower, FieldRelativeMode.kAllianceOrigin);
    }

    /**
     * Commands swerve drive from an Xbox controller.
     *
     * Applies deadband, rate limiting, and power scaling to controller inputs.
     * Handles the controller coordinate system (x-right, y-down) to robot system conversion.
     *
     * The controller inputs are squared to give more fine control at low speeds while
     * maintaining full responsiveness at high speeds.
     *
     * @param controller The Xbox controller to read input from
     */
    public void drive(CommandXboxController controller) {
        // Get controller inputs with deadband applied
        double xspeed = MathUtil.applyDeadband(
            controller.getLeftX(),
            IOConstants.Controller.kDeadband
        );
        double yspeed = MathUtil.applyDeadband(
            controller.getLeftY(),
            IOConstants.Controller.kDeadband
        );
        double rot = MathUtil.applyDeadband(
            controller.getRightX(),
            IOConstants.Controller.kDeadband
        );

        // Apply rate limiting for smooth acceleration
        RateLimiter.Outputs outputs = slew.calculate(
            new Vec2(xspeed, yspeed),
            rot
        );

        // Square rotation while preserving sign for control response
        double rot2 = Math.pow(outputs.rot(), 2) * Math.signum(outputs.rot());

        // Square velocity while preserving sign
        Vec2 vel2 = outputs.vel().powi(2).mul(outputs.vel().sign());

        /*
         * Controller to WPI coordinate system conversion:
         *
         * Controller coordinates:    WPI coordinates:
         *   +X = right                 +X = forward
         *   +Y = down                  +Y = left
         *   +Rot = clockwise           +Rot = counter-clockwise
         *
         * Transformation needed:
         *   +Xc -> -Yw  (right stick -> left motion negation)
         *   +Yc -> -Xw  (down stick -> forward motion negation)
         *   +Rotc -> -Rotw (negate rotation for ccw-positive)
         */
        drive(new Vec2(-vel2.y, -vel2.x), -rot2);
    }

    /**
     * Resets the gyroscope heading to absolute zero (corrected for alliance if applicable).
     * After calling this, the robot will consider its current facing direction as "aligned" with the field.
     * SAFETY: This method assumes FMS/DS has been connected at least once.
     */
    public void resetGyro() {
        // SAFETY: This method is only called in response to driver input,
        // which means DS has reported alliance by this point.
        gyro.reset(AllianceUtil.unsafe.autoflip(new Rotation2d()));
    }

    /**
     * Incorporates a vision measurement into the pose estimate.
     *
     * Allows the pose estimator to correct odometry drift using camera-based pose detection.
     * The measurement includes position, rotation, timestamp, and measurement covariance.
     *
     * @param measurement The vision measurement to incorporate
     */
    public void addVisionMeasurement(VisionMeasurement measurement) {
        odometry.addVisionMeasurement(
            measurement.estimate2(),
            measurement.timestamp(),
            measurement.stdDevs()
        );
    }

    /**
     * Gets the field visualization widget.
     *
     * This can be used to display the robot position and trajectory on SmartDashboard
     * or Elastic dashboard.
     *
     * @return The Field2d widget
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Periodic update method called every 20ms.
     *
     * <p>Updates all drivetrain components and logs data for analysis:
     * <ul>
     *   <li>Reads gyroscope heading
     *   <li>Updates all swerve modules
     *   <li>Updates pose estimator
     *   <li>Logs data to AdvantageKit
     *   <li>Updates dashboard displays
     * </ul>
     */
    @Override
    public void periodic() {
        // Update sensor readings
        gyro.update();
        Arrays.stream(modules()).forEach(ModuleIO::update);

        // Get robot heading from gyro (use zero if disconnected)
        Rotation2d heading = gyro.data.connected
            ? gyro.heading()
            : new Rotation2d();

        // Update odometry with latest measurements
        try {
            odometry.update(heading, modulePositions());
        } catch (Exception ignored) {}

        // Log module states and chassis speeds
        Logger.recordOutput("Drivetrain/ModuleStates", moduleStates());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());

        // Log active command for debugging
        AdvantageUtil.logActiveCommand(this);

        // Update dashboard field display
        SmartDashboard.putData("Field", field);
    }

    /**
     * Enum defining different coordinate frame options for drive commands.
     */
    public enum FieldRelativeMode {
        /** Robot-centric: no field rotation applied */
        kOff,

        /** Field-relative with fixed coordinate system origin */
        kFixedOrigin,

        /** Field-relative with alliance-aware coordinate system (recommended) */
        kAllianceOrigin,
    }
}
