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
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.module.ModuleIO;
import frc.robot.subsystems.photon.CameraIO;
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

public class Drivetrain extends SubsystemBase {

    final ModuleIO frontLeft;
    final ModuleIO frontRight;
    final ModuleIO rearLeft;
    final ModuleIO rearRight;

    final GyroIO gyro;
    final SwerveDrivePoseEstimator odometry;
    final Field2d field;
    final RateLimiter slew;

    public Drivetrain(
        GyroIO gyro,
        Function<SwerveConstants.Module, ModuleIO> moduleFactory
    ) {
        this.gyro = gyro;

        frontLeft = moduleFactory.apply(SwerveConstants.Module.kFrontLeft);
        frontRight = moduleFactory.apply(SwerveConstants.Module.kFrontRight);
        rearLeft = moduleFactory.apply(SwerveConstants.Module.kRearLeft);
        rearRight = moduleFactory.apply(SwerveConstants.Module.kRearRight);

        odometry = new SwerveDrivePoseEstimator(
            SwerveConstants.kKinematics,
            gyro.heading(),
            modulePositions(),
            new Pose2d()
        );

        slew = new RateLimiter(
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kRotation
        );

        field = new Field2d();

        this.gyro.reset(new Rotation2d());

        // Elastic SwerveDrive widget
        SmartDashboard.putData("SwerveDrive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

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

        // template code stuff
        AutoLogOutputManager.addObject(this);
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_RobotDrive,
            FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit
        );
    }

    /**
     * @return A list of all swerve modules on the robot. frontLeft, frontRight, rearLeft, rearRight in that order.
     */
    public ModuleIO[] modules() {
        return new ModuleIO[] { frontLeft, frontRight, rearLeft, rearRight };
    }

    /**
     * @return A list of all swerve module positions on the robot. In the same order as {@link #modules()}.
     */
    @AutoLogOutput
    public SwerveModulePosition[] modulePositions() {
        return Arrays.stream(modules())
            .map(ModuleIO::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    /**
     * @return A list of all swerve module states on the robot. In the same order as {@link #modules()}.
     */
    @AutoLogOutput
    public SwerveModuleState[] moduleStates() {
        return Arrays.stream(modules())
            .map(ModuleIO::getState)
            .toArray(SwerveModuleState[]::new);
    }

    /** @return The speeds of the entire chassis */
    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kKinematics.toChassisSpeeds(moduleStates());
    }

    /** @param speeds The desired speeds for the robot to move at */
    private void setDesiredSpeeds(ChassisSpeeds speeds) {
        // according to delphi, this should remove some skew
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states =
            SwerveConstants.kKinematics.toSwerveModuleStates(discrete);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            SwerveConstants.MaxSpeed.kLinear
        );

        Logger.recordOutput("Drivetrain/StateSetpoints", states);
        Logger.recordOutput("Drivetrain/SpeedSetpoint", discrete);

        IterUtil.zipThen(
            Arrays.stream(modules()),
            Arrays.stream(states),
            ModuleIO::setDesiredState
        );
    }

    /** @return The estimated pose of the robot */
    @AutoLogOutput
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /** @param pose The pose to reset the robot to */
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.heading(), modulePositions(), pose);
    }

    /** @return Equivalent to `getPose().getRotation()` */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Swerve drive control
     *
     * @param speeds        The desired speeds for the robot to move at.
     * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
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
     * Field-relative swerve drive control
     *
     * @param speeds The desired speeds for the robot to move at.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, FieldRelativeMode.kAllianceOrigin);
    }

    /**
     * Manual swerve drive control
     *
     * @param xPower        The desired x-direction power. +X is forward, must be [-1, 1]
     * @param yPower        The desired y-direction power. +Y is left, must be [-1, 1]
     * @param rotPower      The desired rotational power. +R is ccw, must be [-1, 1]
     * @param fieldRelative Whether the speeds are field-relative or robot-relative. Defaults to true.
     */
    public void drive(
        Vec2 pow,
        double rotPower,
        FieldRelativeMode fieldRelative
    ) {
        if (pow.mag() > 1) pow = pow.norm();

        Vec2 vel = pow.mul(SwerveConstants.MaxSpeed.kLinear);
        double rot = rotPower * SwerveConstants.MaxSpeed.kAngular;

        drive(new ChassisSpeeds(vel.x, vel.y, rot), fieldRelative);
    }

    /**
     * Field-relative manual swerve drive control.
     *
     * @param xPower   The desired x-direction power. +X is forward, must be [-1, 1]
     * @param yPower   The desired y-direction power. +Y is left, must be [-1, 1]
     * @param rotPower The desired rotational power. +R is ccw, must be [-1, 1]
     */
    public void drive(Vec2 pow, double rotPower) {
        drive(pow, rotPower, FieldRelativeMode.kAllianceOrigin);
    }

    /**
     * Controller-based swerve drive control
     *
     * @param controller The controller to get input from.
     */
    public void drive(CommandXboxController controller) {
        // [-1..1] inputs w/ deadband

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

        // do a rate limit
        RateLimiter.Outputs outputs = slew.calculate(
            new Vec2(xspeed, yspeed),
            rot
        );

        // square rotation keeping sign (squaring makes sign positive)
        double rot2 = Math.pow(outputs.rot(), 2) * Math.signum(outputs.rot());

        // square velocity keeping sign
        Vec2 vel2 = outputs.vel().powi(2).mul(outputs.vel().sign());

        /*
         * Time to explain some wpilib strangeness
         *
         * xvel, given from the controller, *should* be interpreted as the left-right speed of the robot
         * yvel, given from the controller, *should* be interpreted as the forward-backward speed of the robot
         * however, the WPI coordinate system is such that +Xw is forward, and +Yw is left (using w for WPI)
         * and the controller coordinate system is such that +Xc is right, and +Yc is down (using c for controller)
         * so, we need to mutate x and y, so that +Xc becomes -Yw and +Yc becomes -Xw
         * also, WPIs rotation is ccw-positive and the controller is cw-positive, so we need to negate the rotation
         */
        drive(new Vec2(-vel2.y, -vel2.x), -rot2);
    }

    /** Reset the gyro */
    public void resetGyro() {
        Rotation2d abs0 = new Rotation2d();
        gyro.reset(AllianceUtil.autoflip(abs0).orElse(abs0));
    }

    /**
     * Add a vision measurement
     * @param measurement The vision measurement to add.
     */
    public void addVisionMeasurement(CameraIO.VisionMeasurement measurement) {
        odometry.addVisionMeasurement(
            measurement.estimate2(),
            measurement.timestamp(),
            measurement.stdDevs()
        );
    }

    /**
     * Get Field widget
     */
    public Field2d getField() {
        return field;
    }

    @Override
    public void periodic() {
        // update gyro data
        gyro.update();

        // update all modules
        Arrays.stream(modules()).forEach(ModuleIO::update);

        // get heading, use robot-centric as fallback
        // TODO: incorrect with questnav.
        Rotation2d heading;

        if (gyro.data.connected) heading = gyro.heading();
        else heading = new Rotation2d();

        // update odometry
        try {
            odometry.update(heading, modulePositions());
        } catch (Exception ignored) {}

        // log to akit
        IterUtil.enumerateThen(Arrays.stream(modules()), (idx, module) -> {
            final String path = "Drivetrain/SwerveModule/" + module.id.value;
            // Logger.processInputs(path, module.data);
        });

        Logger.recordOutput("Drivetrain/ModuleStates", moduleStates());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());
        // Logger.processInputs("Drivetrain/Gyro", gyro.data);

        AdvantageUtil.logActiveCommand(this);

        SmartDashboard.putData("Field", field);
    }

    public enum FieldRelativeMode {
        /// i.e. robot centric
        kOff,
        /// relative to fixed field origin, (0, 0) in the library
        kFixedOrigin,
        /// relative to alliance-specific origin, as reported by DS
        kAllianceOrigin,
    }
}
