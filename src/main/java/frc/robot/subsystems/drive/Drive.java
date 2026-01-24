// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.photon.CameraPhoton;
import frc.robot.subsystems.photon.CameraReplay;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.quest.QuestIO;
import frc.robot.util.VisionMeasurement;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs =
        new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Field2d field = new Field2d();
    private final Alert gyroDisconnectedAlert = new Alert(
        "Disconnected gyro, using kinematics as fallback.",
        AlertType.kError
    );

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        moduleTranslations
    );
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
        };
    private SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            Pose2d.kZero
        );

    /** The Meta Quest 3S, our primary vision tool */
    final Quest quest;

    /** List of measurements to calibrate the starting Pose from */
    final List<VisionMeasurement> calibrators = new ArrayList<>();

    /** PhotonVision, to calibrate the starting pose of the quest */
    final Photon photon = new Photon(
        this.calibrators::add,
        switch (Constants.currentMode) {
            case REAL -> data -> new CameraPhoton(data);
            default -> data -> new CameraReplay(data);
        }
    );

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO,
        QuestIO questIO
    ) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        quest = new Quest(questIO, this::addVisionMeasurement);

        // Usage reporting for swerve template
        HAL.report(
            tResourceType.kResourceType_RobotDrive,
            tInstances.kRobotDriveSwerve_AdvantageKit
        );

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state ->
                Logger.recordOutput("Drive/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(voltage.in(Volts)),
                null,
                this
            )
        );
    }

    /**
     * Calibrate the initial pose of the robot
     *
     * Photon/AprilTag measurements are recorded and collected while the robot is disabled.
     * Once enabled, we average the latest few measurements and set our starting pose to that.
     */
    private void calibrate() {
        if (calibrators.isEmpty()) return;

        // Average all calibrator measurements
        double x = 0;
        double y = 0;
        double sin = 0;
        double cos = 0;
        int n = 0;

        for (VisionMeasurement vm : calibrators) {
            double age = Timer.getFPGATimestamp() - vm.timestamp();
            if (age > 15) continue; // Ignore old measurements

            Pose2d est = vm.estimate2();

            x += est.getX();
            y += est.getY();
            sin += est.getRotation().getSin();
            cos += est.getRotation().getCos();
            n++;
        }

        x /= n;
        y /= n;
        sin /= n;
        cos /= n;

        Pose2d avg = new Pose2d(x, y, new Rotation2d(cos, sin));

        // Reset odometry and gyro to the averaged pose
        this.setPose(avg);

        // Clear callibrators for next use
        calibrators.clear();
    }

    @Override
    public void periodic() {
        // Collect updates when disabled; calibrate once enabled.
        // although calibrate() is called many times, it only acts once
        // since we clear the calibrators list at the end of the method.
        if (DriverStation.isDisabled()) photon.update();
        else calibrate();

        photon.update();

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput(
                "SwerveStates/Setpoints",
                new SwerveModuleState[] {}
            );
            Logger.recordOutput(
                "SwerveStates/SetpointsOptimized",
                new SwerveModuleState[] {}
            );
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions =
                new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] =
                    modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters -
                        lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle
                );
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(
                    new Rotation2d(twist.dtheta)
                );
            }

            // Apply update
            poseEstimator.updateWithTime(
                sampleTimestamps[i],
                rawGyroRotation,
                modulePositions
            );
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(
            !gyroInputs.connected && Constants.currentMode != Mode.SIM
        );
    }

    /**
     * Run the drivetrain with robot-relative velocities.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Limit speeds to max
        Vector<N2> velocity = VecBuilder.fill(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond
        );

        // Limit to maximum linear speed, if necessary
        // Can't limit components: <100% vx, 100%vy> = sqrt(2)*100% total speed
        if (velocity.norm() > maxSpeedMetersPerSec) {
            velocity = velocity.unit().times(maxSpeedMetersPerSec);
            speeds.vxMetersPerSecond = velocity.get(0);
            speeds.vyMetersPerSecond = velocity.get(1);
        }

        // Limit max angular speed
        if (speeds.omegaRadiansPerSecond > getMaxAngularSpeedRadPerSec()) {
            speeds.omegaRadiansPerSecond = getMaxAngularSpeedRadPerSec();
        }

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(
            discreteSpeeds
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            setpointStates,
            getMaxLinearSpeedMetersPerSec()
        );

        // Log unoptimized setpoints
        Logger.recordOutput("Drivetrain/StateSetpoints", setpointStates);
        Logger.recordOutput("Drivetrain/SpeedSetpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput(
            "Drivetrain/StateSetpointsOptimized",
            setpointStates
        );

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
    }

    /**
     * Commands swerve drive, field-relative with specified speeds.
     *
     * <p>Use {@link #runVelocity(ChassisSpeeds)} for robot-relative driving.
     *
     * @param speeds The desired chassis speeds
     */
    public void drive(ChassisSpeeds speeds) {
        runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation())
        );
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(
            rawGyroRotation,
            getModulePositions(),
            pose
        );
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(VisionMeasurement vm) {
        poseEstimator.addVisionMeasurement(
            vm.estimate2(),
            vm.timestamp(),
            vm.stdDevs()
        );
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxSpeedMetersPerSec / driveBaseRadius;
    }

    /** Get the field visualization object. */
    public Field2d getField() {
        return field;
    }
}
