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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.photon.CameraPhoton;
import frc.robot.subsystems.photon.CameraReplay;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.quest.Meta3S;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.quest.QuestReplay;
import frc.robot.util.VisionMeasurement;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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

    /** the questnav used for primary vision things */
    final Quest quest = new Quest(
        switch (Constants.currentMode) {
            case REAL -> new Meta3S();
            case REPLAY -> new QuestReplay();
            case SIM -> new QuestReplay();
        },
        this::addVisionMeasurement
    );

    /** vision measurements collected before robot start */
    final List<VisionMeasurement> calibrators = new ArrayList<>();

    /** photonvision, used to calibrate starting position */
    final Photon photon = new Photon(
        switch (Constants.currentMode) {
            case REAL -> CameraPhoton::new;
            case REPLAY -> CameraReplay::new;
            case SIM -> CameraReplay::new;
        },
        this.calibrators::add
    );

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

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

    @Override
    public void periodic() {
        // Collect updates when disabled; calibrate once enabled.
        // although calibrate() is called many times, it only acts once
        // since we clear the calibrators list at the end of the method.
        if (DriverStation.isDisabled()) photon.update();
        else calibrate();
        quest.update();

        // If the quest has disconnected, fallback on photon for vision
        if (!quest.data().connected && DriverStation.isEnabled()) {
            photon.update();
            this.calibrators.stream().forEach(this::addVisionMeasurement);
            this.calibrators.clear();
        }

        odometryLock.lock(); // Prevents odometry updates while reading data
        try {
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Drive/Gyro", gyroInputs);
            for (var module : modules) {
                module.periodic();
            }
        } finally {
            odometryLock.unlock();
        }

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

            Optional<Rotation2d> questHeading = quest.headingAt(
                sampleTimestamps[i]
            );

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else if (questHeading.isPresent()) {
                // Use the Quest heading if available
                rawGyroRotation = questHeading.get();
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
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(
            discreteSpeeds
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            setpointStates,
            maxSpeedMetersPerSec
        );

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
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
    private ChassisSpeeds getChassisSpeeds() {
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
    public void addVisionMeasurement(VisionMeasurement measurement) {
        poseEstimator.addVisionMeasurement(
            measurement.estimate2(),
            measurement.timestamp(),
            measurement.stdDevs()
        );
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxSpeedMetersPerSec;
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

        // Clear calibrators for next use
        calibrators.clear();
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxSpeedMetersPerSec / driveBaseRadius;
    }
}
