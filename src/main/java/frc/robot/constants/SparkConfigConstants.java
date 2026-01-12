package frc.robot.constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigConstants {

    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersistMode =
        PersistMode.kPersistParameters;

    public static final class Drivetrain {

        public static final SparkMaxConfig kDrive;
        public static final SparkMaxConfig kTurn;

        static {
            kDrive = new SparkMaxConfig();
            kTurn = new SparkMaxConfig();

            // current limits
            kDrive.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);
            kTurn.smartCurrentLimit((int) MotorConstants.Neo550.kCurrentLimit);

            // position and velocity factors
            kDrive.encoder.positionConversionFactor(
                SwerveConstants.DriveEncoder.kPositionFactor
            );
            kDrive.encoder.velocityConversionFactor(
                SwerveConstants.DriveEncoder.kVelocityFactor
            );
            kTurn.absoluteEncoder.positionConversionFactor(
                SwerveConstants.TurnEncoder.kPositionFactor
            );
            kTurn.absoluteEncoder.velocityConversionFactor(
                SwerveConstants.TurnEncoder.kVelocityFactor
            );

            // voltage compensation
            kDrive.voltageCompensation(MotorConstants.kNominalVoltage);
            kTurn.voltageCompensation(MotorConstants.kNominalVoltage);

            // setup absolute encoder
            kTurn.absoluteEncoder.inverted(
                SwerveConstants.TurnEncoder.kInverted
            );
            kTurn.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

            // turn pid wrapping
            kTurn.closedLoop.positionWrappingEnabled(true);
            kTurn.closedLoop.positionWrappingMinInput(
                ControlConstants.DrivetrainTurn.kMinInput
            );
            kTurn.closedLoop.positionWrappingMaxInput(
                ControlConstants.DrivetrainTurn.kMaxInput
            );

            // pid constants
            kDrive.closedLoop.pid(
                ControlConstants.DrivetrainDrive.kP,
                ControlConstants.DrivetrainDrive.kI,
                ControlConstants.DrivetrainDrive.kD
            );
            kDrive.closedLoop.feedForward.sva(
                ControlConstants.DrivetrainDrive.kS,
                ControlConstants.DrivetrainDrive.kV,
                ControlConstants.DrivetrainDrive.kA
            );
            kDrive.closedLoop.outputRange(
                ControlConstants.DrivetrainDrive.kMin,
                ControlConstants.DrivetrainDrive.kMax
            );

            kTurn.closedLoop.pid(
                ControlConstants.DrivetrainTurn.kP,
                ControlConstants.DrivetrainTurn.kI,
                ControlConstants.DrivetrainTurn.kD
            );
            kTurn.closedLoop.outputRange(
                ControlConstants.DrivetrainTurn.kMinOutput,
                ControlConstants.DrivetrainTurn.kMaxOutput
            );

            // idle mode
            kDrive.idleMode(SwerveConstants.DriveMotor.kIdleMode);
            kTurn.idleMode(SwerveConstants.TurnMotor.kIdleMode);
        }
    }
}
