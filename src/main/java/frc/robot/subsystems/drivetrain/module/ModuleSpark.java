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

public class ModuleSpark extends ModuleIO {

    final SparkMax driveMotor;
    final SparkMax turnMotor;

    final RelativeEncoder driveEncoder;
    final SparkAbsoluteEncoder turnEncoder;

    final SparkClosedLoopController driveController;
    final SparkClosedLoopController turnController;

    SwerveModuleState desiredState;

    /**
     * Constructs a new SwerveModule
     * @param id data about the module, from ModuleConstants
     */
    public ModuleSpark(SwerveConstants.Module id) {
        super(id);
        // initialize motors, encoders, etc.
        driveMotor = new SparkMax(id.driveId(), MotorType.kBrushless);
        turnMotor = new SparkMax(id.turnId(), MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();

        desiredState = new SwerveModuleState();

        // apply and burn configs
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

        // register with notifier
        Alerter.getInstance().register(
            "Module%dDrive".formatted(id.value),
            driveMotor
        );
        Alerter.getInstance().register(
            "Module%dTurn".formatted(id.value),
            turnMotor
        );
    }

    @Override
    public void update() {
        this.data.driveConnected = driveMotor.getLastError() == REVLibError.kOk;
        this.data.drivePosition = Meters.of(
            driveEncoder.getPosition() * SwerveConstants.Wheel.kRadius
        );
        this.data.driveVelocity = MetersPerSecond.of(
            driveEncoder.getVelocity() * SwerveConstants.Wheel.kRadius
        );
        this.data.driveVoltage = Volts.of(
            driveMotor.getBusVoltage() * driveMotor.getAppliedOutput()
        );
        this.data.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

        this.data.turnConnected = turnMotor.getLastError() == REVLibError.kOk;
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
     * Set a desired module state
     *
     * @param driveVelocity the velocity in radians per second
     * @param turnPosition  the position in radians
     */
    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) {
        double turnSetpoint = MathUtil.normalizeAngle(
            turnPosition + id.angularOffset()
        );

        turnController.setReference(turnSetpoint, ControlType.kPosition);
        driveController.setReference(
            driveVelocity / SwerveConstants.Wheel.kRadius,
            ControlType.kVelocity
        );
    }
}
