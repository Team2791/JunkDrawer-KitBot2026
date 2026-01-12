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

public abstract class ModuleIO {

    @AutoLog
    public static class ModuleData {

        public boolean driveConnected = false;
        public Distance drivePosition = Meters.of(0);
        public LinearVelocity driveVelocity = MetersPerSecond.of(0);
        public Voltage driveVoltage = Volts.of(0);
        public Current driveCurrent = Amps.of(0);

        public boolean turnConnected = false;
        public Angle turnPosition = Radians.of(0);
        public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
        public Voltage turnVoltage = Volts.of(0);
        public Current turnCurrent = Amps.of(0);
    }

    public final ModuleData data = new ModuleData();
    public final SwerveConstants.Module id;

    protected ModuleIO(SwerveConstants.Module id) {
        this.id = id;
    }

    public abstract void update();

    /** Set the desired state of the module. This includes velocity and position */
    public void setDesiredState(SwerveModuleState desired) {
        desired.optimize(new Rotation2d(data.turnPosition));
        desired.cosineScale(new Rotation2d(data.turnPosition));

        double commanded = desired.speedMetersPerSecond;
        double turn = MathUtil.normalizeAngle(desired.angle.getRadians());

        setStateSetpoint(commanded, turn);

        Logger.recordOutput(
            "Drivetrain/Module/%d/DesiredSpeed".formatted(id.value),
            commanded
        );
        Logger.recordOutput(
            "Drivetrain/Module/%d/DesiredAngle".formatted(id.value),
            turn
        );
    }

    /** Set the desired state of the module. This includes velocity and position */
    public abstract void setStateSetpoint(
        double driveVelocity,
        double turnPosition
    );

    public final SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            data.drivePosition,
            new Rotation2d(data.turnPosition)
        );
    }

    public final SwerveModuleState getState() {
        return new SwerveModuleState(
            data.driveVelocity,
            new Rotation2d(data.turnPosition)
        );
    }
}
