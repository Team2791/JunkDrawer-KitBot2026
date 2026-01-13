package frc.robot.subsystems.drivetrain.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public abstract class GyroIO {

    @AutoLog
    public static class GyroData {

        public boolean connected = false;
        public Angle reading = Radians.of(0);
        public Angle zero = Radians.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        public double[] readings = new double[0];

        public Angle heading() {
            return reading.minus(zero);
        }
    }

    /** The current gyro's data, since the last update() call */
    public final GyroDataAutoLogged data = new GyroDataAutoLogged();

    /** Updates this.data with the current gyro data. */
    public abstract void update();

    /** Reset gyro (device) to zero */
    protected abstract void reset();

    /** Reset gyro to the specified angle */
    public void reset(Rotation2d zero) {
        reset();
        data.zero = zero.getMeasure();
    }

    public final Rotation2d heading() {
        return new Rotation2d(data.heading());
    }
}
