package frc.robot.subsystems.drivetrain.gyro;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.ControlConstants.kGyroFactor;

import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.IOConstants;
import frc.robot.util.Alerter;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class NavX extends GyroIO {

    final AHRS gyro;

    final Notifier thread = new Notifier(this::read);
    final Queue<Double> readings = new ArrayBlockingQueue<>(20);
    final ReentrantLock lock = new ReentrantLock();
    final double rate = 100;

    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (byte) rate);
        this.gyro.enableLogging(true);

        Alerter.getInstance().register(this.gyro);
        thread.setName("GyroSensor");
        new Thread(() -> thread.startPeriodic(1.0 / rate));
    }

    void read() {
        if (!lock.tryLock()) return; // if we can't get the lock, don't read the gyro
        this.readings.add(measure().in(Radians));
        lock.unlock(); // release the lock
    }

    Angle measure() {
        return Degrees.of(gyro.getAngle() * kGyroFactor);
    }

    AngularVelocity rate() {
        return DegreesPerSecond.of(gyro.getRate() * kGyroFactor);
    }

    double[] collect() {
        return readings.stream().mapToDouble(Double::doubleValue).toArray();
    }

    @Override
    public void update() {
        lock.lock(); // prevent threaded writes to data during our reads.

        data.connected = gyro.isConnected();
        data.reading = this.measure();
        data.velocity = this.rate();
        data.readings = this.collect();

        readings.clear();

        lock.unlock(); // release the lock
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
