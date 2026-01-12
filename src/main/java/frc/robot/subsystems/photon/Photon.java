package frc.robot.subsystems.photon;

import frc.robot.constants.VisionConstants;
import frc.robot.util.Scheduler;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Photon camera subsystem, owns cameras and handles pose estimation.
 * Does not extend SubsystemBase because this should not be used as a command requirement!
 */
public class Photon {

    final Consumer<CameraIO.VisionMeasurement> onMeasurement;

    final List<CameraIO> cameras;

    public Photon(
        Consumer<CameraIO.VisionMeasurement> onMeasurement,
        Function<VisionConstants.CameraConfig, CameraIO> cameraFactory,
        VisionConstants.CameraConfig... cameras
    ) {
        this.onMeasurement = onMeasurement;
        this.cameras = Arrays.stream(cameras).map(cameraFactory).toList();

        Scheduler.onPeriodic(this::periodic);
    }

    public void periodic() {
        // update cameras
        this.cameras.forEach(CameraIO::update);

        // add to logger
        //        Logger.processInputs("Photon/Front", front.data);
        //        Logger.processInputs("Photon/Rear", rear.data);

        // make vision odometry measurements
        for (CameraIO camera : cameras) {
            CameraIO.VisionMeasurement measurement = camera.data.measurement;
            if (measurement == null) continue;
            onMeasurement.accept(measurement);
        }
    }
}
