package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.photon.VisionConstants.CameraConfig;
import frc.robot.subsystems.quest.Quest.TriConsumer;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * Photon vision subsystem for camera-based pose estimation.
 *
 * <p>This class manages multiple vision cameras for detecting AprilTags and calculating robot
 * position corrections. Unlike a typical subsystem, this does NOT extend SubsystemBase because it
 * should not be used as a command requirement - it operates independently.
 *
 * <p>The subsystem:
 *
 * <ul>
 *   <li>Manages one or more Photon vision cameras
 *   <li>Updates camera measurements periodically
 *   <li>Reports vision measurements to a listener (typically the drivetrain)
 *   <li>Integrates with the Periodic utility for non-subsystem periodic updates
 * </ul>
 *
 * <p>Vision measurements are used to correct odometry drift from encoder/gyro estimates.
 */
public class Photon {

  /** Callback function invoked when a new vision measurement is available. */
  private final TriConsumer<Pose2d, Double, Matrix<N3, N1>> onMeasurement;

  /** List of all vision cameras managed by this subsystem. */
  final List<CameraIO> cameras;

  /**
   * Constructs a Photon vision subsystem.
   *
   * <p>Creates and manages the specified cameras using the provided factory function. Registers a
   * periodic callback to update the cameras each robot tick.
   *
   * @param onMeasurement Callback to invoke when a measurement is ready
   * @param factory Factory function to create camera implementations
   */
  public Photon(
      TriConsumer<Pose2d, Double, Matrix<N3, N1>> onMeasurement,
      Function<VisionConstants.CameraConfig, CameraIO> factory) {
    this.onMeasurement = onMeasurement;
    this.cameras = Arrays.stream(CameraConfig.values()).map(factory).toList();
  }

  /**
   * Periodic update method called every robot tick.
   *
   * <p>Updates all cameras and reports any new vision measurements to the listener. This allows the
   * drivetrain to incorporate vision data into its pose estimate.
   */
  public void update() {
    // Update all camera measurements
    this.cameras.forEach(CameraIO::update);

    // Process vision measurements from each camera
    for (CameraIO camera : cameras) {
      Logger.processInputs(String.format("Camera/%s", camera.config.name), camera.data);

      // Only report valid measurements
      if (camera.data.timestamp == -1) continue;

      // Send measurement to listener (typically drivetrain)
      onMeasurement.accept(camera.data.pose, camera.data.timestamp, camera.data.stdDevs);
    }
  }
}
