package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Vision measurement data structure.
 *
 * <p>Bundles a robot pose estimate with uncertainty (standard deviations)
 * and timestamp for fusion with odometry.
 *
 * @param estimate 3D robot pose from vision
 * @param stdDevs Standard deviations [x, y, theta] for pose estimator weighting
 * @param timestamp FPGA timestamp when image was captured (seconds)
 */
public record VisionMeasurement(
    Pose3d estimate,
    Matrix<N3, N1> stdDevs,
    double timestamp
) {
    /**
     * Convert 3D pose to 2D for odometry fusion.
     *
     * @return 2D robot pose (ignoring Z-axis)
     */
    public Pose2d estimate2() {
        return estimate.toPose2d();
    }
}
