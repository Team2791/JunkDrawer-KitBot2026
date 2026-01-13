package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Main entry point for the FRC robot application.
 *
 * This class contains the main method that starts the robot runtime. It should not be instantiated
 * and serves solely as an entry point for the Java Virtual Machine.
 */
public final class Main {

    /** Private constructor to prevent instantiation of this utility class. */
    private Main() {}

    /**
     * Main method - entry point for the robot application.
     *
     * Starts the WPILib robot framework with an instance of the {@link Robot} class.
     * This method is called by the FRC framework when the robot boots up.
     *
     * @param args Command-line arguments (typically empty in FRC environment)
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
