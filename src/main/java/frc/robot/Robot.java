package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.BuildConstants;
import frc.robot.util.Elastic;
import java.util.Date;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The main robot class extending {@link LoggedRobot}.
 *
 * <p>This class handles:
 * <ul>
 *   <li>Initialization and configuration of the AdvantageKit data logging system
 *   <li>Management of the robot lifecycle (periodic methods for different game states)
 *   <li>Thread scheduling and performance optimization
 *   <li>Web server setup for Elastic dashboard
 * </ul>
 *
 * <p>The robot uses AdvantageKit for advanced data logging and replay capabilities across
 * different modes: Real (on-robot), Sim (simulation), and Replay (from logs).
 */
public class Robot extends LoggedRobot {

    /** Container for all robot subsystems and commands. Initialized on first driver station connection. */
    RobotContainer container;

    /** The autonomous command scheduled during autonomous mode. */
    Command autoCommand;

    /**
     * Robot constructor - initializes logging infrastructure.
     *
     * <p>This constructor runs when the robot code starts and sets up:
     * <ul>
     *   <li>AdvantageKit logger with build metadata (git SHA, branch, commit date)
     *   <li>Data receivers for different robot modes (real robot, simulation, replay)
     *   <li>Hardware logging via URCL for REV hardware monitoring
     *   <li>Web server for the Elastic dashboard
     * </ul>
     *
     * <p>The initialization is mode-aware:
     * <ul>
     *   <li>Real mode: Logs to USB drive and NetworkTables
     *   <li>Sim mode: Logs only to NetworkTables
     *   <li>Replay mode: Replays from a log file and saves simulation results
     * </ul>
     */
    public Robot() {
        // Setup logger constants with build information
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        // Record git repository status
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitStatus", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitStatus", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitStatus", "Unknown");
                break;
        }

        // Setup logger data receivers - where logs are stored based on robot mode
        switch (AdvantageConstants.kCurrentMode) {
            case Real:
                // On real robot: log to USB drive and send to NetworkTables
                String date = new Date()
                    .toString()
                    .replaceAll(" ", "_")
                    .replaceAll(":", "-"); // Windows compatibility fix
                String log = String.format(
                    "/U/logs/akit_%s_%s.wpilog",
                    date,
                    BuildConstants.GIT_SHA
                );

                Logger.addDataReceiver(new WPILOGWriter(log));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case Sim:
                // In simulation: only send to NetworkTables
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case Replay:
                // When replaying: read from log file and run at full speed
                String logfile = LogFileUtil.findReplayLog();
                setUseTiming(false); // Run replay as fast as possible
                Logger.setReplaySource(new WPILOGReader(logfile));
                Logger.addDataReceiver(
                    new WPILOGWriter(LogFileUtil.addPathSuffix(logfile, ".sim"))
                );
                break;
        }

        // Register REV hardware logger for monitoring motor controllers
        Logger.registerURCL(URCL.startExternal());

        // Start the logging system
        Logger.start();

        // Start web server for Elastic dashboard remote access
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /**
     * Called when the robot is first powered on or enters initialization state.
     *
     * Used to initialize the robot container and set the initial dashboard tab.
     */
    @Override
    public void robotInit() {
        // Set the Elastic dashboard to the Autonomous tab on startup
        Elastic.selectTab("Autonomous");
    }

    /**
     * Called when the driver station connects to the robot.
     *
     * <p>Robot subsystems are initialized on first driver station connection rather than
     * at startup to avoid timing issues and ensure the robot is ready for control.
     */
    public void driverStationConnected() {
        if (container == null) container = new RobotContainer();
    }

    /**
     * Called periodically throughout all robot modes.
     *
     * <p>Runs the command scheduler which executes all scheduled commands and subsystem
     * periodic methods. This is the core periodic update loop for the robot.
     *
     * <p><strong>Performance note:</strong> Sets the thread to high priority (99) during execution to ensure
     * timing consistency, then returns to normal priority after the update.
     */
    @Override
    public void robotPeriodic() {
        if (container == null) return;

        // Give the robot loop very high thread priority for consistent timing
        Threads.setCurrentThreadPriority(true, 99);

        // Execute all scheduled commands and subsystem updates
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    /**
     * Called periodically while the robot is disabled.
     *
     * This method is reserved for logic that should run during disabled mode.
     * Currently unused but available for future implementations.
     */
    @Override
    public void disabledPeriodic() {}

    /**
     * Called once at the start of autonomous mode.
     *
     * Switches the dashboard to the Autonomous tab and retrieves the selected
     * autonomous command from the auto manager, then schedules it for execution.
     */
    @Override
    public void autonomousInit() {
        Elastic.selectTab("Autonomous");
        autoCommand = container.getAutonomousCommand();

        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

    /**
     * Called periodically during autonomous mode.
     *
     * This method is reserved for autonomous-specific periodic logic.
     * Currently unused as all logic is handled by the command scheduler.
     */
    @Override
    public void autonomousPeriodic() {}

    /**
     * Called once at the start of teleoperated mode.
     *
     * Switches the dashboard to the Teleoperated tab and cancels any running autonomous
     * commands to ensure clean transition to driver control.
     */
    @Override
    public void teleopInit() {
        Elastic.selectTab("Teleoperated");

        if (autoCommand != null) {
            autoCommand.cancel();
            CommandScheduler.getInstance().cancelAll();
        }
    }

    /**
     * Called periodically during teleoperated mode.
     *
     * This method is reserved for teleop-specific periodic logic.
     * Currently unused as all logic is handled by the command scheduler.
     */
    @Override
    public void teleopPeriodic() {}

    /**
     * Called once at the start of test mode.
     *
     * Cancels all running commands to ensure a clean test environment.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Called periodically during test mode.
     *
     * This method is reserved for test-specific periodic logic.
     * Currently unused but available for future test implementations.
     */
    @Override
    public void testPeriodic() {}

    /**
     * Called periodically during simulation.
     *
     * This method is reserved for simulation-specific updates.
     * Currently unused but available for physics simulation logic.
     */
    @Override
    public void simulationPeriodic() {}
}
