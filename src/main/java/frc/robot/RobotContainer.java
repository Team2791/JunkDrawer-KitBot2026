package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoManager;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IOConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroReplay;
import frc.robot.subsystems.drivetrain.gyro.NavX;
import frc.robot.subsystems.drivetrain.module.ModuleReplay;
import frc.robot.subsystems.drivetrain.module.ModuleSpark;
import frc.robot.subsystems.photon.CameraPhoton;
import frc.robot.subsystems.photon.CameraReplay;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.quest.Quest;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.Alerter;

/**
 * This class is responsible for binding the controls on the driver and operator controllers
 * to actions and commands. It also manages the robot's subsystems and autonomous command selection.
 *
 * Robot subsystems are created here with appropriate implementations selected based on the
 * current robot mode (real, simulation, or replay) via the AdvantageUtil.match() utility.
 */
public class RobotContainer {

    /** Driver controller (left hand side), typically used for drive control */
    final CommandXboxController driverctl;

    /** Operator controller (right hand side), typically used for mechanism control */
    final CommandXboxController operctl;

    /** Swerve drivetrain subsystem - handles robot movement and odometry */
    final Drivetrain drivetrain = new Drivetrain(
        AdvantageUtil.match(NavX::new, GyroReplay::new),
        AdvantageUtil.match(ModuleSpark::new, ModuleReplay::new)
    );

    /** Photon vision subsystem - handles camera-based pose estimation */
    final Photon photon = new Photon(
        drivetrain::addVisionMeasurement,
        AdvantageUtil.match(CameraPhoton::new, CameraReplay::new),
        VisionConstants.CameraConfig.kCamera
    );

    /** Quest vision subsystem - handles secondary vision processing */
    final Quest quest = new Quest(drivetrain::addVisionMeasurement);

    /** Autonomous mode manager - handles auto command selection and execution */
    final AutoManager autoManager = new AutoManager(drivetrain);

    /**
     * Constructs the RobotContainer, initializing all subsystems and binding commands to controls.
     *
     * <p>This constructor:
     * <ul>
     *   <li>Creates the driver and operator Xbox controllers
     *   <li>Configures button bindings for driver inputs
     *   <li>Sets up the alerting system with controllers
     *   <li>Initializes the camera server
     * </ul>
     */
    public RobotContainer() {
        // Create driver and operator controllers with their port assignments
        this.driverctl = new CommandXboxController(
            IOConstants.Controller.kDriver
        );
        this.operctl = new CommandXboxController(
            IOConstants.Controller.kOperator
        );

        // Configure all button bindings and control schemes
        configureBindings();

        // Provide controllers to the alerting system for controller vibration feedback
        Alerter.getInstance().provideControllers(driverctl, operctl);

        // Initialize camera server for USB camera feeds
        CameraServer.startAutomaticCapture();

        // Remove the default USB Camera 0 that causes conflicts
        // (We still need to initialize CameraServer, this just removes the default stream)
        CameraServer.removeCamera("USB Camera 0");
    }

    /**
     * Configures all button bindings and command associations for driver and operator controls.
     *
     * <p>This method sets up:
     * <ul>
     *   <li>Default drive command for joystick control
     *   <li>Button bindings for various robot functions
     *   <li>Operator controls for mechanism manipulation
     * </ul>
     */
    private void configureBindings() {
        // Create the default joystick drive command
        Command joystickDrive = new RunCommand(
            () -> drivetrain.drive(driverctl),
            drivetrain
        );

        // Set joystick drive as the default command for the drivetrain
        drivetrain.setDefaultCommand(joystickDrive);

        // Bind the start button to reset the gyro
        driverctl
            .start()
            .onTrue(
                new FunctionWrapper(drivetrain::resetGyro).ignoringDisable(true)
            );
    }

    /**
     * Returns the command to run during the autonomous phase.
     *
     * This method is called by the Robot class at the start of autonomous mode.
     * The selected autonomous routine is determined by the AutoManager.
     *
     * @return The autonomous command to execute, or null if no autonomous routine is selected
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
