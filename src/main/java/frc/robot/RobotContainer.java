// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.alerter.Rumbler;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.drive.PointAtHub;
import frc.robot.constants.IOConstants;
import frc.robot.constants.RuntimeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.superstructure.SuperstructureIOSpark;
import frc.robot.util.AllianceUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Drive drive = new Drive(
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new GyroIONavX();
            case SIM -> new GyroIO() {};
            default -> new GyroIO() {};
        },
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> moduleId -> new ModuleIOSpark(moduleId);
            case SIM -> moduleId -> new ModuleIOSim();
            default -> moduleId -> new ModuleIO() {};
        }
    );

    private final Superstructure superstructure = new Superstructure(
        switch (RuntimeConstants.kCurrentMode) {
            case REAL -> new SuperstructureIOSpark();
            case SIM -> new SuperstructureIOSim();
            default -> new SuperstructureIO() {};
        }
    );

    // Controllers
    final CommandXboxController driverctl = new CommandXboxController(
        IOConstants.Controller.kDriver
    );
    final CommandXboxController operctl = new CommandXboxController(
        IOConstants.Controller.kOperator
    );

    // Auto selector
    final AutoSelector selector = new AutoSelector(drive);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Set up SysId routines
        // autoChooser.addOption(
        //     "Drive Wheel Radius Characterization",
        //     DriveCommands.wheelRadiusCharacterization(drive)
        // );
        // autoChooser.addOption(
        //     "Drive Simple FF Characterization",
        //     DriveCommands.feedforwardCharacterization(drive)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Forward)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Reverse)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Forward)",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Reverse)",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );

        // Configure the button bindings
        configureButtonBindings();

        Rumbler.getInstance().provideControllers(driverctl, operctl);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(new JoystickDrive(driverctl, drive));

        // Control bindings for superstructure
        driverctl.leftBumper().whileTrue(superstructure.intake());
        driverctl.y().whileTrue(new PointAtHub(drive, driverctl));
        driverctl.a().whileTrue(superstructure.eject());

        // Reset gyro to 0° when B button is pressed
        driverctl
            .start()
            .onTrue(
                Commands.runOnce(
                    // SAFETY: running this command means we see button presses, DS connected
                    () -> drive.setHeading(AllianceUtil.unsafe.zero()),
                    drive
                ).ignoringDisable(true)
            );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return selector.build(drive).cmd();
    }
}
