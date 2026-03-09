// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    private final SuperstructureIO io;
    private final SuperstructureIOInputsAutoLogged inputs =
        new SuperstructureIOInputsAutoLogged();

    public Superstructure(SuperstructureIO io) {
        this.io = io;

        SmartDashboard.putNumber("SS/ShooterVel", 250);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Superstructure", inputs);

        double vel = SmartDashboard.getNumber("SS/ShooterVel", 250);
        io.setLauncherVelocity(vel);
    }

    public SuperstructureIOInputs data() {
        return inputs.clone();
    }

    /** Set the rollers to the values for intaking. */
    public Command intake() {
        return runEnd(
            () -> {
                io.setFeederVoltage(intakingFeederVoltage);
                io.setLauncherVelocity(intakingLauncherVelocity);
            },
            () -> {
                io.setFeederVoltage(0.0);
                io.setLauncherVelocity(0.0);
            }
        );
    }

    /** Set the rollers to the values for ejecting fuel out the intake. */
    public Command eject() {
        return runEnd(
            () -> {
                io.setFeederVoltage(-intakingFeederVoltage);
                io.setLauncherVelocity(-intakingLauncherVelocity);
            },
            () -> {
                io.setFeederVoltage(0.0);
                io.setLauncherVelocity(0.0);
            }
        );
    }

    /** Set the rollers to the values for launching. Spins up before feeding fuel. */
    public void spinUp() {
        io.setLauncherVelocity(launchingLauncherVelocity);
    }

    public boolean launcherAtSetpoint() {
        return (
            Math.abs(
                inputs.intakeLauncherVelocityRadPerSec -
                    launchingLauncherVelocity
            ) <=
            50
        );
    }
}
