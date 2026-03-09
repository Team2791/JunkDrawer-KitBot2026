package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class SpinUp extends Command {

    final Superstructure superstructure;

    public SpinUp(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    public void initialize() {
        superstructure.spinUp();
    }

    public boolean isFinished() {
        return superstructure.launcherAtSetpoint();
    }
}
