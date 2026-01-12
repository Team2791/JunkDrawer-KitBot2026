package frc.robot.subsystems.drivetrain.module;

import frc.robot.constants.SwerveConstants;

public class ModuleReplay extends ModuleIO {

    public ModuleReplay(SwerveConstants.Module id) {
        super(id);
    }

    @Override
    public void update() {}

    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) {}
}
