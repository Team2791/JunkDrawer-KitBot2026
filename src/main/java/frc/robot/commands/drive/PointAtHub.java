package frc.robot.commands.drive;

import static frc.robot.util.MathPlus.kTau;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Vec2;
import org.littletonrobotics.junction.Logger;

public class PointAtHub extends Command {

    final Drive drive;
    final JoystickDrive jsd;

    /** Profiled PID for rotation with velocity/acceleration constraints. */
    PIDController rotController = new PIDController(
        ControlConstants.Nearby.kTurnP,
        ControlConstants.Nearby.kTurnI,
        ControlConstants.Nearby.kTurnD
    );

    public PointAtHub(Drive drive, CommandXboxController ctl) {
        this.drive = drive;
        this.jsd = new JoystickDrive(ctl, drive);
        addRequirements(drive);

        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(
            ControlConstants.Nearby.kTolerance.getRotation().getRadians()
        );

        SmartDashboard.putData("Point/RotPID", rotController);
    }

    public void initialize() {}

    public void execute() {
        Pose2d posBlue = AllianceUtil.unsafe.autoflip(drive.getPose());
        Pose2d blueHub = GameConstants.Objects.kHub;
        Vec2 delta = new Vec2(posBlue).sub(new Vec2(blueHub));
        Rotation2d theta = delta.theta().plus(Rotation2d.kPi);
        double rot = rotController.calculate(
            posBlue.getRotation().getRadians(),
            theta.getRadians()
        );
        Vec2 linear = jsd.linear();

        Logger.recordOutput(
            "Point/Rot",
            new Pose2d(posBlue.getTranslation(), theta)
        );

        Logger.recordOutput("Point/Cur", posBlue);

        drive.drive(new ChassisSpeeds(linear.x, linear.y, -rot));
    }
}
