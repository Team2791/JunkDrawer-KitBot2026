package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.util.RateLimiter;
import frc.robot.util.Vec2;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Command for driving the robot using joystick input from an Xbox controller.
 *
 * <p>This command
 * <ol>
 *  <li>Reads joystick inputs for translation and rotation
 *  <li>Applies deadbands and input shaping for finer control
 *  <li>Implements rate limiting for smooth acceleration
 *  <li>Transforms inputs to field-centric coordinates based on alliance color
 *  <li>Scales inputs to maximum speed limits
 *  <li>Commands the drive subsystem with the calculated chassis speeds
 * </ol>
 */
public class JoystickDrive extends Command {

    /** The Xbox controller used for driving */
    final CommandXboxController ctl;

    /** The drive subsystem to control */
    final Drive drive;

    /** Rate limiter for smoothing joystick inputs */
    final RateLimiter slew;

    public JoystickDrive(CommandXboxController ctl, Drive drive) {
        this.ctl = ctl;
        this.drive = drive;
        this.slew = new RateLimiter(
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kRotational
        );

        addRequirements(drive);
    }

    Vec2 linear() {
        Vec2 linear = new Vec2(ctl.getLeftX(), ctl.getLeftY());
        double mag = MathUtil.applyDeadband(
            linear.mag(),
            IOConstants.Controller.kDeadband
        );

        Vec2 input = linear.norm().mul(mag);
        Vec2 input2 = input.mul(input.abs());
        Vec2 limited = slew.calculateXY(input2);

        /*
         * Controller to WPI coordinate system conversion:
         *
         * Controller coordinates:    WPI coordinates:
         *   +X = right                 +X = forward
         *   +Y = down                  +Y = left
         *
         * Transformation needed:
         *   +Yc -> -Xw  (down stick -> forward motion negation)
         *   +Xc -> -Yw  (right stick -> left motion negation)
         */
        Vec2 cmd = new Vec2(-limited.y, -limited.x);

        /*
         * Field-centric drive coordinate transformation based on alliance color.
         *
         * BLUE ALLIANCE (origin on driver station side):
         *   - Joystick forward (+Y) -> Robot moves toward opposing alliance (+X field coords)
         *   - No transformation needed - joystick and field coordinates align naturally
         *
         * RED ALLIANCE (origin on opposing alliance side):
         *   - Joystick forward (+Y) -> Robot should move toward opposing alliance (-X field coords)
         *   - Linear commands must be inverted to maintain driver perspective
         *   - This ensures drivers have consistent controls regardless of alliance color
         */
        Optional<Boolean> invert = AllianceUtil.invert();
        if (invert.orElse(false)) cmd = cmd.neg(); // invert for Red alliance, skip if no FMS/DS

        // scale to max speed
        Vec2 vel = cmd.mul(ControlConstants.Drivetrain.MaxSpeed.kLinear);

        return vel;
    }

    double rot() {
        double rot = MathUtil.applyDeadband(
            ctl.getRightX(),
            IOConstants.Controller.kDeadband
        );

        double rot2 = rot * Math.abs(rot);
        double limited = slew.calculateRot(rot2);

        /*
         * Controller to WPI coordinate system conversion:
         *
         * Controller coordinates:    WPI coordinates:
         *   +Rot = clockwise           +Rot = counter-clockwise
         *
         * Transformation needed:
         *   +Rotc -> -Rotw (negate rotation for ccw-positive)
         */
        double rotcmd = -limited;
        double omega = rotcmd * ControlConstants.Drivetrain.MaxSpeed.kAngular;

        return omega;
    }

    @Override
    public void execute() {
        Vec2 vel = linear();
        double omega = rot();

        Logger.recordOutput("JSD/vel", vel);
        Logger.recordOutput("JSD/omeg", omega);

        drive.drive(new ChassisSpeeds(vel.x, vel.y, omega));
    }
}
