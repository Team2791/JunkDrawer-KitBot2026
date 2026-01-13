package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * REV Robotics brushless motor specifications.
 *
 * <p>Contains empirically measured free speeds and recommended current limits
 * for each motor type used on the robot. Free speeds are converted from RPM
 * to radians/second for kinematic calculations.
 *
 * <p>Specification sources:
 * <ul>
 *   <li>NEO: <a href="https://www.revrobotics.com/rev-21-1650/">REV-21-1650</a>
 *   <li>NEO Vortex: <a href="https://www.revrobotics.com/rev-21-1652/">REV-21-1652</a>
 *   <li>NEO 550: <a href="https://www.revrobotics.com/rev-21-1651/">REV-21-1651</a>
 * </ul>
 */
public final class MotorConstants {

    /**
     * NEO brushless motor specifications (REV-21-1650).
     *
     * <p>Used for swerve module drive motors.
     */
    public static final class Neo {

        /**
         * Empirical free speed: 5676 RPM (594.7 rad/s).
         *
         * @see <a href="https://www.revrobotics.com/rev-21-1650/#:~:text=empirical%20free%20speed%3A%205676%20rpm">Datasheet</a>
         */
        public static final double kFreeSpeed = RPM.of(5676.0).in(
            RadiansPerSecond
        );

        /** Recommended current limit: 40 amps. */
        public static final double kCurrentLimit = 40;
    }

    /**
     * NEO Vortex brushless motor specifications (REV-21-1652).
     *
     * <p>Higher power motor for demanding applications.
     */
    public static final class NeoVortex {

        /**
         * Free speed: 6784 RPM (710.3 rad/s).
         *
         * @see <a href="https://www.revrobotics.com/rev-21-1652/#:~:text=free%20speed%3A%206784%20rpm">Datasheet</a>
         */
        public static final double kFreeSpeed = RPM.of(6784.0).in(
            RadiansPerSecond
        );

        /** Recommended current limit: 60 amps. */
        public static final double kCurrentLimit = 60;
    }

    /**
     * NEO 550 brushless motor specifications (REV-21-1651).
     *
     * <p>Compact, high-speed motor used for swerve module steering.
     */
    public static final class Neo550 {

        /**
         * Free speed: 11000 RPM (1151.9 rad/s).
         *
         * @see <a href="https://www.revrobotics.com/rev-21-1651/#:~:text=free%20speed%3A%2011000%20rpm">Datasheet</a>
         */
        public static final double kFreeSpeed = RPM.of(11000.0).in(
            RadiansPerSecond
        );

        /** Recommended current limit: 20 amps. */
        public static final double kCurrentLimit = 20;
    }

    /** Nominal battery voltage for motor control (12.0 volts). */
    public static final double kNominalVoltage = 12.0;
}
