package frc.robot.constants;

import com.studica.frc.AHRS.NavXComType;

/**
 * Input/Output port assignments and hardware interface constants.
 *
 * <p><strong>Note:</strong> "IO" refers to Input/Output, not Operator Interface.
 *
 * <p>Contains CAN IDs, USB port assignments, and controller port mappings
 * for all robot hardware peripherals. Organized by subsystem for clarity.
 */
public final class IOConstants {

    /**
     * Drivetrain hardware port assignments.
     *
     * <p>Currently contains only gyroscope configuration (commented out).
     * TODO: May need MXP SPI connection instead of USB1 depending on NavX mounting.
     */
    public static final class Drivetrain {

        public static final NavXComType kGyroPort = NavXComType.kUSB1; // TODO: Season: may need MXP_SPI
    }

    /**
     * Driver station controller port mappings and input filtering.
     *
     * <p>Defines USB ports for driver and operator controllers, plus
     * joystick deadband for eliminating stick drift.
     */
    public static final class Controller {

        /** Driver controller USB port (primary pilot). */
        public static final int kDriver = 0;

        /** Operator controller USB port (secondary controls). */
        public static final int kOperator = 1;

        /**
         * Joystick deadband threshold.
         *
         * <p>Inputs within ±{@code 0.05} of center are treated as zero
         * to prevent unintended movement from controller drift.
         */
        public static final double kDeadband = 0.05;
    }
}
