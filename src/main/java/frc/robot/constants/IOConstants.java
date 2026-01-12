package frc.robot.constants;

// import com.studica.frc.AHRS.NavXComType;

/** Yes, this is input/output not operator interface. CANId constants, mostly. */
public final class IOConstants {
    public static final class Drivetrain {
        // public static final NavXComType kGyroPort = NavXComType.kUSB1; // TODO: Season: may need MXP_SPI
    }

    public static final class Controller {
        public static final int kDriver = 0;
        public static final int kOperator = 1;

        public static final double kDeadband = 0.05;
    }
}
