package frc.robot.constants;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RPM;

public final class MotorConstants {
    public static final class Neo {
        /** https://www.revrobotics.com/rev-21-1650/#:~:text=empirical%20free%20speed%3A%205676%20rpm */
        public static final double kFreeSpeed = RPM.of(5676.0).in(RadiansPerSecond);
        public static final double kCurrentLimit = 40;
    }

    public static final class NeoVortex {
        /** https://www.revrobotics.com/rev-21-1652/#:~:text=free%20speed%3A%206784%20rpm */
        public static final double kFreeSpeed = RPM.of(6784.0).in(RadiansPerSecond);
        public static final double kCurrentLimit = 60;
    }

    public static final class Neo550 {
        /** https://www.revrobotics.com/rev-21-1651/#:~:text=free%20speed%3A%2011000%20rpm */
        public static final double kFreeSpeed = RPM.of(11000.0).in(RadiansPerSecond);
        public static final double kCurrentLimit = 20;
    }

    public static final double kNominalVoltage = 12.0;
}
