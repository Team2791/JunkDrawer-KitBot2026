package frc.robot.alerter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Rumbler class for controller vibration feedback.
 *
 * <p>Provides rumble functionality for driver and operator controllers.
 * The rumble lasts for a fixed duration controlled by a timer.
 */
public class Rumbler {
    /** Driver controller for rumble feedback. */
    CommandXboxController driverctl;

    /** Operator controller for rumble feedback. */
    CommandXboxController operctl;

    /** Timer for controlling rumble duration. */
    Notifier timer = new Notifier(this::still);

    /** Singleton instance of the Rumbler. */
    static final Rumbler instance = new Rumbler();

    /** Private constructor, see {@link #getInstance()}. */
    private Rumbler() {
        timer.setName("RumbleStillTimer");
    }

    /** Stops the rumble on both controllers. */
    private void still() {
        this.operctl.setRumble(RumbleType.kBothRumble, 0);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 0);
    }

    /** Gets the singleton instance of the Rumbler. */
    public static Rumbler getInstance() {
        return instance;
    }

    /**
     * Provides the Xbox controllers to the alerter for rumble feedback.
     *
     * @param driverctl The driver's Xbox controller
     * @param operctl The operator's Xbox controller
     */
    public void provideControllers(
        CommandXboxController driverctl,
        CommandXboxController operctl
    ) {
        if (this.driverctl != null || this.operctl != null) {
            return;
        }

        this.driverctl = driverctl;
        this.operctl = operctl;
    }

    /**
     * Triggers controller vibration feedback for 0.5 seconds.
     *
     * <p>No rumble occurs during autonomous mode.
     * The vibration is controlled by a timer that automatically stops after 0.5 seconds.
     */
    public void rumble() {
        if (this.driverctl == null || this.operctl == null) {
            return;
        }

        // Don't rumble during autonomous
        if (edu.wpi.first.wpilibj.DriverStation.isAutonomous()) return;

        this.operctl.setRumble(RumbleType.kBothRumble, 1);
        this.driverctl.setRumble(RumbleType.kBothRumble, 1);

        this.timer.stop();
        this.timer.startSingle(0.5);
    }
}
