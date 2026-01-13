package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AdvantageConstants;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Monitors hardware devices for errors and provides alerts to the driver and operator.
 *
 * <p>This singleton class:
 * <ul>
 *   <li>Tracks registered devices (motors, sensors, etc.) and their error states
 *   <li>Sends notifications to the Elastic dashboard when hardware fails
 *   <li>Provides controller vibration feedback for important events
 *   <li>Only monitors devices when running on real robots (not in replay mode)
 * </ul>
 *
 * <p>The alerting system is one-shot per error type, meaning each unique error is only
 * reported once to avoid flooding the dashboard with repeated notifications.
 */
public class Alerter {

    /**
     * Internal record representing a monitored device and its error state.
     *
     * @param <T> The device type
     * @param <E> The error type for this device
     */
    record Device<T, E>(
        /** The device instance being monitored. */
        T device,
        /** Human-readable name of the device. */
        String name,
        /** Function to extract the current error state from the device. */
        Function<T, E> error,
        /** Function to convert an error to a human-readable description. */
        Function<E, String> serialize,
        /** List of errors that have already been reported (to avoid duplicates). */
        ArrayList<E> signaled
    ) {
        /**
         * Checks the device for a new error and alerts if one is found.
         *
         * Only reports each unique error once - subsequent occurrences are ignored.
         */
        void alert() {
            E err = error.apply(device);
            if (err == null || signaled.contains(err)) return;

            // Send notification to Elastic dashboard
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.ERROR,
                    "Device failed",
                    String.format("%s reported: %s", name, serialize.apply(err))
                )
            );

            signaled.add(err);
        }
    }

    /** Singleton instance of the Alerter. */
    private static Alerter instance;

    /** List of all monitored devices. */
    ArrayList<Device<?, ?>> devices = new ArrayList<>();

    /** Driver controller for rumble feedback. */
    CommandXboxController driverctl;

    /** Operator controller for rumble feedback. */
    CommandXboxController operctl;

    /** Timer for controlling rumble duration. */
    Notifier timer = new Notifier(this::still);

    /**
     * Private constructor - use getInstance() instead.
     *
     * Initializes the vibration timer with an appropriate name for debugging.
     */
    private Alerter() {
        timer.setName("VibrateTimer");
    }

    /**
     * Gets the singleton instance of the Alerter.
     *
     * Creates the instance on first call. Thread-safe using synchronized keyword.
     *
     * @return The Alerter singleton instance
     */
    public static synchronized Alerter getInstance() {
        if (instance == null) {
            instance = new Alerter();
        }

        return instance;
    }

    /**
     * Converts a REVLibError to a human-readable error message.
     *
     * @param error The error code from a REV motor controller
     * @return A descriptive error message
     */
    private static String serialize(REVLibError error) {
        return switch (error) {
            case kOk -> "Everything is fine";
            case kError -> "General error";
            case kTimeout -> "Spark took too long to respond";
            case kNotImplemented -> "Function not implemented";
            case kHALError -> "Hardware abstraction layer error";
            case kCantFindFirmware -> "No firmware found on Spark";
            case kFirmwareTooOld -> "Firmware version is too old to be used with this library";
            case kFirmwareTooNew -> "Firmware version is too new to be used with this library";
            case kParamInvalidID -> "Invalid parameter ID";
            case kParamMismatchType -> "Parameter type mismatch";
            case kParamAccessMode -> "Parameter access mode mismatch";
            case kParamInvalid -> "Invalid parameter";
            case kParamNotImplementedDeprecated -> "Parameter not implemented or deprecated";
            case kFollowConfigMismatch -> "Follower configuration mismatch";
            case kInvalid -> "Invalid Spark configuration";
            case kSetpointOutOfRange -> "Motor setpoint out of range";
            case kUnknown -> "Unknown error";
            case kCANDisconnected -> "CAN bus was disconnected";
            case kDuplicateCANId -> "Duplicate CAN ID detected on bus";
            case kInvalidCANId -> "Spark has invalid can ID";
            case kSparkMaxDataPortAlreadyConfiguredDifferently -> "SparkMax data port already configured differently";
            case kSparkFlexBrushedWithoutDock -> "SparkFlex brushed motor without dock detected";
            case kInvalidBrushlessEncoderConfiguration -> "Invalid brushless encoder configuration";
            case kFeedbackSensorIncompatibleWithDataPortConfig -> "Sensor not compatible with data port configuration";
            case kParamInvalidChannel -> "Invalid parameter channel";
            case kParamInvalidValue -> "Invalid parameter value";
            case kCannotPersistParametersWhileEnabled -> "Cannot persist parameters while Spark is enabled";
        };
    }

    /**
     * Provides the Xbox controllers to the alerter for rumble feedback.
     *
     * Must be called before using rumble() functionality. Asserts that controllers
     * haven't already been provided to prevent accidental reconfiguration.
     *
     * @param driverctl The driver's Xbox controller
     * @param operctl The operator's Xbox controller
     */
    public void provideControllers(
        CommandXboxController driverctl,
        CommandXboxController operctl
    ) {
        assert this.driverctl == null &&
        this.operctl == null : "Controllers already provided";

        this.driverctl = driverctl;
        this.operctl = operctl;
    }

    /**
     * Triggers controller vibration feedback for 0.5 seconds.
     *
     * <p>Activates different rumble patterns on each controller:
     * <ul>
     *   <li>Operator: Both-sided rumble
     *   <li>Driver: Left-side rumble
     * </ul>
     *
     * <p>No rumble occurs during autonomous mode.
     * The vibration is controlled by a timer that automatically stops after 0.5 seconds.
     */
    public void rumble() {
        assert driverctl != null &&
        operctl != null : "Controllers not provided";

        // Don't rumble during autonomous
        if (DriverStation.isAutonomous()) return;

        this.operctl.setRumble(RumbleType.kBothRumble, 1);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 1);

        timer.stop();
        timer.startSingle(0.5);
    }

    /**
     * Stops all controller vibration.
     *
     * Called automatically by the timer after the rumble duration expires.
     */
    private void still() {
        this.operctl.setRumble(RumbleType.kBothRumble, 0);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 0);
    }

    /**
     * Registers a REV Spark motor controller for error monitoring.
     *
     * <p>The controller will be checked periodically for errors, and any new errors
     * will trigger a notification on the Elastic dashboard.
     *
     * @param name Human-readable name for the motor (e.g., "Drive Left Front")
     * @param spark The Spark motor controller instance to monitor
     */
    public void register(String name, SparkBase spark) {
        devices.add(
            new Device<>(
                spark,
                name,
                SparkBase::getLastError,
                Alerter::serialize,
                new ArrayList<>(List.of(REVLibError.kOk))
            )
        );
    }

    /**
     * Registers a gyroscope for error monitoring.
     *
     * <p>See {@link #register(String, SparkBase)} for details.
     */
    public void register(AHRS gyro) {
        devices.add(
            new Device<>(
                gyro,
                "Gyro",
                AHRS::isConnected,
                x -> x ? "" : "Disconnected",
                new ArrayList<>(List.of(true))
            )
        );
    }

    // TODO: when gyro library is updated.
    // public void register(AHRS gyro) {
    //     devices.add(
    //         new Device<>(
    //             gyro,
    //             "Gyro",
    //             AHRS::isConnected,
    //             x -> x ? "" : "Disconnected",
    //             new ArrayList<>(List.of(true))
    //         )
    //     );
    // }

    /**
     * Updates all device monitoring and sends alerts for any new errors.
     *
     * This should be called periodically (typically in robotPeriodic).
     * Only monitors devices when running on a real robot - skips checks during replay.
     */
    public void update() {
        if (
            AdvantageConstants.kCurrentMode !=
            AdvantageConstants.AdvantageMode.Real
        ) {
            return;
        }

        for (Device<?, ?> device : devices) {
            device.alert();
        }
    }
}
