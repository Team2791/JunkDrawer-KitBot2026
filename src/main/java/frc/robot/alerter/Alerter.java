package frc.robot.alerter;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants;

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
 *   <li>Only monitors devices when running on real robots (not in replay mode)
 * </ul>
 *
 * <p>The alerting system is one-shot per error type, meaning each unique error is only
 * reported once to avoid flooding the dashboard with repeated notifications.
 */
public class Alerter {

    /** Singleton instance of the Alerter. */
    private static Alerter instance;

    private Notifier periodic = new Notifier(this::update);

    /** List of all monitored devices. */
    ArrayList<Device<?, ?>> devices = new ArrayList<>();

    /**
     * Private constructor, see {@link #getInstance()}.
     */
    private Alerter() {
        periodic.startPeriodic(TimedRobot.kDefaultPeriod);
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
     *
     * @param gyro The AHRS gyro instance to monitor
     */
    public void register(AHRS gyro) {
        register("Gyro", gyro, AHRS::isConnected);
    }

    /**
     * Registers a generic device for disconnection monitoring.
     *
     * <p>See {@link #register(String, SparkBase)} for details.
     *
     * @param <T> The device type
     * @param name Human-readable name for the device
     * @param device The device instance to monitor
     * @param isConnected Function to check if the device is connected
     */
    public <T> void register(
        String name,
        T device,
        Function<T, Boolean> isConnected
    ) {
        devices.add(
            new Device<>(
                device,
                name,
                isConnected,
                x -> x ? "" : "Disconnected",
                new ArrayList<>(List.of(true))
            )
        );
    }

    /**
     * Updates all device monitoring and sends alerts for any new errors.
     *
     * <p>This should be called periodically.
     * <p>Only monitors devices when running on a real robot - skips checks during replay.
     */
    private void update() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            return;
        }

        for (Device<?, ?> device : devices) {
            device.alert();
        }
    }
}
