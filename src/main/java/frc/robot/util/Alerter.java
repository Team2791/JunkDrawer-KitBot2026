package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AdvantageConstants;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class Alerter {

    record Device<T, E>(
        T device,
        String name,
        Function<T, E> error,
        Function<E, String> serialize,
        ArrayList<E> signaled
    ) {
        void alert() {
            E err = error.apply(device);
            if (err == null || signaled.contains(err)) return;

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

    private static Alerter instance;

    ArrayList<Device> devices = new ArrayList<>();

    CommandXboxController driverctl;
    CommandXboxController operctl;
    Notifier timer = new Notifier(this::still);

    private Alerter() {
        timer.setName("VibrateTimer");
    }

    public static synchronized Alerter getInstance() {
        if (instance == null) {
            instance = new Alerter();
        }

        return instance;
    }

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

    public void provideControllers(
        CommandXboxController driverctl,
        CommandXboxController operctl
    ) {
        assert this.driverctl == null &&
        this.operctl == null : "Controllers already provided";

        this.driverctl = driverctl;
        this.operctl = operctl;
    }

    public void rumble() {
        assert driverctl != null &&
        operctl != null : "Controllers not provided";

        // no rumble in auto
        if (DriverStation.isAutonomous()) return;

        this.operctl.setRumble(RumbleType.kBothRumble, 1);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 1);

        timer.stop();
        timer.startSingle(0.5);
    }

    private void still() {
        this.operctl.setRumble(RumbleType.kBothRumble, 0);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 0);
    }

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

    public void update() {
        if (
            AdvantageConstants.kCurrentMode !=
            AdvantageConstants.AdvantageMode.Real
        ) {
            return;
        }

        for (Device device : devices) {
            device.alert();
        }
    }
}
