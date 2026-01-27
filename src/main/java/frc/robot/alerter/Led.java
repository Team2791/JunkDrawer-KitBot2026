package frc.robot.alerter;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * LED controller class for managing addressable LED strips with priority-based state management.
 *
 * <p>This class implements a singleton pattern to control an addressable LED strip connected to a PWM port.
 * It supports multiple LED modes (solid colors, rainbow, blinking) with a priority system that allows
 * different robot states to control the LEDs without conflicts.</p>
 *
 * <p>The priority system ensures that more important alerts (like robot alerts) take precedence over
 * less important ones (like idle states). States are automatically sorted by priority level.</p>
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * Led led = Led.getInstance();
 *
 * // Register different LED states with priorities
 * led.register(() -> robot.isDisabled(), Led.Mode.solid(255, 0, 0), Led.Priority.STATE);
 * led.register(() -> robot.hasError(), Led.Mode.blink(255, 255, 0), Led.Priority.ALERT);
 * led.register(() -> true, Led.Mode.rainbow(), Led.Priority.IDLE);
 * }</pre>
 *
 * <p>The LED strip is automatically updated at the robot's default period rate using a Notifier.
 * Only the highest priority active state will be displayed at any given time.</p>
 *
 * @see Mode
 * @see Priority
 */
public class Led {

    /** PWM port for the LED strip */
    private static final int LED_PORT = 0;

    /** Number of LEDs on the strip */
    private static final int LED_COUNT = 60;

    /** Speed of the rainbow animation */
    private static final double RAINBOW_SPEED = 160.0;

    /** Number of rainbows (180deg hue rotations) in the strip */
    private static final int RAINBOW_COUNT = 2;

    /** Interval for blinking in seconds */
    private static final double BLINK_INTERVAL = 0.5;

    /** Addressable LED instance */
    private final AddressableLED led;

    /** Buffer for LED colors */
    private final AddressableLEDBuffer buffer;

    /**
     * Represents a LED display mode that defines how LEDs should behave.
     * Each mode encapsulates a specific behavior pattern that can be applied to LED strips.
     *
     * <p>This class uses the command pattern to encapsulate LED behaviors as reusable modes.
     * The behavior is defined as a Consumer function that operates on an LED instance.
     *
     * <p>Available modes include:
     * <ul>
     *   <li>Solid color display</li>
     *   <li>Off (all LEDs turned off)</li>
     *   <li>Rainbow color pattern</li>
     *   <li>Blinking pattern</li>
     * </ul>
     */
    public static class Mode {

        /** Behavior function defining how the LEDs should be updated */
        private final Consumer<Led> behavior;

        /** Constructor to create a Mode with a specific behavior */
        private Mode(Consumer<Led> behavior) {
            this.behavior = behavior;
        }

        /**
         * Creates a solid color mode.
         *
         * @param r Red component (0-255)
         * @param g Green component (0-255)
         * @param b Blue component (0-255)
         * @return A {@link Mode} instance representing a solid color display
         */
        public static Mode solid(int r, int g, int b) {
            return new Mode(leds -> leds.solid(r, g, b));
        }

        /**
         * Creates an off mode (all LEDs turned off).
         * @return A {@link Mode} instance representing all LEDs turned off
         */
        public static Mode off() {
            return new Mode(leds -> leds.solid(0, 0, 0));
        }

        /**
         * Creates a rainbow mode.
         * @return A {@link Mode} instance representing a rainbow color pattern
         */
        public static Mode rainbow() {
            return new Mode(leds -> leds.rainbow());
        }

        /**
         * Creates a blinking mode.
         *
         * @param r Red component (0-255)
         * @param g Green component (0-255)
         * @param b Blue component (0-255)
         * @return A {@link Mode} instance representing a blinking pattern
         */
        public static Mode blink(int r, int g, int b) {
            return new Mode(leds -> leds.blink(r, g, b));
        }
    }

    /**
     * Enumeration representing different priority levels for LED alerts.
     *
     * <p>Each priority has an associated numeric level, with higher numbers indicating higher priority.
     *
     * <p>Priority levels from lowest to highest:</p>
     * <ul>
     *   <li>{@link #IDLE} (0) - No active alerts, default state (e.g. teleop vs autonomous)</li>
     *   <li>{@link #CONTEXT} (1) - Contextual information display (e.g. DriverStation not connected)</li>
     *   <li>{@link #STATE} (2) - Robot state indicators (e.g. shooting, intaking)</li>
     *   <li>{@link #IMPORTANT} (3) - Important notifications requiring attention (e.g. intake full or empty)</li>
     *   <li>{@link #ALERT} (4) - Critical alerts requiring immediate attention (e.g. motor failure)</li>
     * </ul>
     */
    public enum Priority {
        IDLE(0),
        CONTEXT(1),
        STATE(2),
        IMPORTANT(3),
        ALERT(4);

        public final int level;

        private Priority(int level) {
            this.level = level;
        }
    }

    /** Internal record representing a registered LED state with its activation condition and priority */
    private static record State(
        BooleanSupplier active,
        Mode mode,
        Priority priority
    ) {}

    /** List of registered LED states */
    private ArrayList<State> states = new ArrayList<>();

    /** Singleton instance of the LED controller */
    private static Led instance;

    /** Notifier for periodic LED updates */
    private Notifier periodic = new Notifier(this::update);

    /**
     * Private constructor for singleton pattern
     *
     * <ol>
     *  <li>Initializes the AddressableLED and buffer</li>
     *  <li>Starts the LED output</li>
     *  <li>Starts the periodic Notifier to update LED states</li>
     * </ol>
     *
     * @see #getInstance()
     */
    private Led() {
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(LED_COUNT);
        led.start();

        periodic.startPeriodic(TimedRobot.kDefaultPeriod);
    }

    /**
     * Gets the singleton instance of the LED controller.
     * @return The LED singleton instance
     */
    public static synchronized Led getInstance() {
        if (instance == null) instance = new Led();
        return instance;
    }

    /**
     * Registers a new LED state with its activation condition and priority.
     *
     * <p>The states are automatically sorted by priority level, ensuring that higher
     * priority states take precedence during updates.</p>
     *
     * @param active A BooleanSupplier that returns true when this state should be active
     * @param mode The LED mode to display when this state is active
     * @param priority The priority level of this state
     */
    public synchronized void register(
        BooleanSupplier active,
        Mode mode,
        Priority priority
    ) {
        states.add(new State(active, mode, priority));
        states.sort((a, b) -> b.priority.level - a.priority.level);
    }

    /** Set buffer to a solid color */
    private void solid(int r, int g, int b) {
        for (int i = 0; i < LED_COUNT; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    /** Blink the LEDs with a specified color */
    private void blink(int r, int g, int b) {
        boolean on = (Timer.getFPGATimestamp() / BLINK_INTERVAL) % 2 == 0;

        if (on) solid(r, g, b);
        else solid(0, 0, 0);
    }

    /** Display a rainbow pattern on the LEDs */
    private void rainbow() {
        double increment = (180.0 * RAINBOW_COUNT) / LED_COUNT;
        double offset = (Timer.getFPGATimestamp() * RAINBOW_SPEED) % 180;

        for (int i = 0; i < LED_COUNT; i++) {
            double hue = (i * increment + offset) % 180;
            buffer.setHSV(i, (int) hue, 255, 128);
        }
    }

    /**
     * Periodic update method to evaluate and apply the highest priority active LED state.
     *
     * <p>This method checks all registered states in order of priority and applies
     * the first active state's mode to the LED strip.</p>
     */
    private synchronized void update() {
        // get highest priority active state, sorted therefore first
        for (State state : states) {
            if (state.active.getAsBoolean()) {
                state.mode.behavior.accept(this);
                break;
            }
        }

        led.setData(buffer);
    }
}
