package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * A rate limiter for constraining acceleration of robot movement inputs.
 *
 * This class wraps WPILib's SlewRateLimiter to provide a convenient interface for
 * limiting the rate of change of velocity in three axes: X, Y, and rotation.
 *
 * Rate limiting ensures smooth, predictable robot motion and prevents sudden jerky
 * movements that could cause traction loss or unexpected behavior. It's particularly
 * useful for managing joystick inputs.
 */
public class RateLimiter {

    /**
     * Record containing the calculated limited velocities.
     *
     * @param vel The limited X and Y velocities
     * @param rot The limited rotational velocity
     */
    public record Outputs(Vec2 vel, double rot) {}

    /** Slew rate limiter for X-axis velocity. */
    final SlewRateLimiter xLimiter;

    /** Slew rate limiter for Y-axis velocity. */
    final SlewRateLimiter yLimiter;

    /** Slew rate limiter for rotational velocity. */
    final SlewRateLimiter rotLimiter;

    /**
     * Constructs a rate limiter with specified maximum rates of change for each axis.
     *
     * @param xRate The maximum rate of change (units/sec²) for X-axis velocity
     * @param yRate The maximum rate of change (units/sec²) for Y-axis velocity
     * @param rotRate The maximum rate of change (rad/sec²) for rotational velocity
     */
    public RateLimiter(double xRate, double yRate, double rotRate) {
        xLimiter = new SlewRateLimiter(xRate);
        yLimiter = new SlewRateLimiter(yRate);
        rotLimiter = new SlewRateLimiter(rotRate);
    }

    /**
     * Calculates rate-limited velocities based on the provided inputs.
     *
     * Each component of velocity is passed through its corresponding slew rate limiter,
     * which constrains the acceleration to the configured maximum rates.
     *
     * @param speed The requested velocity as a Vec2, where each component is in [-1, 1]
     * @param rot The requested rotational velocity in [-1, 1]
     * @return An Outputs record containing the rate-limited velocities
     */
    public Outputs calculate(Vec2 speed, double rot) {
        return new Outputs(
            speed.apply(xLimiter::calculate, yLimiter::calculate),
            rotLimiter.calculate(rot)
        );
    }
}
