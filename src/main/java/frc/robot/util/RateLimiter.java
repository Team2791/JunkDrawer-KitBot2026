package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class RateLimiter {

    public record Outputs(Vec2 vel, double rot) {}

    final SlewRateLimiter xLimiter;
    final SlewRateLimiter yLimiter;
    final SlewRateLimiter rotLimiter;

    public RateLimiter(double xRate, double yRate, double rotRate) {
        xLimiter = new SlewRateLimiter(xRate);
        yLimiter = new SlewRateLimiter(yRate);
        rotLimiter = new SlewRateLimiter(rotRate);
    }

    /**
     * Calculate the new speeds based on the controller inputs
     *
     * @param xspeed the [-1, 1] value from the controller for x speed
     * @param yspeed the [-1, 1] value from the controller for y speed
     * @param rot    the [-1, 1] value from the controller for rotation
     */
    public Outputs calculate(Vec2 speed, double rot) {
        return new Outputs(
            speed.apply(xLimiter::calculate, yLimiter::calculate),
            rotLimiter.calculate(rot)
        );
    }
}
