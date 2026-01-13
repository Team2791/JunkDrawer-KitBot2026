package frc.robot.util;

import java.util.function.Function;

/**
 * A 2D vector class for representing and manipulating 2D coordinates and directions.
 *
 * This immutable class provides common vector operations including addition, subtraction,
 * scalar multiplication, dot product, and magnitude calculations. It's commonly used for
 * representing velocities, positions, and forces in 2D space.
 *
 * The class is immutable, meaning all operations return new Vec2 instances rather than
 * modifying the original object.
 */
public class Vec2 {

    /** The X component of the vector. */
    public final double x;

    /** The Y component of the vector. */
    public final double y;

    /** Constant for the zero vector (0, 0). */
    public static final Vec2 ZERO = new Vec2(0, 0);

    /** Constant for the unit vector (1, 1). */
    public static final Vec2 ONE = new Vec2(1, 1);

    /** Constant for the unit vector along the X axis (1, 0). */
    public static final Vec2 UNIT_X = new Vec2(1, 0);

    /** Constant for the unit vector along the Y axis (0, 1). */
    public static final Vec2 UNIT_Y = new Vec2(0, 1);

    /**
     * Constructs a new Vec2 with the given X and Y components.
     *
     * @param x The X component
     * @param y The Y component
     */
    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a new Vec2 with both components set to the same value.
     *
     * @param value The value for both x and y components
     * @return A new Vec2 with equal x and y components
     */
    public static Vec2 fill(double value) {
        return new Vec2(value, value);
    }

    /**
     * Adds another vector to this vector.
     *
     * @param other The vector to add
     * @return A new Vec2 representing the sum of this and other
     */
    public Vec2 add(Vec2 other) {
        return new Vec2(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another vector from this vector.
     *
     * @param other The vector to subtract
     * @return A new Vec2 representing the difference (this - other)
     */
    public Vec2 sub(Vec2 other) {
        return new Vec2(this.x - other.x, this.y - other.y);
    }

    /**
     * Multiplies this vector by a scalar value.
     *
     * @param scalar The scalar multiplier
     * @return A new Vec2 with both components multiplied by the scalar
     */
    public Vec2 mul(double scalar) {
        return new Vec2(this.x * scalar, this.y * scalar);
    }

    /**
     * Performs element-wise multiplication with another vector.
     *
     * @param elementwise The vector to multiply element-wise
     * @return A new Vec2 with each component multiplied (x*x, y*y)
     */
    public Vec2 mul(Vec2 elementwise) {
        return new Vec2(this.x * elementwise.x, this.y * elementwise.y);
    }

    /**
     * Raises each component to a given power.
     *
     * @param exp The exponent
     * @return A new Vec2 with each component raised to the power
     */
    public Vec2 powi(int exp) {
        return new Vec2(Math.pow(this.x, exp), Math.pow(this.y, exp));
    }

    /**
     * Divides this vector by a scalar value.
     *
     * @param scalar The scalar divisor
     * @return A new Vec2 with both components divided by the scalar
     */
    public Vec2 div(double scalar) {
        return new Vec2(this.x / scalar, this.y / scalar);
    }

    /**
     * Performs element-wise division with another vector.
     *
     * @param elementwise The vector to divide element-wise
     * @return A new Vec2 with each component divided (x/x, y/y)
     */
    public Vec2 div(Vec2 elementwise) {
        return new Vec2(this.x / elementwise.x, this.y / elementwise.y);
    }

    /**
     * Computes the dot product with another vector.
     *
     * The dot product represents the projection of one vector onto another.
     * For perpendicular vectors, the dot product is zero.
     *
     * @param other The other vector
     * @return The dot product as a scalar (this · other)
     */
    public double dot(Vec2 other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Returns the normalized (unit) version of this vector.
     *
     * The normalized vector has a magnitude of 1 but points in the same direction.
     * For zero vectors, returns a zero vector to avoid division by zero.
     *
     * @return A new Vec2 with the same direction but magnitude of 1
     */
    public Vec2 norm() {
        double len = mag();
        if (len == 0) return new Vec2(0, 0);
        return new Vec2(this.x / len, this.y / len);
    }

    /**
     * Calculates the magnitude (length) of this vector.
     *
     * Computed as sqrt(x² + y²).
     *
     * @return The magnitude of this vector
     */
    public double mag() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Calculates the squared magnitude of this vector.
     *
     * Computed as x² + y². This is more efficient than mag() when you only
     * need to compare magnitudes, since comparing squares preserves the order.
     *
     * @return The squared magnitude of this vector
     */
    public double mag2() {
        return x * x + y * y;
    }

    /**
     * Applies a function to both components of this vector.
     *
     * @param f The function to apply to each component
     * @return A new Vec2 with f applied to both x and y
     */
    public Vec2 apply(Function<Double, Double> f) {
        return new Vec2(f.apply(this.x), f.apply(this.y));
    }

    /**
     * Applies separate functions to each component of this vector.
     *
     * @param x The function to apply to the x component
     * @param y The function to apply to the y component
     * @return A new Vec2 with the functions applied to each component
     */
    public Vec2 apply(Function<Double, Double> x, Function<Double, Double> y) {
        return new Vec2(x.apply(this.x), y.apply(this.y));
    }

    /**
     * Returns the sign of each component.
     *
     * Each component becomes -1 (negative), 0 (zero), or 1 (positive).
     *
     * @return A new Vec2 with the sign of each component
     */
    public Vec2 sign() {
        return new Vec2(Math.signum(this.x), Math.signum(this.y));
    }

    public Vec2 copy() {
        return new Vec2(this.x, this.y);
    }
}
