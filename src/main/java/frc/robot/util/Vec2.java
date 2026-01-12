package frc.robot.util;

import java.util.function.Function;

public class Vec2 {
	public final double x;
	public final double y;

    public static final Vec2 ZERO = new Vec2(0, 0);
    public static final Vec2 ONE = new Vec2(1, 1);
    public static final Vec2 UNIT_X = new Vec2(1, 0);
    public static final Vec2 UNIT_Y = new Vec2(0, 1);

	public Vec2(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public static Vec2 fill(double value) {
		return new Vec2(value, value);
	}

    public Vec2 add(Vec2 other) {
        return new Vec2(this.x + other.x, this.y + other.y);
    }

    public Vec2 sub(Vec2 other) {
        return new Vec2(this.x - other.x, this.y - other.y);
    }

    public Vec2 mul(double scalar) {
        return new Vec2(this.x * scalar, this.y * scalar);
    }

    public Vec2 mul(Vec2 elementwise) {
        return new Vec2(this.x * elementwise.x, this.y * elementwise.y);
    }

    public Vec2 powi(int exp) {
        return new Vec2(Math.pow(this.x, exp), Math.pow(this.y, exp));
    }

    public Vec2 div(double scalar) {
        return new Vec2(this.x / scalar, this.y / scalar);
    }

    public Vec2 div(Vec2 elementwise) {
        return new Vec2(this.x / elementwise.x, this.y / elementwise.y);
    }

    public double dot(Vec2 other) {
        return this.x * other.x + this.y * other.y;
    }

    public Vec2 norm() {
        double len = mag();
        if (len == 0) return new Vec2(0, 0);
        return new Vec2(this.x / len, this.y / len);
    }

    public double mag() {
        return Math.sqrt(x * x + y * y);
    }

    public double mag2() {
        return x * x + y * y;
    }

    public Vec2 apply(Function<Double, Double> f) {
        return new Vec2(f.apply(this.x), f.apply(this.y));
    }

    public Vec2 apply(Function<Double, Double> x, Function<Double, Double> y) {
        return new Vec2(x.apply(this.x), y.apply(this.y));
    }

    public Vec2 sign() {
        return new Vec2(Math.signum(this.x), Math.signum(this.y));
    }

    public Vec2 copy() {
        return new Vec2(this.x, this.y);
    }
}
