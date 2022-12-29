package org.firstinspires.ftc.teamcode.utils;

/**
 * A class that represents a point in 2D space.
 */
public class Point2D {
	public double x;
	public double y;

	/**
	 * Given no values, sets the properties' values to 0.
	 */
	public Point2D() {
		this.x = 0;
		this.y = 0;
	}

	public Point2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * @return sqrt(x ^ 2 + y ^ 2).
	 */
	public double hyp() {
		return Math.hypot(x, y);
	}
}
