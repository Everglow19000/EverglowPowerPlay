package org.firstinspires.ftc.teamcode.utils;

/**
 * A class that represents a point in 2D space.
 */
public class PointD {
	/**
	 * Position on the sideways-axis.
	 */
	public double x;
	/**
	 * Position on the forward-axis.
	 */
	public double y;

	/**
	 * Given no values, sets the properties' values to 0.
	 */
	public PointD() {
		this.x = 0;
		this.y = 0;
	}

	/**
	 * @param x position on the sideways-axis.
	 * @param y position on the forward-axis.
	 */
	public PointD(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public static PointD difference(PointD p1, PointD p2) {
		return new PointD(p1.x - p2.x, p1.y - p2.y);
	}
}
