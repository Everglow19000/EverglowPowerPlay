package org.firstinspires.ftc.teamcode.utils;

/**
 * A class that represents a point in 2D space.
 */
public class Point2D {
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
	public Point2D() {
		this.x = 0;
		this.y = 0;
	}

	/**
	 * @param x position on the sideways-axis.
	 * @param y position on the forward-axis.
	 */
	public Point2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public static Point2D difference(Point2D p1, Point2D p2){
		return new Point2D(p1.x - p2.x, p1.y - p2.y);
	}

}
