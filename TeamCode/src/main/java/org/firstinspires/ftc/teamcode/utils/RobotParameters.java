package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;

public class RobotParameters {

	public static double ODOMETRY_SCALAR = 154./150;

	// Maximal movement
	public static double MAX_V_Y = 153.19296 * ODOMETRY_SCALAR; // [cm/s]
	public static double MAX_A_Y = 100; // [cm/s^2]

	public static double MAX_V_X = 136.17152 * ODOMETRY_SCALAR; // [cm/s]
	public static double MAX_A_X = 100; // [cm/s^2]

	public static final double DRIVE_Y_FACTOR = MAX_V_X / MAX_V_Y;

	// Constants used to follow path
	public static final double k_a_accelerating = 1. / 500.;
	public static final double k_a_decelerating = 1. / 1500.;
//	public static final double k_error = 0;
//	public static final double k_d_error = 0;
	public static final double k_error = 0.1;
	public static final double k_d_error = 0.005;
	public static final double k_v_y = 1 / MAX_V_X;
	public static final double k_v_x = 1 / MAX_V_X * 1.5 * 150./140;


	// Constants used for rotation
	public static double WHEEL_DISTANCE_X = 31.5; // [cm]
	public static double WHEEL_DISTANCE_Y = 27; // [cm]

	public static double ROT_MOVE_SCALAR = 105. / 120;

	public static double WHEEL_CENTER_DISTANCE = Math.hypot(WHEEL_DISTANCE_X, WHEEL_DISTANCE_Y) / 2;
	public static double ADJUSTED_WHEEL_CENTER_DISTANCE = WHEEL_CENTER_DISTANCE * ROT_MOVE_SCALAR;

	public static double MAX_V_ROT = MAX_V_Y / ADJUSTED_WHEEL_CENTER_DISTANCE;
	public static double MAX_A_ROT = MAX_A_Y / ADJUSTED_WHEEL_CENTER_DISTANCE;


	/**
	 * The radius of the odometry wheels, in centimeters.
	 */
	private static final double WHEEL_RADIUS = 2.5;
	/**
	 * The number of ticks per full revolution of the odometry wheels.
	 */
	private static final double TICKS_PER_ROTATION = 8192;
	/**
	 * Conversion factor from ticks to centimeters.
	 */
	public static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS * 2 * PI;
	/**
	 * The distance between the two front odometry wheels, in centimeters.
	 */
	public static final double LATERAL_DISTANCE = 13.5 * 1840./1801 * 1783/1801;
	/**
	 * The distance between the center of the robot and the back odometry wheel, in centimeters.
	 */
	public static final double FORWARD_OFFSET = -5.0;
	/**
	 * The size of a tile side, in centimeters.
	 */
	public static final double TILE_SIZE = 60;
	/**
	 * The distance from which the center robot drops the cones onto the polls, in centimeters.
	 */
	public static final double DROP_OFF_DISTANCE = 25;
}
