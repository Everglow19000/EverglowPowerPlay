package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;

public class RobotParameters {
	// Maximal movement
	public static double MAX_V_Y = 153.19296; // [cm/s]
	public static double MAX_A_Y = 100; // [cm/s^2]

	public static double MAX_V_X = 136.17152; // [cm/s]
	public static double MAX_A_X = 100; // [cm/s^2]

	public static final double DRIVE_Y_FACTOR = RobotParameters.MAX_V_X / RobotParameters.MAX_V_Y;

	// Constants used to follow path
	public static final double k_a_accelerating = 1. / 500.;
	public static final double k_a_decelerating = 1. / 1000.;
	public static final double k_error = 0;
	public static final double k_d_error = 0;
	//	private static final double k_error = 0.1;
	//	private static final double k_d_error = 0.005;
	public static final double k_v = 1 / MAX_V_X;


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
	public static final double LATERAL_DISTANCE = 13.5;
	/**
	 * The distance between the center of the robot and the back odometry wheel, in centimeters.
	 */
	public static final double FORWARD_OFFSET = -5.0;
	/**
	 * The size of a tile side, in centimeters.
	 */
	public static final double TILE_SIZE = 71;
	/**
	 * The distance from which the center robot drops the cones onto the polls, in centimeters.
	 */
	public static final double DROP_OFF_DISTANCE = 25;
}
