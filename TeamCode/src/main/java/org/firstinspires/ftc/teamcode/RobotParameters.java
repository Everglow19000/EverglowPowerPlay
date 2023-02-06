package org.firstinspires.ftc.teamcode;

public class RobotParameters {

	public static double V_SCALE =  122./150 * 120. / 150 * 4 / 3 * 1.09 * 1.2;

	public static double MAX_V_Y = 135 * V_SCALE; // [cm/s]
	public static double MAX_A_Y = 100; // [cm/s^2]

	public static double MAX_V_X = 120 * V_SCALE; // [cm/s]
	public static double MAX_A_X = 100; // [cm/s^2]

	public static double WHEEL_DISTANCE_X = 31.5; // [cm]
	public static double WHEEL_DISTANCE_Y = 27; // [cm]

	public static double ROT_MOVE_SCALAR = 105./120;

	public static double WHEEL_CENTER_DISTANCE = Math.hypot(WHEEL_DISTANCE_X, WHEEL_DISTANCE_Y) / 2;
	public static double ADJUSTED_WHEEL_CENTER_DISTANCE = WHEEL_CENTER_DISTANCE * ROT_MOVE_SCALAR;

	public static double MAX_V_ROT = MAX_V_Y / ADJUSTED_WHEEL_CENTER_DISTANCE;
	public static double MAX_A_ROT = MAX_A_Y / ADJUSTED_WHEEL_CENTER_DISTANCE;
}
