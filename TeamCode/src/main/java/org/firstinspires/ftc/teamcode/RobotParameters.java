package org.firstinspires.ftc.teamcode;

public class RobotParameters {
	// maximal movement
	public static double MAX_V_Y = 153.19296; // [cm/s]
	public static double MAX_A_Y = 100; // [cm/s^2]

	public static double MAX_V_X = 136.17152; // [cm/s]
	public static double MAX_A_X = 100; // [cm/s^2]

	public static final double DRIVE_Y_FACTOR = RobotParameters.MAX_V_X / RobotParameters.MAX_V_Y;

	// constants used to follow path
	public static final double k_a_accelerating = 1. / 500.;
	public static final double k_a_decelerating = 1. / 1000.;
	public static final double k_error = 0;
	public static final double k_d_error = 0;
	//	private static final double k_error = 0.1;
	//	private static final double k_d_error = 0.005;
	public static final double k_v = 1 / MAX_V_X;



	// constants used for rotation
	public static double WHEEL_DISTANCE_X = 31.5; // [cm]
	public static double WHEEL_DISTANCE_Y = 27; // [cm]

	public static double ROT_MOVE_SCALAR = 105./120;

	public static double WHEEL_CENTER_DISTANCE = Math.hypot(WHEEL_DISTANCE_X, WHEEL_DISTANCE_Y) / 2;
	public static double ADJUSTED_WHEEL_CENTER_DISTANCE = WHEEL_CENTER_DISTANCE * ROT_MOVE_SCALAR;

	public static double MAX_V_ROT = MAX_V_Y / ADJUSTED_WHEEL_CENTER_DISTANCE;
	public static double MAX_A_ROT = MAX_A_Y / ADJUSTED_WHEEL_CENTER_DISTANCE;
}
