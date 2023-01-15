package org.firstinspires.ftc.teamcode;

public class RobotParameters {

        public static double V_SCALE = 120./150 * 4/3 * 1.09 * 1.2;

    public static double MAX_V_Y = 135 * V_SCALE; // [cm/s]
    public static double MAX_A_Y = 100; // [cm/s^2]

    public static double MAX_V_X = 120 * V_SCALE; // [cm/s]
    public static double MAX_A_X = 100; // [cm/s^2]

    public static double MAX_V_ROT = Math.toRadians(225); // [radians/s]
    public static double MAX_A_ROT = Math.toRadians(200); // [radians/s^2]

    public static double WHEEL_DISTANCE_X = 31.5; // [cm]
    public static double WHEEL_DISTANCE_Y = 27; // [cm]


}
