package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;

import android.annotation.SuppressLint;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Utils {
	/**
	 * @return The current time, formatted as a string with milliseconds.
	 */
	@SuppressLint("SimpleDateFormat")
	public static String timestampString() {
		return new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss_SSS").format(new Date());
	}

	/**
	 * Given any angle, normalizes it such that it is between PI and PI radians,
	 * increasing or decreasing by 2PI radians to make it so.
	 *
	 * @param angle Given angle in radians.
	 * @return The angle normalized (-PI < angle < PI).
	 */
	public static double normalizeAngle(double angle) {
		while (angle >= PI) angle -= 2.0 * PI;
		while (angle < -PI) angle += 2.0 * PI;
		return angle;
	}
}
