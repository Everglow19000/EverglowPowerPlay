package org.firstinspires.ftc.teamcode.utils;

public class MathUtils {
	public static double floatingPrecision(double num, int digits) {
		return Math.round(num*10*digits)/(10*digits*1.0);
	}
}
