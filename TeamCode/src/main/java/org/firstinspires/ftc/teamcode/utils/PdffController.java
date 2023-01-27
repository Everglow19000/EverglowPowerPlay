package org.firstinspires.ftc.teamcode.utils;

public class PdffController {
	private final double k_v;
	private final double k_a;
	private final double k_error;
	private final double k_d_error;

	private double prevT = 0;
	private double prevError = 0;

	public PdffController(double k_v, double k_a, double k_error, double k_d_error) {
		this.k_v = k_v;
		this.k_a = k_a;
		this.k_error = k_error;
		this.k_d_error = k_d_error;
	}

	public double getPower(double t, double error, double v, double a){
		double dt = prevT - t;
		double d_error_dt = (error - prevError) / dt;

		prevT = t;
		return k_v * v + k_a * a + k_error * error + k_d_error + d_error_dt;
	}
}
