package org.firstinspires.ftc.teamcode.utils;

/**
 * Used to control power activations while in track.
 */
public class PIDController {
	private final double Kp, Ki, Kd; // scalars that are all positives
	private double deviationIntegral = 0;
	private double lastTime, lastDeviation = 0;

	public PIDController(double Kp, double Ki, double Kd) {
		lastTime = System.nanoTime();
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
	}

	public double powerByDeviation(double deviation) {
		final double currentTime = System.nanoTime();

		final double deltaTimeS = (currentTime - lastTime) / 1e9;
		final double derivative = (deviation - lastDeviation) / deltaTimeS;
		deviationIntegral += deltaTimeS * (deviation + lastDeviation) / 2;

		lastTime = currentTime;
		lastDeviation = deviation;

		return Kp * deviation + deviationIntegral * Ki + Kd * derivative;
	}
}
