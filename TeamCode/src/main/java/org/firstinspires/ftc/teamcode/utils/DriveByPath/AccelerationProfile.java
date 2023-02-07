package org.firstinspires.ftc.teamcode.utils.DriveByPath;

/**
 * A class that creates an acceleration profile which controls how the robot accelerates and decelerates.
 * It's used in order to minimize drift in sharp curves.
 */
public class AccelerationProfile {
	private final double a;
	private final double vMax;
	private final double t1, t2; // The end time of the first & second section, respectively.
	private final double tEnd; // The total end time
	private final double x1, x2; // The end position of the first & second section, respectively.
	private final double d;
	// true --> the robot gets to max speed); false --> the robot doesn't get to max speed.
	private final boolean reachMaxSpeedEh;

	public AccelerationProfile(double a, double vMax, double d) {
		this.a = a;
		this.vMax = vMax;
		this.d = d;

		this.reachMaxSpeedEh = (d / 2 > Math.pow(vMax, 2) / a);
		if (reachMaxSpeedEh) {
			this.t1 = vMax / a;
			this.x1 = 0.5 * a * Math.pow(t1, 2);
			this.x2 = d - 2 * x1;
			this.t2 = x2 / vMax;
			this.tEnd = 2 * t1 + t2;
		} else {
			this.t1 = Math.sqrt(d / a);
			this.t2 = t1;
			this.tEnd = 2 * t1;
			this.x1 = d / 2;
			this.x2 = x1;
		}

	}

	public double acceleration(double t) {
		if (reachMaxSpeedEh) {
			if (t < t1)
				return a;
			else if (t < t1 + t2)
				return 0;
			else if (t < tEnd)
				return -a;
			else
				return 0;
		}
		if (t < t1)
			return a;
		else if (t < tEnd)
			return -a;
		else
			return 0;
	}

	public double getVelocity(double t) {
		if (reachMaxSpeedEh) {
			if (t < t1)
				return a * t;
			else if (t < t1 + t2)
				return vMax;
			else if (t < tEnd)
				return vMax - a * (t - t2 - t1);
			else
				return 0;
		}
		if (t < t1)
			return a * t;
		else if (t < tEnd)
			return a * t1 - a * (t - t1);
		else
			return 0;
	}

	public double getPosition(double t) {
		if (reachMaxSpeedEh) {
			if (t < t1)
				return 0.5 * a * Math.pow(t, 2);
			else if (t >= t1 && t < t1 + t2)
				return vMax * (t - t1) + x1;
			else if (t < tEnd)
				return -0.5 * a * Math.pow(t - t2 - t1, 2) + vMax * (t - t2 - t1) + x1 + x2;
			else
				return d;
		}
		if (t < t1)
			return 0.5 * a * Math.pow(t, 2);
		else if (t < tEnd)
			return x1 + a * t1 * (t - t1) - 0.5 * a * Math.pow(t - t1, 2);
		else
			return d;
	}

	public double finalTime() {
		return tEnd;
	}
}
