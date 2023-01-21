package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class for handling the grabbing wheel.
 */
public class GWheelSystem {
	/**
	 * The motor which controls the grabbing wheel.
	 */
	private final DcMotor gWheel;

	/**
	 * The power at which the motor for grabbing wheel should spin.
	 */
	public static double GWHEEL_POWER = 0.65;

	// These variables control whether the grabbing wheel is running or not.
	private boolean isCollecting = false;
	private boolean isSpitting = false;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public GWheelSystem(OpMode opMode) {
		gWheel = opMode.hardwareMap.get(DcMotor.class, "gWheel");
		gWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	/**
	 * A function which spins te grabbing wheel to inward to collect the cones.
	 * Used in toggleCollecting().
	 */
	private void collect() {
		gWheel.setPower(GWHEEL_POWER);
		isCollecting = true;
	}

	/**
	 * A function which spins the grabbing wheel outward to spit the cones.
	 * Used in toggleSpitting().
	 */
	private void spit() {
		gWheel.setPower(-GWHEEL_POWER);
		isSpitting = true;
	}

	/**
	 * A function which stops the grabbing wheel and resets the isCollecting and isSpitting variables.
	 * Used in toggleCollecting() and toggleSpitting().
	 */
	private void stop() {
		gWheel.setPower(0);
		isSpitting = false;
		isCollecting = false;
	}

	/**
	 * A function which collects cones using the grabbing wheel.
	 */
	public void toggleCollect() {
		if (isCollecting) {
			stop();
		} else {
			collect();
		}
	}

	/**
	 * A function which ejects cones using the grabbing wheel.
	 */
	public void toggleSpit() {
		if (isSpitting) {
			stop();
		} else {
			spit();
		}
	}

	/**
	 * A temporary function for testing the grabbing wheel.
	 *
	 * @param power The power at which the grabbing wheel should spin.
	 */
	public void setPower(double power){
		gWheel.setPower(power);
	}
}
