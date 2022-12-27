package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBarSystem {
	/**
	 * Enum encapsulating the two positions the system should reach.
	 */
	public enum FourBarState {
		PICKUP(0.56), DROPOFF(0.123);

		private final double state;

		FourBarState(double state) {
			this.state = state;
		}
	}

	private final Servo fourBar;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(LinearOpMode opMode) {
		fourBar = opMode.hardwareMap.get(Servo.class, "4bar");
	}

	/**
	 * Sets the fourBar to the specified state.
	 *
	 * @param state The level to move the elevator to.
	 */
	public void goTo(FourBarState state) {
		fourBar.setPosition(state.state);
	}
}
