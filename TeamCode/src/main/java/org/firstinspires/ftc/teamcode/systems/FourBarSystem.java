package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBarSystem {
	/**
	 * Enum encapsulating the two positions the fourBar should reach.
	 */
	public enum FourBarState {
		PICKUP(0.1, 0.), DROPOFF(0.6, 0.5);

		private final double posRight;
		private final double posLeft;

		FourBarState(double posRight, double state2) {
			this.posRight = posRight;
			this.posLeft = state2;
		}

		/*
		 * Toggles the state of the fourBar
		 */
		public FourBarState toggle() {
			switch (this) {
				case PICKUP:
					return DROPOFF;
				case DROPOFF:
					return PICKUP;
				default:
					throw new IllegalStateException();
			}
		}
	}

	/**
	 * The first of the two servos which controls the fourBar.
	 */
	private final Servo servoRight;
	/**
	 * The second of the two servos which controls the fourBar.
	 */
	private final Servo servoLeft;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(OpMode opMode) {
		servoRight = opMode.hardwareMap.get(Servo.class, "4bar_right");
		servoLeft = opMode.hardwareMap.get(Servo.class, "4bar_left");
		servoRight.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Sets the fourBar to the specified state.
	 *
	 * @param state The state to move the fourBar to.
	 */
	public void goTo(FourBarState state) {
		servoLeft.setPosition(state.posLeft);
		servoRight.setPosition(state.posRight);
	}
}
