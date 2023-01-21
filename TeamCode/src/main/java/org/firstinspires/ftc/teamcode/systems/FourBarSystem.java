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
		PICKUP(0.56), DROPOFF(0.46);

		private final double state;

		FourBarState(double state) {
			this.state = state;
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
	private final Servo servo1;
	/**
	 * The second of the two servos which controls the fourBar.
	 */
	private final Servo servo2;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(OpMode opMode) {
		servo1 = opMode.hardwareMap.get(Servo.class, "4bar_right");
		servo2 = opMode.hardwareMap.get(Servo.class, "4bar_left");
		servo1.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Sets the fourBar to the specified state.
	 *
	 * @param state The state to move the fourBar to.
	 */
	public void goTo(FourBarState state) {
		goTo(state.state);
	}

	/**
	 * A temporary function for setting the fourBar to a specified position.
	 *
	 * @param state The state to move the fourBar to.
	 */
	public void goTo(double state) {
		double factor = 1; // 0.85
		servo1.setPosition(state);
		servo2.setPosition(factor * state);
	}
}
