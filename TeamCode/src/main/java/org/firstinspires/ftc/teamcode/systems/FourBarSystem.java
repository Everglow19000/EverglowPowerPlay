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

		/*
		 * Toggles the state of the fourbar
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

	private final Servo servo1;
	private final Servo servo2;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(LinearOpMode opMode) {
		servo1 = opMode.hardwareMap.get(Servo.class, "4bar_right");
		servo2 = opMode.hardwareMap.get(Servo.class, "4bar_left");
		servo2.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Sets the fourBar to the specified state.
	 *
	 * @param state The level to move the elevator to.
	 */
	public void goTo(FourBarState state) {
		double factor = 0.85;
		servo1.setPosition(state.state);
		servo2.setPosition(state.state*factor);
	}

	public void goTo(double state) {
		double factor = 0.85;
		servo1.setPosition(state);
		servo2.setPosition(factor*state);
	}
}
