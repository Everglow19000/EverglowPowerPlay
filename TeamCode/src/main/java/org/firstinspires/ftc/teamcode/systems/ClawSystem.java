package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A Class for handling the claw system.
 */
public class ClawSystem {
	/**
	 * Enum encapsulating the two positions the system should reach.
	 */
	public enum ClawState {
		OPEN(0.37),
		CLOSED(0.6);

		public final double clawState;


		ClawState(double clawState) {
			this.clawState = clawState;
		}

		/*
		* Switches the state of the claw from open to closed or vice versa.
		*/
		public ClawState flip() {
			switch (this) {
				case OPEN:
					return CLOSED;
				case CLOSED:
					return OPEN;
				default:
					throw new IllegalStateException();
			}
		}
	}

	private final Servo claw;


	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public ClawSystem(LinearOpMode opMode) {
		claw = opMode.hardwareMap.get(Servo.class, "claw");
		claw.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Sets the claw to the specified state.
	 *
	 * @param state The state to set the claw to (open or closed).
	 */
	public void goTo(ClawState state) {
		claw.setPosition(state.clawState);
	}
}
