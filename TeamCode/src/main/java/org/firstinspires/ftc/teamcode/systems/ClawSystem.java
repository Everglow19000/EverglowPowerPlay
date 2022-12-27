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
		OPEN(0.36, 0.3),
		CLOSED(0.3, 0.26);

		public final double claw1State;
		public final double claw2State;

		ClawState(double claw1State, double claw2State) {
			this.claw1State = claw1State;
			this.claw2State = claw2State;
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

	private final Servo claw1;
	private final Servo claw2;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public ClawSystem(LinearOpMode opMode) {
		claw1 = opMode.hardwareMap.get(Servo.class, "claw1");
		claw2 = opMode.hardwareMap.get(Servo.class, "claw2");
		claw2.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Sets the claw to the specified state.
	 *
	 * @param state The state to set the claw to (open or closed).
	 */
	public void goTo(ClawState state) {
		claw1.setPosition(state.claw1State);
		claw2.setPosition(state.claw2State);
	}
}
