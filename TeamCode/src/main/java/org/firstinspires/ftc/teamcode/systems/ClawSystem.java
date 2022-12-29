package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.State;

/**
 * A Class for handling the claw system.
 */
public class ClawSystem {
	private final Servo claw;
	public State state;

	/**
	 * A state used when the robot should be moving.
	 */
	public class GoToPositionState implements State {
		private final double totalMovementTime;
		private final double startPosition;
		private final ElapsedTime timer;
		private final double velocity;

		/**
		 * @param state A claw state to move to (ClawState.OPEN or ClawState.CLOSED)
		 * @param totalMovementTime The total time the movement should take.
		 */
		public GoToPositionState(ClawState state, double totalMovementTime) {
			this(state.desiredPosition, totalMovementTime);
		}

		/**
		 * @param desiredPosition The desired position the claw should move to, between 0 and 1.
		 * @param totalMovementTime The total time the movement should take.
		 */
		public GoToPositionState(double desiredPosition, double totalMovementTime) {
			this.totalMovementTime = totalMovementTime;
			this.startPosition = claw.getPosition();
			this.timer = new ElapsedTime();
			// Calculate position change per tick
			this.velocity = (desiredPosition - startPosition) / (totalMovementTime);
		}

		public void tick() {
			// The claw has reached its desired position
			if (timer.time() > totalMovementTime) {
				state = new RestingState();
				return;
			}

			// Otherwise, update the claw position
			claw.setPosition(startPosition + velocity * timer.time());
		}
	}

	/**
	 * A state used when the robot should not be moving.
	 */
	public class RestingState implements State {
		public void tick() {
			// Do nothing
		}
	}

	/**
	 * Enum encapsulating the two positions the system should reach.
	 */
	public enum ClawState {
		OPEN(0.36),
		CLOSED(0.3);

		public final double desiredPosition;

		ClawState(double desiredPosition) {
			this.desiredPosition = desiredPosition;
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

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public ClawSystem(LinearOpMode opMode) {
		claw = opMode.hardwareMap.get(Servo.class, "claw");
	}

	/**
	 * Sets the claw to the specified state.
	 *
	 * @param state The state to set the claw to (open or closed).
	 * @param movementTime The time it should take the claw to reach the desired position.
	 */
	public void goTo(ClawState state, double movementTime) {
		this.state = new GoToPositionState(state, movementTime);
	}

	/**
	 * Sets the claw to the specified state, without specifying total movement time.
	 *
	 * @param state The state to set the claw to (ClaState.OPEN or ClawState.CLOSED).
	 */
	public void goTo(ClawState state) {
		goTo(state, 0.5);
	}
}
