package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.State;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBarSystem {
	private final Servo fourBar;
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
		 * @param state A fourBar state to move to (FourBarState.OPEN or FourBarState.CLOSED).
		 * @param totalMovementTime The total time the movement should take.
		 */
		public GoToPositionState(FourBarState state, double totalMovementTime) {
			this(state.desiredPosition, totalMovementTime);
		}

		/**
		 * @param desiredPosition The desired position the fourBar should move to, between 0 and 1.
		 * @param totalMovementTime The total time the movement should take.
		 */
		public GoToPositionState(double desiredPosition, double totalMovementTime) {
			this.totalMovementTime = totalMovementTime;
			this.startPosition = fourBar.getPosition();
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
			fourBar.setPosition(startPosition + velocity * timer.time());
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
	public enum FourBarState {
		PICKUP(0.56), DROPOFF(0.123);

		private final double desiredPosition;

		FourBarState(double desiredPosition) {
			this.desiredPosition = desiredPosition;
		}
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(LinearOpMode opMode) {
		fourBar = opMode.hardwareMap.get(Servo.class, "4bar");
	}

	/**
	 * Sets the fourBar to the specified state.
	 *
	 * @param state The level to move the fourBar to.
	 * @param movementTime The time it should take the fourBar to reach the desired position.
	 */
	public void goTo(FourBarState state, double movementTime) {
		this.state = new GoToPositionState(state, movementTime);
	}

	/**
	 * Sets the fourBar to the specified state, without specifying total movement time.
	 *
	 * @param state The state to set the fourBar to (FourBarState.PICKUP or FourBarState.DROPOFF).
	 */
	public void goTo(FourBarState state) {
		goTo(state, 0.5);
	}
}
