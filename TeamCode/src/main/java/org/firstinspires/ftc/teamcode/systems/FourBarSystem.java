package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBarSystem {
	private final Servo servo1;
	private final Servo servo2;
	private State state;

	/**
	 * A state used when the robot should be moving.
	 */
	public class ActingState implements State {
		private final double totalMovementTime;
		private final double startPosition;
		private final ElapsedTime timer;
		private final double velocity;

		/**
		 * @param state             A fourBar state to move to (FourBarState.OPEN or FourBarState.CLOSED).
		 * @param totalMovementTime The total time the movement should take.
		 */
		public ActingState(FourBarState state, double totalMovementTime) {
			this(state.desiredPosition, totalMovementTime);
		}

		/**
		 * @param desiredPosition   The desired position the fourBar should move to, between 0 and 1.
		 * @param totalMovementTime The total time the movement should take.
		 */
		public ActingState(double desiredPosition, double totalMovementTime) {
			this.totalMovementTime = totalMovementTime;
			this.startPosition = (servo1.getPosition() + servo2.getPosition()) / 2; // average of the two servos
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
			servo1.setPosition(startPosition + velocity * timer.time());
			servo2.setPosition(startPosition + velocity * timer.time());
		}

		public void onReceiveMessage(State.Message message) {
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
		servo1 = opMode.hardwareMap.get(Servo.class, "4bar_left");
		servo2 = opMode.hardwareMap.get(Servo.class, "4bar_right");
		servo2.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Sets the fourBar to the specified state.
	 *
	 * @param state        The state to set the claw to (pickup or dropoff).
	 * @param movementTime The time it should take the fourBar to reach the desired position.
	 */
	public void goTo(FourBarState state, double movementTime) {
		goTo(state.desiredPosition, movementTime);
	}
	public void goTo(double state, double movementTime) {
		this.state = new ActingState(state, movementTime);
	}

	/**
	 * Sets the fourBar to the specified state.
	 * The time to reach the position is static at 0.5 seconds.
	 *
	 * @param state The state to set the fourBar to (FourBarState.PICKUP or FourBarState.DROPOFF).
	 */
	public void goTo(FourBarState state) {
		goTo(state, 0.5);
	}
	public void goTo(double state) {
		goTo(state, 0.5);
	}

	/**
	 * Ticks the fourBar system.
	 */
	public void tick() {
		state.tick();
	}

	/**
	 * Receives a message from all the other classes.
	 */
	public void receiveMessage() {
		state.onReceiveMessage();
	}
}
