package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Sequence;
import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;

/**
 * A Class for handling the claw system.
 */
public class ClawSystem {
	private final Servo claw;
	private State state = new RestingState();

	/**
	 * Enum encapsulating the two positions the system should reach.
	 */
	public enum ClawPosition {
		OPEN(0.6),
		CLOSED(0.78);

		public final double desiredPosition;

		ClawPosition(double desiredPosition) {
			this.desiredPosition = desiredPosition;
		}

		/*
		 * Switches the state of the claw from open to closed or vice versa.
		 */
		public ClawPosition flip() {
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
	 * A state used when the claw should be moving.
	 */
	public class ActingState implements State {
		private final double totalMovementTime;
		private final double startPosition;
		private final ElapsedTime timer;
		private final double velocity;
		private final ClawPosition finalState;

		/**
		 * @param finalState A claw finalState to move to (ClawState.OPEN or ClawState.CLOSED)
		 */
		public ActingState(ClawPosition finalState, double velocity) {
			this.finalState = finalState;
			this.startPosition = claw.getPosition();
			double deviation = finalState.desiredPosition - startPosition;
			this.totalMovementTime = abs(deviation) / velocity;
			this.velocity = velocity * signum(deviation);
			this.timer = new ElapsedTime();
		}

		public void tick() {
			// The claw has reached its desired position
			if (timer.time() > totalMovementTime) {
				claw.setPosition(finalState.desiredPosition);
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.CLAW_DONE);
				return;
			}

			// Otherwise, update the claw position
			claw.setPosition(startPosition + velocity * timer.time());
		}
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public ClawSystem(LinearOpMode opMode) {
		claw = opMode.hardwareMap.get(Servo.class, "claw");
	}

	public Sequence.SequenceItem goToSequenceItem(ClawPosition position, double velocity) {
		return new Sequence.SequenceItem(State.Message.CLAW_DONE, () -> {
			state = new ActingState(position, velocity);
		});
	}

	/**
	 * Goes to the specified position immediately, without relying on the state machine.
	 * Should only be used for testing.
	 *
	 * @param position the position to go to.
	 */
	public void goToImmediate(ClawPosition position) {
		claw.setPosition(position.desiredPosition);
	}

	/**
	 * Ticks the claw system.
	 */
	public void tick() {
		state.tick();
	}

	/**
	 * Stops the claw system.
	 */
	public void interrupt(){
		state = new RestingState();
	}

}