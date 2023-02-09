package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.StateMachine.RestingState;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;
import org.firstinspires.ftc.teamcode.utils.StateMachine.State;
import org.firstinspires.ftc.teamcode.utils.StateMachine.StateMessages;

/**
 * A class for handling the fourBar system.
 */
public class FourBarSystem {
	/**
	 * The fourBar motor.
	 */
	private final DcMotor fourBar;
	/**
	 * The current state of the fourBar system.
	 */
	private State state = new RestingState();

	/**
	 * Enum encapsulating all the positions the system should reach.
	 */
	public enum Position {
		//START(170), PICKUP(65), DROPOFF(-170);
		BACK(30), START(0), PICKUP(-75), DROPOFF(-325);

		public final int desiredPosition;

		Position(int desiredPosition) {
			this.desiredPosition = desiredPosition;
		}
	}

	/**
	 * A state used when the fourBar should be moving.
	 */
	public class ActingState implements State {
		private static final int EPSILON = 20;
		private final Position position;

		/**
		 * @param position A fourBar position to move to (FourBarPosition.PICKUP or FourBarPosition.DROPOFF).
		 */
		public ActingState(Position position) {
			this.position = position;
			fourBar.setTargetPosition(position.desiredPosition);
		}

		public void tick() {
			// The fourBar has reached its desired position
			int error = abs(position.desiredPosition - fourBar.getCurrentPosition());
			if (error <= EPSILON) {
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(StateMessages.FOURBAR_DONE);
			}
		}

	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(OpMode opMode) {
		fourBar = opMode.hardwareMap.get(DcMotor.class, "4bar");

		fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		fourBar.setTargetPosition(0);
		fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		fourBar.setPower(0.5);
	}

	public Sequence.SequenceItem goToSequenceItem(Position position) {
		return new Sequence.SequenceItem(StateMessages.FOURBAR_DONE, () -> {
			state = new ActingState(position);
		});
	}

	// should only be used for testing
	public void goToImmediate(Position position) {
		fourBar.setTargetPosition(position.desiredPosition);
	}

	/**
	 * Ticks the fourBar system.
	 */
	public void tick() {
		state.tick();
	}

	/**
	 * Stops the fourBar system.
	 */
	public void interrupt() {
		state = new RestingState();
		fourBar.setTargetPosition(fourBar.getCurrentPosition());
	}
}
