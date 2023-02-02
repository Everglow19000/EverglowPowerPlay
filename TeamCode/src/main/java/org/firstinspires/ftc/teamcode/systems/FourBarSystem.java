package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.RestingState;
import org.firstinspires.ftc.teamcode.utils.Sequence;
import org.firstinspires.ftc.teamcode.utils.State;

/**
 * A class for handling the fourBar system.
 */
public class FourBarSystem {
	private final DcMotor fourBar;
	private State state = new RestingState();
	/**
	 * Enum encapsulating all the positions the system should reach.
	 */
	public enum FourBarPosition {
		PICKUP(0), DROPOFF(-500);

		public final int desiredPosition;

		FourBarPosition(int desiredPosition) {
			this.desiredPosition = desiredPosition;
		}
	}

	/**
	 * A state used when the fourBar should be moving.
	 */
	public class ActingState implements State {
		private static final int EPSILON = 20;
		private final FourBarPosition position;

		/**
		 * @param position A fourBar position to move to (FourBarPosition.PICKUP or FourBarPosition.DROPOFF).
		 */
		public ActingState(FourBarPosition position) {
			this.position = position;
			fourBar.setTargetPosition(position.desiredPosition);
		}

		public void tick() {
			// The fourBar has reached its desired position
			int error = abs(position.desiredPosition - fourBar.getCurrentPosition());
			SystemCoordinator.instance.opMode.telemetry.addData("error", error);
			SystemCoordinator.instance.opMode.telemetry.update();
			if (error <= EPSILON) {
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.ELEVATOR_DONE);
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
		fourBar.setPower(0.7);
	}

	public Sequence.SequenceItem goToSequenceItem(FourBarPosition fourBarPosition) {
		return new Sequence.SequenceItem(State.Message.ELEVATOR_DONE, () -> {
			state = new ActingState(fourBarPosition);
		});
	}

	// should only be used for testing
	public void goToImmediate(FourBarPosition fourBarPosition) {
		fourBar.setTargetPosition(fourBarPosition.desiredPosition);
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
