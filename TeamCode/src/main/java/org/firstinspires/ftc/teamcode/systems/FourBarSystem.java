package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.RestingState;
import org.firstinspires.ftc.teamcode.utils.Sequence;
import org.firstinspires.ftc.teamcode.utils.State;

/**
 * A class for handling the elevator system.
 */
public class FourBarSystem {
	/**
	 * Enum encapsulating all the positions the system should reach.
	 */
	public enum Position {
		PICKUP(0), DROPOFF(-500);

		public final int desiredPosition;

		Position(int desiredPosition) {
			this.desiredPosition = desiredPosition;
		}
	}

	/**
	 * A state used when the robot should be moving.
	 */
	public class ActingState implements State {
		private static final int EPSILON = 20;
		private final Position position;

		/**
		 * @param position A elevator level to move to (e.g. ElevatorLevel.GROUND).
		 */
		public ActingState(Position position) {
			this.position = position;
			motor.setTargetPosition(position.desiredPosition);
		}

		public void tick() {
			// The claw has reached its desired position
			int error = abs(position.desiredPosition - motor.getCurrentPosition());
			SystemCoordinator.instance.opMode.telemetry.addData("error", error);
			SystemCoordinator.instance.opMode.telemetry.update();
			if (error <= EPSILON) {
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.ELEVATOR_DONE);
			}
		}

	}

	private final DcMotor motor;
	private State state = new RestingState();

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(OpMode opMode) {
		motor = opMode.hardwareMap.get(DcMotor.class, "4bar");
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setTargetPosition(0);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(0.7);
	}

	public Sequence.SequenceItem goToSequenceItem(Position position) {
		return new Sequence.SequenceItem(State.Message.ELEVATOR_DONE, () -> {
			state = new ActingState(position);
		});
	}

	// should only be used for testing
	public void goToImmediate(Position position) {
		motor.setTargetPosition(position.desiredPosition);
	}

	/**
	 * Ticks the elevator system.
	 */
	public void tick() {
		state.tick();
	}

	public void interrupt() {
		state = new RestingState();
		motor.setTargetPosition(motor.getCurrentPosition());
	}
}
