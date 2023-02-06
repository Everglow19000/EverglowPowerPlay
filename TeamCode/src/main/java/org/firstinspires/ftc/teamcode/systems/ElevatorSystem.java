package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.Sequence;
import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;

/**
 * A class for handling the elevator system.
 */

public class ElevatorSystem {
	/**
	 * The left elevator motor.
	 */
	private final DcMotor left;
	/**
	 * The right elevator motor.
	 */
	private final DcMotor right;
	/**
	 * The current state of the the DrivingSystem.
	 */
	private State state = new RestingState();

	/**
	 * Enum encapsulating all the positions the system should reach.
	 */
	public enum Level {
		PICKUP(0), PRE_PICKUP(-1833), LOW(-1833), MID(-2914), HIGH(-2914);

		public final int desiredPosition;

		Level(int desiredPosition) {
			this.desiredPosition = desiredPosition;
		}
	}

	/**
	 * A state used when the elevator should be moving.
	 */
	public class ActingState implements State {
		private static final int EPSILON = 20;
		private final Level level;

		/**
		 * @param level A elevator level to move to (e.g. ElevatorLevel.GROUND).
		 */
		public ActingState(Level level) {
			this.level = level;
			left.setTargetPosition(level.desiredPosition);
			right.setTargetPosition(level.desiredPosition);
		}

		public void tick() {
			// The elevator has reached its desired position
			int leftError = abs(level.desiredPosition - left.getCurrentPosition());
			boolean leftArrived = leftError < EPSILON;
			int rightError = abs(level.desiredPosition - right.getCurrentPosition());
			boolean rightArrived = rightError < EPSILON;
			SystemCoordinator.instance.opMode.telemetry.addData("leftError", leftError);
			SystemCoordinator.instance.opMode.telemetry.addData("rightError", rightError);
			SystemCoordinator.instance.opMode.telemetry.update();
			if (leftArrived && rightArrived) {
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.ELEVATOR_DONE);
			}
		}

	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public ElevatorSystem(OpMode opMode) {
		left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
		right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

		left.setDirection(DcMotor.Direction.REVERSE);
		left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left.setTargetPosition(0);
		right.setTargetPosition(0);
		left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		left.setPower(0.7);
		right.setPower(0.7);
	}

	/**
	 * Ticks the elevator system.
	 */
	public void tick() {
		state.tick();
	}

	/**
	 * Stops the elevator system.
	 */
	public void interrupt() {
		state = new RestingState();
		left.setTargetPosition(left.getCurrentPosition());
		right.setTargetPosition(right.getCurrentPosition());
	}

	public Sequence.SequenceItem goToSequenceItem(ElevatorSystem.Level level) {
		return new Sequence.SequenceItem(State.Message.ELEVATOR_DONE, () -> {
			state = new ActingState(level);
		});
	}

	public void goToTicks(int ticks) {
		left.setTargetPosition(ticks);
		right.setTargetPosition(ticks);
	}

	public void setPower(double power) {
		left.setPower(power);
		right.setPower(power);
	}

	/**
	 * Goes to the specified position immediately, without relying on the state machine.
	 * Should only be used for testing.
	 *
	 * @param level the level to go it.
	 */
	public void goToImmediate(Level level) {
		left.setTargetPosition(level.desiredPosition);
		right.setTargetPosition(level.desiredPosition);
	}
}
