package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;
import org.firstinspires.ftc.teamcode.utils.StateMachine.State;
import org.firstinspires.ftc.teamcode.utils.StateMachine.RestingState;
import org.firstinspires.ftc.teamcode.utils.StateMachine.StateMessages;

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
	 * The current state of the the Elevator System.
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

	public class ElevatorRestingState implements State {

		public ElevatorRestingState() {
			left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			left.setPower(0);
			right.setPower(0);
			left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

		/**
		 * Keep the elevator at the desired position
		 */
		public void tick() {
			// Do nothing
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
				SystemCoordinator.instance.sendMessage(StateMessages.ELEVATOR_DONE);
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
		left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
		return new Sequence.SequenceItem(StateMessages.ELEVATOR_DONE, () -> {
			state = new ActingState(level);
		});
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
