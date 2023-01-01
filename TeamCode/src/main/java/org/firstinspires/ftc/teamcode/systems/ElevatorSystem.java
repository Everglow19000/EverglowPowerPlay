package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;

/**
 * A class for handling the elevator system.
 */
public class ElevatorSystem {
	private final DcMotor left;
	private final DcMotor right;
	private State state;

	/**
	 * A state used when the robot should be moving.
	 */
	public class GoToPositionState implements State {
		private final double totalMovementTime;
		private final double startPositionLeft;
		private final double startPositionRight;
		private final ElapsedTime timer;
		private final double velocity;

		/**
		 * @param level             A elevator level to move to (e.g. ElevatorLevel.GROUND).
		 * @param totalMovementTime The total time the movement should take.
		 */
		public GoToPositionState(Level level, double totalMovementTime) {
			this(level.desiredPosition, totalMovementTime);
		}

		/**
		 * @param desiredPosition   The desired position the elevator should move to, in ticks.
		 * @param totalMovementTime The total time the movement should take.
		 */
		public GoToPositionState(double desiredPosition, double totalMovementTime) {
			this.totalMovementTime = totalMovementTime;
			this.startPositionLeft = left.getCurrentPosition();
			this.startPositionRight = right.getCurrentPosition();
			this.timer = new ElapsedTime();
			// Calculate position change per tick
			//TODO: USE ACCELERATION PROFILE - THIS DOES NOT CURRENTLY WORK
			this.velocity = (desiredPosition - (startPositionLeft + startPositionRight) / 2) / (totalMovementTime);
		}

		public void tick() {
			// The claw has reached its desired position
			if (timer.time() > totalMovementTime) {
				state = new RestingState();
				return;
			}

			// Otherwise, update the elevator level
			left.setPower(velocity);
			right.setPower(velocity);
		}

		public void onReceiveMessage(State.Message message) {
			// Do nothing
		}
	}


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
	 * Sets the elevator to the specified level.
	 *
	 * @param level        The level to move the elevator to.
	 * @param movementTime The time it should take the elevator to reach the desired position.
	 */
	public void goTo(Level level, double movementTime) {
		this.state = new GoToPositionState(level, movementTime);
	}

	/**
	 * Sets the elevator to the specified level.
	 * The time to reach the position is static at 2 seconds.
	 *
	 * @param level The level to set the elevator to (e.g. ElevatorLevel.GROUND).
	 */
	public void goTo(Level level) {
		goTo(level, 2);
	}

	/**
	 * Ticks the elevator system.
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
