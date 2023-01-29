package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Sequence;
import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBarSystem {
	/**
	 * Enum encapsulating the two positions the system should reach.
	 */
	public enum FourBarPosition {
		PICKUP(0.1, 0.), DROPOFF(0.3, 0.2);
//		PICKUP(0.1, 0.), DROPOFF(0.6, 0.5);

		private final double posRight;
		private final double posLeft;

		FourBarPosition(double posRight, double posLeft) {
			this.posRight = posRight;
			this.posLeft = posLeft;
		}

		/*
		 * Toggles the state of the fourBar
		 */
		public FourBarPosition toggle() {
			switch (this) {
				case PICKUP:
					return DROPOFF;
				case DROPOFF:
					return PICKUP;
				default:
					throw new IllegalStateException();
			}
		}
	}
	private final Servo servoRight;
	private final Servo servoLeft;

	private State state = new RestingState();
	/**
	 * A state used when the robot should be moving.
	 */
	public class ActingState implements State {
		private final double totalMovementTime;
		private final double startPositionLeft;
		private final double startPositionRight;
		private final double velocityLeft;
		private final ElapsedTime timer;
		private final double velocityRight;

		private final FourBarPosition targetState;

		public ActingState(FourBarPosition targetState, double velocity) {
			startPositionRight = servoRight.getPosition();
			startPositionLeft = servoLeft.getPosition();
			double errorRight = targetState.posRight - startPositionRight;
			double errorLeft = targetState.posLeft - startPositionLeft;
			this.targetState = targetState;
			this.totalMovementTime = max(abs(errorRight / velocity), abs(errorLeft / velocity));
			this.timer = new ElapsedTime();
			// Calculate position change per tick
			this.velocityRight = errorRight / totalMovementTime;
			this.velocityLeft = errorLeft / totalMovementTime;
		}

		public void tick() {
			// The claw has reached its desired position
			if (timer.time() > totalMovementTime) {
				servoRight.setPosition(targetState.posRight);
				servoLeft.setPosition(targetState.posLeft);
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.FOUR_BAR_DONE);
				return;
			}

			// Otherwise, update the claw position
			double positionRight = startPositionRight + velocityRight * timer.time();
			double positionLeft = startPositionLeft + velocityLeft * timer.time();
			servoRight.setPosition(positionRight);
			servoLeft.setPosition(positionLeft);

			SystemCoordinator.instance.opMode.telemetry.addData("positionRight", positionRight);
			SystemCoordinator.instance.opMode.telemetry.addData("positionLeft", positionLeft);
			SystemCoordinator.instance.opMode.telemetry.update();
		}
		public void onReceiveMessage(State.Message message) {
			// Do nothing
		}
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public FourBarSystem(LinearOpMode opMode) {
		servoRight = opMode.hardwareMap.get(Servo.class, "4bar_right");
		servoLeft = opMode.hardwareMap.get(Servo.class, "4bar_left");
		servoRight.setDirection(Servo.Direction.REVERSE);
	}

	/**
	 * Ticks the fourBar system.
	 */
	public void tick() {
		state.tick();
	}


	public Sequence.SequenceItem goToSequenceItem(FourBarPosition position, double velocity){
		return new Sequence.SequenceItem(State.Message.FOUR_BAR_DONE, ()->{
			state = new ActingState(position, velocity);
		});
	}

	/**
	 * Goes to the specified position immidiatly, without relying on the sate machine.
	 * Should only be used for testing.
	 * @param position the position to go to.
	 */
	public void goToImmediate(FourBarPosition position){
		servoRight.setPosition(position.posRight);
		servoLeft.setPosition(position.posLeft);
	}

}
