package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.StateMachine.RestingState;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;
import org.firstinspires.ftc.teamcode.utils.StateMachine.State;
import org.firstinspires.ftc.teamcode.utils.StateMachine.StateMessages;

public class SleepingSystem {
	/**
	 * The current state of the the Elevator System.
	 */
	private State state = new RestingState();

	/**
	 * A state used when the elevator should be moving.
	 */
	public class SleepingState implements State {
		/**
		 * The time the robot should wait, in milliseconds.
		 */
		private final int time;
		private final ElapsedTime timer;

		/**
		 * @param time The time the robot should wait, in milliseconds.
		 */
		public SleepingState(int time) {
			this.time = time;
			timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
		}

		public void tick() {
			// The time has passed, so we are done
			if (timer.milliseconds() >= time) {
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(StateMessages.SLEEP_DONE);
			}
		}
	}



	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public SleepingSystem(LinearOpMode opMode) {

	}

	public Sequence.SequenceItem goToSequenceItem(int time) {
		return new Sequence.SequenceItem(StateMessages.SLEEP_DONE, () -> {
			state = new SleepingSystem.SleepingState(time);
		});
	}
}
