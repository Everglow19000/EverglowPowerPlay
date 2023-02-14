package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.StateMachine.RestingState;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;
import org.firstinspires.ftc.teamcode.utils.StateMachine.State;
import org.firstinspires.ftc.teamcode.utils.StateMachine.StateMessages;

/**
 * A Class for handling the claw system.
 */
public class VerifyStopSequence {
	private State state = new RestingState();

	/**
	 * A state used when the claw should be moving.
	 */
	public class ActingState implements State {
		ElapsedTime elapsedTime = new ElapsedTime();
		@Override
		public void tick() {
			if (elapsedTime.milliseconds() > 25 * 1000){
				SystemCoordinator systems = SystemCoordinator.instance;
				Sequence endSequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.BACK),
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.START),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START),
						systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
				);


				systems.drivingSystem.enabled = false;
				systems.interrupt();
				systems.executeSequence(endSequence);

			}
		}
	}

	public void enable(){
		state = new ActingState();
	}

	public void tick(){
		state.tick();
	}

}