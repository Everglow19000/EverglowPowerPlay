package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.Sequence;

import java.util.ArrayList;

/**
 * A class for coordinating the all systems on the robot in a state machine.
 */
public class SystemCoordinator {
	/**
	 * The instance of the system coordinator, for use in other classes.
	 * This is done to avoid dependency injection.
	 */
	public static SystemCoordinator instance;

	public final LinearOpMode opMode;

	//Create a new instance of each system
	public final ElevatorSystem elevatorSystem;
	public final ClawSystem clawSystem;
	public final DrivingSystem drivingSystem;
	public final FourBarSystem fourBarSystem;
	public final TrackingSystem trackingSystem;
	public final GWheelSystem gWheelSystem;
	//There is no camera system because it runs in a separate thread.

	private final ArrayList<Sequence> actionSequences;

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public SystemCoordinator(LinearOpMode opMode) {
		instance = this;
		this.opMode = opMode;
		// Initiate all the systems
		elevatorSystem = new ElevatorSystem(opMode);
		clawSystem = new ClawSystem(opMode);
		drivingSystem = new DrivingSystem(opMode);
		fourBarSystem = new FourBarSystem(opMode);
		trackingSystem = new TrackingSystem(opMode);
		gWheelSystem = new GWheelSystem(opMode);

		// Initiate the list of sequences
		actionSequences = new ArrayList<>();
	}

	/**
	 * Each system should be ticked in an epsilon amount of time.
	 */
	public void tick() {
		//Tick each system
		clawSystem.tick();
		drivingSystem.tick();
		elevatorSystem.tick();
		fourBarSystem.tick();
		trackingSystem.tick();
	}

	/**
	 * Stops the execution of all the systems and clears the queued sequences.
	 */
	public void interrupt() {
		clawSystem.interrupt();
		elevatorSystem.interrupt();
		fourBarSystem.interrupt();
		actionSequences.clear();
	}

	/**
	 * Executes a sequence.
	 *
	 * @param sequence The sequence to be executed.
	 */
	public void executeSequence(Sequence sequence) {
		sequence.start();
		actionSequences.add(sequence);
	}

	/**
	 * Broadcasts a message from one system to all of them.
	 *
	 * @param message The message to be broadcasted.
	 */
	public void sendMessage(State.Message message) {
		//TODO: remove the sequences if they are done. This isn't critical but should probably be done for elegance.
		for (Sequence sequence : actionSequences) {
			sequence.handleMessage(message);
		}
	}
}
