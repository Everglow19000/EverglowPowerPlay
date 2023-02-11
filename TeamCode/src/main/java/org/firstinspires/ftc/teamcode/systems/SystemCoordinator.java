package org.firstinspires.ftc.teamcode.systems;

import java.util.ArrayList;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;
import org.firstinspires.ftc.teamcode.utils.StateMachine.StateMessages;

/**
 * A class for coordinating the all systems on the robot in a state machine.
 */
public class SystemCoordinator {
	/**
	 * The instance of the system coordinator, for use in other classes.
	 * It is used to avoid dependency injection.
	 */
	public static SystemCoordinator instance;

	public final LinearOpMode opMode;

	//Create a new instance of each system
	public final DrivingSystem drivingSystem;
	public final TrackingSystem trackingSystem;
	public final ClawSystem clawSystem;
	public final FourBarSystem fourBarSystem;
	public final ElevatorSystem elevatorSystem;
	public final GWheelSystem gWheelSystem;
	public final SleepingSystem sleepingSystem;
	public final PositionLogger positionLogger;
	//There is no camera system because it runs in a separate thread.

	private final ArrayList<Sequence> actionSequences;

	private long lastCycleTime;

	public long getLastCycleTime() {
		return lastCycleTime;
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public SystemCoordinator(LinearOpMode opMode) {
		instance = this;
		this.opMode = opMode;

		lastCycleTime = System.nanoTime();

		// Initiate all the systems
		drivingSystem = new DrivingSystem(opMode);
		trackingSystem = new TrackingSystem(opMode);
		clawSystem = new ClawSystem(opMode);
		fourBarSystem = new FourBarSystem(opMode);
		elevatorSystem = new ElevatorSystem(opMode);
		gWheelSystem = new GWheelSystem(opMode);
		sleepingSystem = new SleepingSystem(opMode);
		positionLogger = new PositionLogger(drivingSystem);

		// Initiate the list of sequences
		actionSequences = new ArrayList<>();

	}

	/**
	 * Each system should be ticked in an epsilon amount of time.
	 */
	public void tick() {
		lastCycleTime = System.nanoTime();
		//Tick each system
		clawSystem.tick();
		elevatorSystem.tick();
		fourBarSystem.tick();
		trackingSystem.tick();
		positionLogger.tick();
		sleepingSystem.tick();
	}

	/**
	 * Stops the execution of all the systems and clears the queued sequences.
	 * Should often be used before executing a new sequence.
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
	 * Returns weather all sequences have finished executing.
	 */
	public boolean allSequencesDone() {
		for (Sequence sequence : actionSequences) {
			if (!sequence.isSequenceDone()) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Waits for all sequences to finish, and blocks until then.
	 */
	public void waitForSequencesDone() {
		while (opMode.opModeIsActive() && !allSequencesDone()) {
			tick();
		}
	}

	/**
	 * Waits a certain amount of time, while ticking the systems in the process.
	 * This function should be used in autonomous.
	 * This is used because opMode.sleep() should NOT be used when using state machine.
	 *
	 * @param milliseconds how many milliseconds to wait.
	 */
	public void sleep(long milliseconds) {
		ElapsedTime elapsedTime = new ElapsedTime();
		while (opMode.opModeIsActive() && elapsedTime.milliseconds() < milliseconds) {
			tick();
		}
	}

	/**
	 * Broadcasts a message from one system to all of them.
	 *
	 * @param message The message to be broadcasted.
	 */
	public void sendMessage(StateMessages message) {
		//TODO: remove the sequences if they are done. This isn't critical but should probably be done for elegance.
		for (Sequence sequence : actionSequences) {
			sequence.handleMessage(message);
		}
	}
}
