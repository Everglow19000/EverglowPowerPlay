package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.State;

/**
 * A class for coordinating the all systems on the robot in a state machine.
 */
public class SystemCoordinator {
	//Create a new instance of each system
	public final ElevatorSystem elevatorSystem;
	public final ClawSystem clawSystem;
	public final DrivingSystem drivingSystem;
	public final FourBarSystem fourBarSystem;
	//There is no gWheel system because is has only two states: running or not. There might be in the future.
	//There is no camera system because it runs in a separate thread.

	public SystemCoordinator(LinearOpMode opMode) {
		//Initiate all the systems
		elevatorSystem = new ElevatorSystem(opMode);
		clawSystem = new ClawSystem(opMode);
		drivingSystem = new DrivingSystem(opMode);
		fourBarSystem = new FourBarSystem(opMode);
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
	}

	/**
	 * Broadcasts a message from one system to all of them.
	 *
	 * @param message The message to be broadcasted.
	 */
	public void sendMessage(State.Message message) {
		clawSystem.receiveMessage();
		drivingSystem.receiveMessage();
		elevatorSystem.receiveMessage();
		fourBarSystem.receiveMessage();
	}
}
