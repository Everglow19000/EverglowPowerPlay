package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A class for coordinating the all systems on the robot in a state machine.
 */
public class SystemCoordinator {
	//Create a new instance of each system
	private final ElevatorSystem elevatorSystem;
	private final ClawSystem clawSystem;
	private final DrivingSystem drivingSystem;
	private final FourBarSystem fourBarSystem;
	private final GWheelSystem gWheelSystem;
	//There is not camera system because it runs in a separate thread.

	public SystemCoordinator(LinearOpMode opMode) {
		//Create a new instance of each system and initiate them
		elevatorSystem = new ElevatorSystem(opMode);
		clawSystem = new ClawSystem(opMode);
		drivingSystem = new DrivingSystem(opMode);
		fourBarSystem = new FourBarSystem(opMode);
		gWheelSystem = new GWheelSystem(opMode);
	}

	/**
	 * Each system should be ticked in an epsilon amount of time.
	 */
	public void tick() {
		//Tick each system
		elevatorSystem.state.tick();
		clawSystem.state.tick();
		//drivingSystem.state.tick(); //TODO: Too complex for now
		fourBarSystem.state.tick();
		//gWheelSystem.state.tick(); //Commented out because is has only two states: running or not
	}
}
