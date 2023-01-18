package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;

/**
 * A class that contains all of the autonomous routes for the robot.
 */
public class AutonomousRoutes {
	private final LinearOpMode opMode;
	private final DrivingSystem drivingSystem;
	private final CameraSystem cameraSystem;
	private final ClawSystem clawSystem;
	private final ElevatorSystem elevatorSystem;
	private final FourBarSystem fourBarSystem;

	public AutonomousRoutes(LinearOpMode opMode) {
		this.opMode = opMode;
		drivingSystem = new DrivingSystem(opMode);
		cameraSystem = new CameraSystem(opMode);
		clawSystem = new ClawSystem(opMode);
		elevatorSystem = new ElevatorSystem(opMode);
		fourBarSystem = new FourBarSystem(opMode);
	}

	/**
	 * A test method that drives the robot forwards or sideways, depending on the value the AprilTag.
	 */
	public void run(int right) {
		clawSystem.goTo(ClawSystem.ClawState.CLOSED);
		CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
		opMode.telemetry.addData("tag", tagType.toString());
		opMode.telemetry.update();

		int sideWays = -85*right;

		drivingSystem.driveStraight(60, 0.5);
		drivingSystem.driveSideways(sideWays, 0.5);
		elevatorSystem.goTo(ElevatorSystem.Level.HIGH);
		fourBarSystem.goTo(FourBarSystem.FourBarState.DROPOFF);
		drivingSystem.driveStraight(5, 0.5);
		clawSystem.goTo(ClawSystem.ClawState.OPEN);
		opMode.sleep(1000);
		drivingSystem.driveStraight(-5, 0.5);
		clawSystem.goTo(ClawSystem.ClawState.CLOSED);
		fourBarSystem.goTo(FourBarSystem.FourBarState.PICKUP);
		elevatorSystem.goTo(ElevatorSystem.Level.PICKUP);

		switch (tagType) {
			case TAG_1:
				drivingSystem.driveSideways(60-sideWays, 0.5);
				break;
			case TAG_2:
			default:
				drivingSystem.driveSideways(90, 0.5);
				break;
			case TAG_3:
				drivingSystem.driveSideways(-65-sideWays, 0.5);
				break;
		}
	}
}
