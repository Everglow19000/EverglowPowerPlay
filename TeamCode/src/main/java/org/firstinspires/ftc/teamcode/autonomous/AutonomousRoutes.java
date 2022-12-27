package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;

/**
 * A class that contains all of the autonomous routes for the robot.
 */
public class AutonomousRoutes {
	private final LinearOpMode opMode;
	private final DrivingSystem drivingSystem;
	private final CameraSystem cameraSystem;
	private final ClawSystem clawSystem;

	public AutonomousRoutes(LinearOpMode opMode) {
		this.opMode = opMode;
		drivingSystem = new DrivingSystem(opMode);
		cameraSystem = new CameraSystem(opMode);
		clawSystem = new ClawSystem(opMode);
	}

	/**
	 * A test method that drives the robot forwards or sideways, depending on the value the AprilTag.
	 */
	public void run() {
		clawSystem.goTo(ClawSystem.ClawState.CLOSED);
		CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
		opMode.telemetry.addData("tag", tagType.toString());
		opMode.telemetry.update();
		switch (tagType) {
			case TAG_1:
				drivingSystem.driveSideways(65, 0.5);
				drivingSystem.driveStraight(90, 0.5);
				break;
			case TAG_2:
			default:
				drivingSystem.driveStraight(90, 0.5);
				break;
			case TAG_3:
				drivingSystem.driveSideways(-65, 0.5);
				drivingSystem.driveStraight(90, 0.5);
				break;
		}
	}
}
