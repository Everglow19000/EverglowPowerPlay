package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;

/**
 * A class that contains all of the autonomous routes for the robot.
 */
public class AutonomousRoutes {
	private final LinearOpMode opMode;
	private final CameraSystem cameraSystem;

	public AutonomousRoutes(LinearOpMode opMode) {
		this.opMode = opMode;
		cameraSystem = new CameraSystem(opMode);
	}

	/**
	 * A test method that drives the robot forwards or sideways, depending on the value the AprilTag.
	 */
	public void run() {
//		clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED);
		CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
		opMode.telemetry.addData("tag", tagType.toString());
		opMode.telemetry.update();
		switch (tagType) {
			case TAG_1:
				//DO SOMETHING
				break;
			case TAG_2:
			default:
				//DO SOMETHING
				break;
			case TAG_3:
				//DO SOMETHING
				break;
		}
	}
}