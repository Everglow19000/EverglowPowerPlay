package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Sequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		CameraSystem cameraSystem = new CameraSystem(this);
		AutonomousRoutes autonomousRoutes = new AutonomousRoutes(this, true);
		waitForStart();
		CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();

		autonomousRoutes.putConesAndBack();
		autonomousRoutes.park(tagType);
	}
}