package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;

@Autonomous(name = "Autonomous", group = ".Main")
public class MainAutonomous extends LinearOpMode {
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