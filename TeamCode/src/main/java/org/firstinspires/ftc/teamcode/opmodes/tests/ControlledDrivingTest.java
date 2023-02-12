package org.firstinspires.ftc.teamcode.opmodes.tests;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.RobotParameters;

@TeleOp(name = "ControlledDrivingTest", group = "Test")
public class ControlledDrivingTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systems = SystemCoordinator.init(this);
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		Pose actPowers = new Pose();

		int driveType = 0;

		systems.trackingSystem.resetPosition(new Pose(RobotParameters.TILE_SIZE*1.5, RobotParameters.TILE_SIZE*(-1.5), toRadians(0)));

		waitForStart();
		while (opModeIsActive()) {
			gamepadA.update();

			if (gamepadA.square()) {
				systems.drivingSystem.driveToClosestPole();
			}

			// Calculate desired robot velocity
			actPowers.x = -gamepad1.left_stick_x * 0.4;
			actPowers.y = -gamepad1.left_stick_y * 0.4;
			actPowers.angle = -gamepad1.right_stick_x * 0.3;

			// Change driving mode
			if (gamepadA.cross()) {
				driveType = (driveType + 1) % 4;
			}

			systems.drivingSystem.driveMecanum(actPowers);

			switch (driveType) {
				case 0:
					systems.drivingSystem.driveMecanum(actPowers);
					break;
				case 1:
					systems.drivingSystem.controlledDriveByAxis(actPowers);
					break;
				case 2:
					systems.drivingSystem.controlledDriveByAxis2(actPowers);
					break;
				case 3:
					systems.drivingSystem.controlledDriveByAxis3(actPowers);
					break;
			}

			PointD tileLocation = systems.trackingSystem.getTileLocation();
			PointD tileDeviation = systems.trackingSystem.getTileDeviation();
			PointD tileLCenter = systems.trackingSystem.getTileCenter();
			PointD closePole = systems.trackingSystem.getClosestPoleLocation();
			telemetry.addData("driveType: ", driveType);

			telemetry.addData("tileLocation.x: ", tileLocation.x);
			telemetry.addData("tileLocation.y: ", tileLocation.y);
			telemetry.addData("tileDeviation.x: ", tileDeviation.x);
			telemetry.addData("tileDeviation.y: ", tileDeviation.y);
			//telemetry.addData("tileLCenter.x: ", tileLCenter.x);
			//telemetry.addData("tileLCenter.y: ", tileLCenter.y);
			//telemetry.addData("closePole.x: ", closePole.y);
			//telemetry.addData("closePole.y: ", closePole.x);

			telemetry.update();
			systems.tick();
		}
	}
}