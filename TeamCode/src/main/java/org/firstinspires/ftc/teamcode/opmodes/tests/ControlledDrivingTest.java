package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "ControlledDrivingTest", group = ".Main")
public class ControlledDrivingTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		Pose actPowers = new Pose();

		int driveType = 0;

		waitForStart();
		while (opModeIsActive()) {
			gamepad.update();

			if (gamepad.square()) {
				systems.drivingSystem.driveToClosestPole();
			}

			// Calculate desired robot velocity
			actPowers.x = -gamepad1.left_stick_x * 0.4;
			actPowers.y = -gamepad1.left_stick_y * 0.4;
			actPowers.angle = -gamepad1.right_stick_x * 0.3;

			// Change driving mode
			if (gamepad.triangle()) {
				driveType = (driveType + 1) % 4;
			}

			switch (driveType) {
				case 0:
					systems.drivingSystem.driveByAxis(actPowers);
				case 1:
					systems.drivingSystem.controlledDriveByAxis(actPowers);
				case 2:
					systems.drivingSystem.controlledDriveByAxis2(actPowers);
				case 3:
					systems.drivingSystem.controlledDriveByAxis3(actPowers);
			}

			PointD tileLocation = systems.trackingSystem.getTileLocation();
			PointD tileDeviation = systems.trackingSystem.getTileDeviation();
			PointD tileLCenter = systems.trackingSystem.getTileCenter();
			PointD closePole = systems.trackingSystem.getClosestPoleLocation();

			telemetry.addData("tileLocation.x: ", tileLocation.x);
			telemetry.addData("tileLocation.y: ", tileLocation.y);
			telemetry.addData("tileDeviation.x: ", tileDeviation.x);
			telemetry.addData("tileDeviation.y: ", tileDeviation.y);
			telemetry.addData("tileLCenter.x: ", tileLCenter.x);
			telemetry.addData("tileLCenter.y: ", tileLCenter.y);
			telemetry.addData("closePole.x: ", closePole.x);
			telemetry.addData("closePole.y: ", closePole.y);

			telemetry.update();
			systems.tick();
		}
	}
}