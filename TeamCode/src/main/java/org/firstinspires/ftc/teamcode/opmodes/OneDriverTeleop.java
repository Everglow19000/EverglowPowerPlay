package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "OneDriverTeleop", group = ".Main")
public class OneDriverTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

		DrivingSystem drivingSystem = new DrivingSystem(this);
		ClawSystem claw = new ClawSystem(this);
		ElevatorSystem elevator = new ElevatorSystem(this);

		Pose actPowers = new Pose(0, 0, 0);
		final int divisorSpeed = 10; // the amount to divide the speed when finner controls are activated

		waitForStart();

		while (opModeIsActive()) {
			gamepad.update();

			// Calculate desired robot velocity
			actPowers.x = -gamepad1.left_stick_x;
			actPowers.y = -gamepad1.left_stick_y;
			actPowers.angle = -gamepad1.right_stick_x;

			// Activate slower driving and turning, for finer adjustment
			if (gamepad1.left_bumper) {
				actPowers.x /= divisorSpeed;
				actPowers.y /= divisorSpeed;
				actPowers.angle /= divisorSpeed;
			}

			// Apply calculated velocity to mecanum wheels
			drivingSystem.driveMecanum(actPowers);

			// Claw controls
			if (gamepad.rt()) {
				claw.goTo(ClawSystem.ClawState.CLOSED);
			}
			if (gamepad.lt()) {
				claw.goTo(ClawSystem.ClawState.OPEN);
			}

			// Elevator controls
			if (gamepad.dpad_down()) {
				elevator.goTo(ElevatorSystem.Level.PICKUP);
			}

			if (gamepad.circle()) {
				elevator.goTo(ElevatorSystem.Level.LOW);
			}

			if (gamepad.triangle()) {
				elevator.goTo(ElevatorSystem.Level.MID);
			}
			Orientation orientation = drivingSystem.getOrientation();
			telemetry.addData("z:", orientation.firstAngle);
			telemetry.addData("x: ", orientation.secondAngle);
			telemetry.addData("y: ", orientation.thirdAngle);
			// Telemetry
			drivingSystem.printPosition();
			telemetry.update();
		}
	}
}
