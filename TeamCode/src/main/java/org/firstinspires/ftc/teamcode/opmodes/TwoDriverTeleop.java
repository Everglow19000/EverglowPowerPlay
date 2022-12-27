package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "TwoDriverTeleop", group = ".Main")
public class TwoDriverTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

		DrivingSystem drivingSystem = new DrivingSystem(this);
		ClawSystem claw = new ClawSystem(this);
		ElevatorSystem elevator = new ElevatorSystem(this);

		Pose actPowers = new Pose(0, 0, 0);
		ClawSystem.ClawState clawPosition = ClawSystem.ClawState.OPEN;
		final double speedDivisor = 4.5;

		claw.goTo(clawPosition);

		waitForStart();

		while (opModeIsActive()) {
			gamepadA.update();
			gamepadB.update();

			// If we want to drive and turn slower, for finer adjustment
			if (gamepad1.right_trigger > 0.2) {
				actPowers.x = -gamepad1.left_stick_x / speedDivisor;
				actPowers.y = -gamepad1.left_stick_y / speedDivisor;
				actPowers.angle = -gamepad1.right_stick_x / speedDivisor;
			} else {
				actPowers.x = -gamepad1.left_stick_x * 0.75;
				actPowers.y = -gamepad1.left_stick_y * 0.75;
				actPowers.angle = -gamepad1.right_stick_x * 0.5;
			}

			drivingSystem.driveMecanum(actPowers);

			if (gamepadB.lt()) {
				clawPosition = clawPosition.flip();
				claw.goTo(clawPosition);
			}

			if (gamepadB.dpad_down()) {
				elevator.goTo(ElevatorSystem.Level.PICKUP);
			}

			if (gamepadB.circle()) {
				elevator.goTo(ElevatorSystem.Level.LOW);
			}

			if (gamepadB.triangle()) {
				elevator.goTo(ElevatorSystem.Level.MID);
			}

			drivingSystem.printPosition();
			telemetry.update();
		}
	}
}
