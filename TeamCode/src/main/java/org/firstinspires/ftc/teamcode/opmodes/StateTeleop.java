package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "StateTeleop", group = ".Main")
public class StateTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);

		Pose actPowers = new Pose(0, 0, 0);
		final int divisorSpeed = 10; // The amount to divide the speed when finner controls are activated

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
			systemCoordinator.drivingSystem.goTo(actPowers);

			// Claw controls
			if (gamepad.rt()) {
				systemCoordinator.clawSystem.goTo(ClawSystem.ClawState.OPEN);
			} else if (gamepad.lt()) {
				systemCoordinator.clawSystem.goTo(ClawSystem.ClawState.OPEN);
			}

			// FourBar controls
			if (gamepad.rb()) {
				systemCoordinator.fourBarSystem.goTo(FourBarSystem.FourBarState.PICKUP);
			} else if (gamepad.lb()) {
				systemCoordinator.fourBarSystem.goTo(FourBarSystem.FourBarState.DROPOFF);
			}

			// Elevator controls
			if (gamepad.dpad_down()) {
				systemCoordinator.elevatorSystem.goTo(ElevatorSystem.Level.PICKUP);
			} else if (gamepad.circle()) {
				systemCoordinator.elevatorSystem.goTo(ElevatorSystem.Level.LOW);
			} else if (gamepad.triangle()) {
				systemCoordinator.elevatorSystem.goTo(ElevatorSystem.Level.MID);
			}

			// Telemetry
			systemCoordinator.trackingSystem.printPosition();
			telemetry.update();

			systemCoordinator.tick();
		}
	}
}
