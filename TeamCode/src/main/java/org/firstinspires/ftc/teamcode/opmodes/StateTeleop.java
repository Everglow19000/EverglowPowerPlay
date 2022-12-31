package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "StateTeleop", group = ".Main")
public class StateTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);

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
			systemCoordinator.drivingSystem.state = new DrivingSystem.GoToPositionState(actPowers, 10);

			// Claw controls
			if (gamepad.rt()) {
				systemCoordinator.clawSystem.state = new ClawSystem.GoToPositionState(ClawSystem.ClawState.OPEN);
			} else if (gamepad.lt()) {
				systemCoordinator.clawSystem.state = new ClawSystem.GoToPositionState(ClawSystem.ClawState.CLOSED);
			}

			// Elevator controls
			if (gamepad.dpad_down()) {
				systemCoordinator.elevatorSystem.state = new ElevatorSystem.GoToPositionState(ElevatorSystem.Level.PICKUP);
			} else if (gamepad.circle()) {
				systemCoordinator.elevatorSystem.state = new ElevatorSystem.GoToPositionState(ElevatorSystem.Level.LOW);
			} else if (gamepad.triangle()) {
				systemCoordinator.elevatorSystem.state = new ElevatorSystem.GoToPositionState(ElevatorSystem.Level.MID);
			}

			// Telemetry
			systemCoordinator.drivingSystem.printPosition();
			telemetry.update();

			systemCoordinator.tick();
		}
	}
}
