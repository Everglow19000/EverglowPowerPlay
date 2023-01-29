package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.Sequence;

@TeleOp(name = "TwoDriverTeleop2", group = ".Main")
public class TwoDriverTeleop2 extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);
		Pose actPowers = new Pose(0, 0, 0);

		boolean isClawOpen = false;
		final double speedDivisor = 4.5; // the amount to divide the speed when finner controls are activated

		waitForStart();
		Sequence sequence = new Sequence(
				systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 1),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP, 1));
		sequence.start();

		while (opModeIsActive()) {
			gamepadA.update();
			gamepadB.update();

			// Calculate desired robot velocity
			actPowers.x = -gamepad1.left_stick_x * 0.75;
			actPowers.y = -gamepad1.left_stick_y * 0.75;
			actPowers.angle = -gamepad1.right_stick_x * 0.5;

			// Activate slower driving and turning, for finer adjustment
			if (gamepad1.right_trigger > 0.2) {
				actPowers.x = -gamepad1.left_stick_x / speedDivisor;
				actPowers.y = -gamepad1.left_stick_y / speedDivisor;
				actPowers.angle = -gamepad1.right_stick_x / speedDivisor;
			}

			// Apply calculated velocity to mecanum wheels
			systems.drivingSystem.driveMecanum(actPowers);

			if (gamepadB.dpad_up()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH);
				sequenceItem.runAction.run();
			}
			if (gamepadB.dpad_left()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID);
				sequenceItem.runAction.run();
			}
			if (gamepadB.dpad_down()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW);
				sequenceItem.runAction.run();
			}
			if (gamepadB.dpad_right()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP);
				sequenceItem.runAction.run();
			}

			if (gamepadB.lt()) {
				systems.gWheelSystem.toggleSpit();
			}
			if (gamepadB.rt()) {
				systems.gWheelSystem.toggleCollect();
			}

			if (gamepadB.lb()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.DROPOFF, 1);
				sequenceItem.runAction.run();
			}
			if (gamepadB.rb()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP, 1);
				sequenceItem.runAction.run();
			}

			if (gamepadB.circle()) {
				Sequence.SequenceItem sequenceItem = systems.clawSystem.goToSequenceItem(isClawOpen
						? ClawSystem.ClawPosition.OPEN : ClawSystem.ClawPosition.CLOSED, 1);
				sequenceItem.runAction.run();
				isClawOpen = !isClawOpen;
			}
			telemetry.update();
		}
	}
}