package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@TeleOp(name = "TwoDriverTeleopAdvanced", group = ".Main")
public class TwoDriverTeleopAdvanced extends LinearOpMode {
	/**
	 * A number to divide the speed by when finner controls are activated.
	 */
	final double speedDivisor = 4.5;

	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);
		Sequence sequence;

		waitForStart();

		while (opModeIsActive()) {
			gamepadA.update();
			gamepadB.update();

			Pose powers = new Pose();

			powers.x = -gamepad1.left_stick_x * 0.75;
			powers.y = -gamepad1.left_stick_y * 0.75;
			powers.angle = -gamepad1.right_stick_x * 0.5;

			// Activate slower driving and turning, for finer adjustment
			if (gamepad1.right_trigger > 0.2) {
				powers.x = -gamepad1.left_stick_x / speedDivisor;
				powers.y = -gamepad1.left_stick_y / speedDivisor;
				powers.angle = -gamepad1.right_stick_x / speedDivisor;
			}

			// Apply calculated velocity to mecanum wheels
			systems.drivingSystem.driveMecanum(powers);


			// Gwheel controls
			if (gamepadB.lt()) {
				systems.gWheelSystem.toggleSpit();
			} else if (gamepadB.rt()) {
				systems.gWheelSystem.toggleCollect();
			}

			// Get ready for pickup from gWheel
			if (gamepadB.cross()) {
				systems.interrupt();
				sequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.BACK),
						systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1));
				systems.executeSequence(sequence);
			}
			// Get ready for pickup from dropoff
			if (gamepadB.triangle()) {
				systems.interrupt();
				sequence = new Sequence(
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.BACK),
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
						systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1));
				systems.executeSequence(sequence);
			}
			// Pickup
			else if (gamepadB.square()) {
				systems.interrupt();
				sequence = new Sequence(
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
						systems.sleepingSystem.goToSequenceItem(200),
						systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1));
				systems.executeSequence(sequence);
			}
			// Go to dropoff low
			else if (gamepadB.dpad_down()) {
				systems.interrupt();
				sequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF));
				systems.executeSequence(sequence);
			}
			// Go to dropoff mid
			else if (gamepadB.dpad_left()) {
				systems.interrupt();
				sequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF));
				systems.executeSequence(sequence);
			}
			// Go to dropoff high
			else if (gamepadB.dpad_up()) {
				systems.interrupt();
				sequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF));
				systems.executeSequence(sequence);
			}
			// Open claw
			else if (gamepadB.circle()) {
				systems.interrupt();
				Sequence clawSequence = new Sequence(systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1));
				systems.executeSequence(clawSequence);
			}
			systems.tick();
		}
	}
}
