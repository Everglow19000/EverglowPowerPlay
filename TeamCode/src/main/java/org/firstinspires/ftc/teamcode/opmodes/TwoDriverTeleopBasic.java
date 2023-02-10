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

@TeleOp(name = "TwoDriverTeleopBasic", group = ".Main")
public class TwoDriverTeleopBasic extends LinearOpMode {
	/**
	 * A number to divide the speed by when finner controls are activated.
	 */
	final double speedDivisor = 4.5;

	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);
		Pose actPowers = new Pose(0, 0, 0);

		ClawSystem.Position claw = ClawSystem.Position.OPEN;

		waitForStart();
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

			// Elevator
			if (gamepadB.dpad_up()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepadB.dpad_left()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepadB.dpad_down()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepadB.dpad_right()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP);
				systems.executeSequence(new Sequence(sequenceItem));
			}

			// FourBar
			if (gamepadB.lb()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepadB.rb()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepadB.square()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START);
				systems.executeSequence(new Sequence(sequenceItem));
			}

			// Grabbing Wheel
			if (gamepadB.lt()) {
				systems.gWheelSystem.toggleSpit();
			}
			if (gamepadB.rt()) {
				systems.gWheelSystem.toggleCollect();
			}

			// Claw
			if (gamepadB.triangle()) {
				claw = claw.flip();
				Sequence clawSequence = new Sequence(systems.clawSystem.goToSequenceItem(claw, 1));
				systems.executeSequence(clawSequence);
			}

			telemetry.update();
			systems.tick();
		}
	}
}