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

@TeleOp(name = "OneDriverTeleopBasic", group = ".Main")
public class OneDriverTeleopBasic extends LinearOpMode {
	/**
	 * A number to divide the speed by when finner controls are activated.
	 */
	final double speedDivisor = 4.5;

	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		Pose actPowers = new Pose(0, 0, 0);

		ClawSystem.Position claw = ClawSystem.Position.CLOSED;

		waitForStart();
		while (opModeIsActive()) {
			gamepad.update();

			// Calculate desired robot velocity
			actPowers.x = -gamepad1.left_stick_x * 0.75;
			actPowers.y = -gamepad1.left_stick_y * 0.75;
			actPowers.angle = -gamepad1.right_stick_x * 0.5;

			// Activate slower driving and turning, for finer adjustment
			if (gamepad.cross()) {
				actPowers.x = -gamepad1.left_stick_x / speedDivisor;
				actPowers.y = -gamepad1.left_stick_y / speedDivisor;
				actPowers.angle = -gamepad1.right_stick_x / speedDivisor;
			}

			// Apply calculated velocity to mecanum wheels
			systems.drivingSystem.driveMecanum(actPowers);

			// Elevator
			if (gamepad.dpad_up()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepad.dpad_left()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepad.dpad_down()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepad.dpad_right()) {
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP);
				systems.executeSequence(new Sequence(sequenceItem));
			}

			// FourBar
			if (gamepad.lb()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepad.rb()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP);
				systems.executeSequence(new Sequence(sequenceItem));
			}
			if (gamepad.triangle()) {
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START);
				systems.executeSequence(new Sequence(sequenceItem));
			}

			// Grabbing Wheel
			if (gamepad.lt()) {
				systems.gWheelSystem.toggleSpit();
			}
			if (gamepad.rt()) {
				systems.gWheelSystem.toggleCollect();
			}

			// Claw
			if (gamepad.circle()) {
				claw = claw.flip();
				Sequence clawSequence = new Sequence(systems.clawSystem.goToSequenceItem(claw, 1));
				systems.executeSequence(clawSequence);
			}

			telemetry.update();
			systems.tick();
		}
	}
}