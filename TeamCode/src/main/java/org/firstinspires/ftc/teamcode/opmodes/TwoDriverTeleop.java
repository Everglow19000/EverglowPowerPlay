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

@TeleOp(name = "TwoDriverTeleop", group = ".Main")
public class TwoDriverTeleop extends LinearOpMode {
	/**
	 * A number to divide the speed by when finner controls are activated
	 */
	final double speedDivisor = 4.5;

	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

		Sequence elevatorSequence = null;
		boolean isClawOpen = false;

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
			systems.drivingSystem.driveByAxis(powers);


			if (gamepadB.lt()) {
				systems.gWheelSystem.toggleSpit();
			} else if (gamepadB.rt()) {
				systems.gWheelSystem.toggleCollect();
			}

			if (gamepadB.circle()) {
				systems.interrupt();
				Sequence.SequenceItem sequenceItem = systems.clawSystem.goToSequenceItem(isClawOpen
						? ClawSystem.ClawPosition.OPEN : ClawSystem.ClawPosition.CLOSED, 1);
				sequenceItem.runAction.run();
				isClawOpen = !isClawOpen;
			} // claw toggle

			if (gamepadB.dpad_down()) {
				systems.interrupt();
				elevatorSequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP)
//						systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP, 1)
				);
				systems.executeSequence(elevatorSequence);
			} else if (gamepadB.dpad_left()) {
				systems.interrupt();
				elevatorSequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP)
				);
				systems.executeSequence(elevatorSequence);
			} else if (gamepadB.dpad_up()) {
				systems.interrupt();
				elevatorSequence = new Sequence(
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP)
				);
				systems.executeSequence(elevatorSequence);
			} // go to dropOff

			else if (gamepadB.triangle()) {
				systems.interrupt();
				elevatorSequence = new Sequence(
						systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 1),
						systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
						systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP)
				);
				systems.executeSequence(elevatorSequence);
			} // dropOff

			systems.tick();
		}
	}
}
