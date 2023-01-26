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

@TeleOp(name = "TestStates", group = ".Main")
public class TestStates extends LinearOpMode {

	final double speedDivisor = 4.5; // the amount to divide the speed when finner controls are activated

	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		Sequence elevatorSequence = null;
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);
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

			if (gamepadB.cross()){
				Sequence.SequenceItem sequenceItem = systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID);
				sequenceItem.runAction.run();
			}
			if (gamepadB.circle()){
				Sequence.SequenceItem sequenceItem = systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED, 1);
				sequenceItem.runAction.run();
			}
			if (gamepadB.square()){
				Sequence.SequenceItem sequenceItem = systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 1);
				sequenceItem.runAction.run();
			}
			if (gamepadB.dpad_down()){
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.DROPOFF, 1);
				sequenceItem.runAction.run();
			}
			if (gamepadB.dpad_up()){
				Sequence.SequenceItem sequenceItem = systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP, 1);
				sequenceItem.runAction.run();
			}
			systems.tick();
		}
	}
}
