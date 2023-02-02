package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "OneDriverTeleop", group = ".Main")
public class OneDriverTeleop extends LinearOpMode {
	/**
	 * A number to divide the speed by when finner controls are activated
	 */
	final double speedDivisor = 4.5; // the amount to divide the speed when finner controls are activated

	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		waitForStart();

		while (opModeIsActive()) {
			gamepad.update();

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

			if (gamepad.lt()) {
				systems.gWheelSystem.toggleSpit();
			}
			if (gamepad.rt()) {
				systems.gWheelSystem.toggleCollect();
			}
		}
	}
}
