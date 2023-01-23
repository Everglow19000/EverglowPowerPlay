package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "FourBarTest", group = "Test")
public class FourBarTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		FourBarSystem system = new FourBarSystem(this);

		double position = 0.5;
		waitForStart();
		system.goTo(position);
		while (opModeIsActive()) {
			position += gamepad1.left_stick_y * 0.01;
			system.goTo(position);
			telemetry.addData("4bar:", position);
			telemetry.update();

			system.tick();
			sleep(10);
		}
	}
}