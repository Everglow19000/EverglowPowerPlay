package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "FourBarTest", group = "Test")
public class FourBarTest extends LinearOpMode {
	Servo servo1;
	Servo servo2;

	@Override
	public void runOpMode() {
		servo1 = hardwareMap.get(Servo.class, "4bar_right");
		servo2 = hardwareMap.get(Servo.class, "4bar_left");
		servo1.setDirection(Servo.Direction.REVERSE);

		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		double position = 0.5;
		waitForStart();
		goTo(position);
		while (opModeIsActive()) {
			gamepad.update();
			position += gamepad1.left_stick_y * 0.01;
			position = Math.min(Math.max(position, 0), 1);
			goTo(position);
			telemetry.addData("position", position);
			telemetry.update();
			sleep(10);
		}
	}

	private void goTo(double state) {
		servo1.setPosition(state);
		servo2.setPosition(state);
	}
}