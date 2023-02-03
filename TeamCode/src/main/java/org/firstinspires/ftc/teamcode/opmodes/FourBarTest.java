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
		double position1 = 0.5;
		double position2 = 0.5;
		waitForStart();
		while (opModeIsActive()) {
			gamepad.update();
			position1 += gamepad1.left_stick_y * 0.003;
			position2 += gamepad1.right_stick_y * 0.003;
			position1 = Math.min(Math.max(position1, 0), 1);
			position2 = Math.min(Math.max(position2, 0), 1);
			servo1.setPosition(position1);
			servo2.setPosition(position2);
			telemetry.addData("position1", position1);
			telemetry.addData("position2", position2);
			telemetry.update();
			sleep(10);
		}
	}
}