package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ClawTest", group = ".Main")
public class ClawTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		Servo servoRight = hardwareMap.get(Servo.class, "4bar_right");
		Servo servoLeft = hardwareMap.get(Servo.class, "4bar_left");
		servoRight.setDirection(Servo.Direction.REVERSE);
		waitForStart();

		double position = 0.5;
		double offset = 0.1;
		while (opModeIsActive()) {
			//Code goes here
			position += 0.01 * gamepad1.left_stick_x;
			telemetry.addData("position:", position);
			telemetry.update();

			servoRight.setPosition(position + offset);
			servoLeft.setPosition(position);
			sleep(10);
		}
	}
}
