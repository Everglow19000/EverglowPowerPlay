package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "ClawTest")
public class ClawTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

		Servo claw1 = hardwareMap.get(Servo.class, "claw1");
		Servo claw2 = hardwareMap.get(Servo.class, "claw2");
		claw2.setDirection(Servo.Direction.REVERSE);
		double position1 = 0.5;
		double position2 = 0.5;
		waitForStart();
		claw1.setPosition(position1);
		claw2.setPosition(position1);
		while (opModeIsActive()) {
			position1 += gamepad1.left_stick_y * 0.003;
			position2 += gamepad2.left_stick_y * 0.003;
			claw1.setPosition(position1);
			claw2.setPosition(position2);
			telemetry.addData("position1", position1);
			telemetry.addData("position2", position2);
			telemetry.update();
			sleep(10);
		}
	}
}