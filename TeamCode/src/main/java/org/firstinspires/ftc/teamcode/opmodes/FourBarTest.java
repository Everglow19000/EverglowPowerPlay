package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "FourBarTest")
@Disabled
public class FourBarTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		Servo claw = hardwareMap.get(Servo.class, "4bar");
		double position = 0.5;
		waitForStart();
		claw.setPosition(position);
		while (opModeIsActive()) {
			position += gamepad1.left_stick_y * 0.01;
			claw.setPosition(position);
			telemetry.addData("position", position);
			telemetry.update();
			sleep(10);
		}
	}
}