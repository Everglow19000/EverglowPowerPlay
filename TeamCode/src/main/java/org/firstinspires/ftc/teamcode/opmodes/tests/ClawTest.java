package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ClawTest", group = "Test")
public class ClawTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		Servo claw = hardwareMap.get(Servo.class, "claw");
		double pos = 0;
		claw.setPosition(pos);

		waitForStart();
		while (opModeIsActive()) {
			pos += gamepad1.left_stick_y * 0.01;
			claw.setPosition(pos);
			telemetry.addData("pos: ", pos);

			telemetry.update();
			sleep(10);
		}
	}
}