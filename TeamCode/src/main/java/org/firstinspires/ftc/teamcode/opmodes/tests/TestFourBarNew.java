package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TemplateTeleop", group = "Template")
public class TestFourBarNew extends LinearOpMode {
	@Override
	public void runOpMode() {
		DcMotor fourBarMotor = hardwareMap.get(DcMotor.class, "4bar_motor");
		fourBarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		waitForStart();

		while (opModeIsActive()) {
			fourBarMotor.setPower(-gamepad1.left_stick_y);
			telemetry.addData("position", fourBarMotor.getCurrentPosition());
			telemetry.update();
		}
	}
}
