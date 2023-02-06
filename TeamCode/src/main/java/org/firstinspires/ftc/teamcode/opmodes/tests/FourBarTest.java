package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FourBarTest", group = "Test")
public class FourBarTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		DcMotor fourBarMotor = hardwareMap.get(DcMotor.class, "4bar");
		fourBarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		fourBarMotor.setTargetPosition(0);
		waitForStart();

		while (opModeIsActive()) {
			fourBarMotor.setPower(-gamepad1.left_stick_y * 0.25);
			telemetry.addData("position", fourBarMotor.getCurrentPosition());
			telemetry.update();
		}
	}
}
