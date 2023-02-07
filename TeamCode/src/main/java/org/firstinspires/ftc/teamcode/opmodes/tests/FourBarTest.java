package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FourBarTest", group = "Test")
public class FourBarTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		DcMotor fourBar = hardwareMap.get(DcMotor.class, "4bar");
		fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		fourBar.setTargetPosition(0);
		fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		fourBar.setPower(0);
		fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		waitForStart();
		while (opModeIsActive()) {
			fourBar.setPower(-gamepad1.left_stick_y * 0.25);
			telemetry.addData("position", fourBar.getCurrentPosition());
			telemetry.update();
		}
	}
}
