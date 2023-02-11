package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FourBarTest", group = "Test")
public class FourBarTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		int targetPosition = 0;
		DcMotor fourBar = hardwareMap.get(DcMotor.class, "4bar");
		fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		fourBar.setPower(0.5);
		fourBar.setTargetPosition(targetPosition);
		fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		waitForStart();
		while (opModeIsActive()) {
			targetPosition += gamepad1.left_stick_y * 5;
			fourBar.setTargetPosition(targetPosition);
			telemetry.addData("position", fourBar.getCurrentPosition());
			telemetry.addData("targetPosition", targetPosition);
			telemetry.update();
			sleep(10);
		}
	}
}
