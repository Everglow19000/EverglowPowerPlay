package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test270", group = "Template")
public class Test270 extends LinearOpMode {
	@Override
	public void runOpMode() {
		waitForStart();
		final DcMotor fourBarMotor;
		fourBarMotor = hardwareMap.get(DcMotor.class, "4bar");
		double sumPower = 0;
		while (opModeIsActive()) {
			double power = gamepad1.left_stick_x;
			fourBarMotor.setPower(power);
			sumPower += power;
			telemetry.addData("sum power:", sumPower);
			telemetry.update();
		}
	}
}