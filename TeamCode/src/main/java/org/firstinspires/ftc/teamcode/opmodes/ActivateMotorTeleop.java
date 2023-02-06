package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ActivateMotorTeleop", group = ".Main")
public class ActivateMotorTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {
		DcMotor motor = hardwareMap.get(DcMotor.class, "back_right");
		DcMotor motor2 = hardwareMap.get(DcMotor.class, "front_right");
		waitForStart();

		while (opModeIsActive()) {
			motor.setPower(gamepad1.left_stick_x);
			motor2.setPower(gamepad1.right_stick_x);
		}
	}
}
