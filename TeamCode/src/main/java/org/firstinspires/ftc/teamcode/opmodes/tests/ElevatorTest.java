package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "ElevatorTest", group = "Test")
public class ElevatorTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		ClawSystem claw = new ClawSystem(this);
		FourBarSystem fourBar = new FourBarSystem(this);
		DcMotor leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
		DcMotor rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");

		leftElevator.setDirection(DcMotor.Direction.REVERSE);
		leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		waitForStart();
		while (opModeIsActive()) {
			gamepad.update();
			if (gamepad.square()) {
				claw.goToImmediate(ClawSystem.Position.CLOSED);
			}
			if (gamepad.circle()) {
				claw.goToImmediate(ClawSystem.Position.OPEN);
			}

			if (gamepad.rb()) {
				fourBar.goToImmediate(FourBarSystem.Position.PICKUP);
			}
			if (gamepad.lb()) {
				fourBar.goToImmediate(FourBarSystem.Position.DROPOFF);
			}

			leftElevator.setPower(gamepad1.left_stick_y);
			rightElevator.setPower(gamepad1.left_stick_y);
			telemetry.addData("left position", leftElevator.getCurrentPosition());
			telemetry.addData("right position", rightElevator.getCurrentPosition());

			telemetry.update();
		}
	}
}
