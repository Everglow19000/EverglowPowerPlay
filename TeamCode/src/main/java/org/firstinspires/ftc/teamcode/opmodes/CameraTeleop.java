package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "Camera Teleop", group = "Test")
public class CameraTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		DrivingSystem drivingSystem = new DrivingSystem(this);
		CameraSystem cameraSystem = new CameraSystem(this);
		//temp for Adi's tests - Ethan's doing
		final DcMotor gWheel = hardwareMap.get(DcMotor.class,"gWheel");
		gWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		Pose actPowers = new Pose();

		waitForStart();

		cameraSystem.captureImage();

		while (opModeIsActive()) {
			gamepad.update();

			// Move robot
			actPowers.x = -gamepad1.left_stick_x;
			actPowers.y = -gamepad1.left_stick_y;
			actPowers.angle = -gamepad1.right_stick_x;
			drivingSystem.driveMecanum(actPowers);

			// Assign image capture to the circle button
			if (gamepad.circle()) {
				cameraSystem.captureImage();
			}

			// Assign AprilTag detection to cross button
			if (gamepad.cross()) {
				telemetry.addLine("Detecting AprilTag...");

				CameraSystem.AprilTagType aprilTagType = cameraSystem.detectAprilTag();
				telemetry.addData("AprilTag ID:", aprilTagType);
				telemetry.update();
			}

			// GWheel for Adi
			if(gamepad.rt()) {
				gWheel.setPower(0.5);
			}
			if(gamepad.lt()) {
				gWheel.setPower(-0.5);
			}
			if(gamepad.triangle()){
				gWheel.setPower(0);
			}
		}
	}
}
