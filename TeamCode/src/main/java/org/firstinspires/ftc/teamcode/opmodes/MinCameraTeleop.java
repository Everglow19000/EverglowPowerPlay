package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "MinCameraTeleop", group = "Test")
@Disabled
public class MinCameraTeleop extends LinearOpMode {

	static class Pipeline extends OpenCvPipeline {
		@Override
		public Mat processFrame(Mat input) {
			return input;
		}
	}

	@Override
	public void runOpMode() {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
		OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
		camera.setPipeline(new Pipeline());
		camera.openCameraDevice();
		camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

		DcMotor motor = hardwareMap.get(DcMotor.class, "front_right");

		waitForStart();
		while (opModeIsActive()) {
			motor.setPower(gamepad1.right_stick_x);
		}
		motor.setPower(0); // even this doesn't work or get called, for some reason.
	}
}
