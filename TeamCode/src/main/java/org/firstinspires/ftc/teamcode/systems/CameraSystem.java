package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.camera.CameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * A Class for handling the image processing for camera.
 */
public class CameraSystem {
	private final LinearOpMode opMode;
	private final CameraPipeline cameraPipeline;
	private final OpenCvCamera camera;

	// Types for AprilTag IDs
	public enum AprilTagType {
		TAG_1,
		TAG_2,
		TAG_3,
		UNIDENTIFIED,
		DETECTION_ERROR,
		INVALID,
		DETECTION_IN_PROGRESS
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public CameraSystem(LinearOpMode opMode) {
		this.opMode = opMode;

		// Initiate camera pipeline
		cameraPipeline = new CameraPipeline(opMode);
		int cameraMonitorViewId =
				this.opMode.hardwareMap.appContext.getResources()
					.getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
		WebcamName webcamName =
				this.opMode.hardwareMap.get(WebcamName.class, "webcam");

		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
		camera.setPipeline(cameraPipeline);

		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				opMode.telemetry.addLine("CAMERA ERROR!!! code: " + errorCode);
			}
		});

		opMode.sleep(4000);
	}

	/**
	 * Take an image capture
	 */
	public void captureImage() {
		cameraPipeline.captureImage();
	}

	/**
	 * Detects AprilTag from camera.
	 *
	 * @return The type of the AprilTag detected.
	 */
	public AprilTagType detectAprilTag() {
		// reset previous april tag id
		cameraPipeline.aprilTagID = AprilTagType.DETECTION_IN_PROGRESS;

		// make the processFrame loop try to detect AprilTags
		cameraPipeline.detectAprilTag();

		ElapsedTime elapsedTime = new ElapsedTime();
		// block the main loop until an AprilTag is detected
		while (opMode.opModeIsActive() && cameraPipeline.aprilTagID == AprilTagType.DETECTION_IN_PROGRESS && elapsedTime.milliseconds() < 8*1000) {
		}

		// return the id
		return cameraPipeline.aprilTagID;
	}
}
