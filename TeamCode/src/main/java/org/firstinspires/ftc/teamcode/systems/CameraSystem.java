package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.AndroidUtils;
import org.firstinspires.ftc.teamcode.utils.CameraCalibration;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;

/**
 * A Class for handling the image processing for camera.
 */
public class CameraSystem {
	private final LinearOpMode opMode;
	private final CameraPipeline cameraPipeline;
	private final OpenCvCamera camera;

	static final double TAG_SIZE = 0.166; // UNITS ARE METERS

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

	// Inline camera pipeline class
	static class CameraPipeline extends OpenCvPipeline {
		private long nativeAprilTagPtr; // A pointer to the JNI AprilTag detector
		private final Mat grey = new Mat(); // A blank canvas to store the b&w image

		private final LinearOpMode opMode; // The OpMode that's currently running
		private boolean isCapturingImage = false;
		private boolean isDetectingAprilTag = false;

		public AprilTagType aprilTagID = AprilTagType.UNIDENTIFIED; // Last identified AprilTag

		public CameraPipeline(LinearOpMode opMode) {
			this.opMode = opMode;

			// initiate AprilTag detector
			nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
		}

		// runs when class is deleted
		@Override
		protected void finalize() {
			// -- safely delete AprilTag detector from memory -- //
			// Might be null if createAprilTagDetector() threw an exception
			if (nativeAprilTagPtr != 0) {
				// Delete the native context we created in the constructor
//                AprilTagDetectorJNI.releaseAprilTagDetector(nativeAprilTagPtr);
				nativeAprilTagPtr = 0;
			} else {
				// handle exceptions
				System.out.println("AprilTagDetectionPipeline.finalize(): nativeAprilTagPtr was NULL");
			}
		}

		/**
		 * Capture an image and save to robot
		 */
		public void captureImage() {
			isCapturingImage = true;
		}

		/**
		 * Run AprilTag detection
		 */
		public void detectAprilTag() {
			isDetectingAprilTag = true;
		}

		// Runs automatically every frame
		@Override
		public Mat processFrame(Mat input) {
			// Image capturing is enabled
			if (isCapturingImage) {
				opMode.telemetry.addLine("Capture");
				opMode.telemetry.update();

				isCapturingImage = false; // turn off capturing an image for the next iteration

				try {
					String timeStamp = AndroidUtils.timestampString(); // get current time
					String filepath = // target path for the image capture
							new File(AppUtil.ROBOT_DATA_DIR, String.format("img_%s.png", timeStamp)).getAbsolutePath();
					// save capture to disk
					saveMatToDiskFullPath(input, filepath);

					opMode.telemetry.addLine("Capturing Image");
					opMode.telemetry.update();
				} catch (Exception e) {
					e.printStackTrace();
				}
			}

			// AprilTag detection is enabled
			if (isDetectingAprilTag) {
				isDetectingAprilTag = false; // Turn off AprilTag detection for the next iteration

				try {
					// convert image to grey
					Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

					// call the detector on the image
					ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, grey, TAG_SIZE, CameraCalibration.FX, CameraCalibration.FY, CameraCalibration.CX, CameraCalibration.CY);

					if (detections.size() > 0) { // detected at least one april tag
						// assign global variable based on result
						switch (detections.get(0).id) {
							case 0:
								aprilTagID = AprilTagType.TAG_1;
								break;
							case 1:
								aprilTagID = AprilTagType.TAG_2;
								break;
							case 2:
								aprilTagID = AprilTagType.TAG_3;
								break;
							default:
								aprilTagID = AprilTagType.INVALID;
						}
					} else { // no AprilTag detected
						aprilTagID = AprilTagType.DETECTION_ERROR;
					}
				} catch (Exception e) {
					throw new RuntimeException(e);
				}
			}
			return input;
		}
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
