package org.firstinspires.ftc.teamcode.systems;

import java.io.File;
import java.util.ArrayList;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.AndroidUtils;
import org.firstinspires.ftc.teamcode.utils.CameraCalibration;

/**
 * A Class for handling the image processing for the camera.
 */
public class CameraSystem {
	/**
	 * Enum encapsulating the types of AprilTag IDs.
	 */
	public enum AprilTagType {
		TAG_1,
		TAG_2,
		TAG_3,
		UNIDENTIFIED,
		DETECTION_ERROR,
		INVALID,
		DETECTION_IN_PROGRESS
	}

	static final double TAG_SIZE = 0.166; // In meters

	private final LinearOpMode opMode;
	private final CameraPipeline cameraPipeline;
	private final OpenCvCamera camera;

	// Inline camera pipeline class
	static class CameraPipeline extends OpenCvPipeline {
		private final LinearOpMode opMode;

		/**
		 * A pointer to the JNI AprilTag detector.
		 * Might be null if createAprilTagDetector() in the constructor threw an exception.
		 */
		private long nativeAprilTagPtr;
		/**
		 * Last identified AprilTag
		 */
		public AprilTagType aprilTagID = AprilTagType.UNIDENTIFIED;

		private final Mat grey = new Mat(); // A blank canvas to store the b&w image
		private boolean isCapturingImage = false;
		private boolean isDetectingAprilTag = false;

		/**
		 * @param opMode The current opMode running on the robot.
		 */
		public CameraPipeline(LinearOpMode opMode) {
			this.opMode = opMode;

			// Initiate AprilTag detector
			nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
		}

		/**
		 * Runs when class is deleted to free up memory.
		 */
		@Override
		protected void finalize() {
			// Try to safely delete AprilTag detector from memory
			if (nativeAprilTagPtr != 0) {
				// Delete the native context we created in the constructor
				// AprilTagDetectorJNI.releaseAprilTagDetector(nativeAprilTagPtr); TODO: (???)
				nativeAprilTagPtr = 0;
			} else {
				System.out.println("AprilTagDetectionPipeline.finalize(): nativeAprilTagPtr was NULL");
			}
		}

		/**
		 * Enables capturing an image.
		 */
		public void captureImage() {
			isCapturingImage = true;
		}

		/**
		 * Enables running the AprilTag detection.
		 */
		public void detectAprilTag() {
			isDetectingAprilTag = true;
		}

		/**
		 * Processes a captured image. Runs automatically every frame.
		 *
		 * @param input The input image to process, a matrix.
		 * @return The processed image, a matrix.
		 */
		@Override
		public Mat processFrame(Mat input) {
			if (isCapturingImage) {
				opMode.telemetry.addLine("Capture");
				opMode.telemetry.update();

				// Turn off capturing an image for the next iteration
				isCapturingImage = false;

				try {
					// Define a target path for the image capture
					String filepath = new File(
							AppUtil.ROBOT_DATA_DIR,
							String.format("img_%s.png", AndroidUtils.timestampString())
					).getAbsolutePath();
					// Save capture to storage
					saveMatToDiskFullPath(input, filepath);

					opMode.telemetry.addLine("Captured Image");
					opMode.telemetry.update();
				} catch (Exception err) {
					err.printStackTrace();
				}
			}

			if (isDetectingAprilTag) {
				// Turn off AprilTag detection for the next iteration
				isDetectingAprilTag = false;

				try {
					// Convert the image to grey
					Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
					// Detect april tags in the image
					ArrayList<AprilTagDetection> detections =
							AprilTagDetectorJNI.runAprilTagDetectorSimple(
									nativeAprilTagPtr, grey, TAG_SIZE,
									CameraCalibration.FX, CameraCalibration.FY,
									CameraCalibration.CX, CameraCalibration.CY
							);

					// If at least one april tag was detected
					if (detections.size() > 0) {
						// Assign a global variable based on result
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
					} else {
						aprilTagID = AprilTagType.DETECTION_ERROR;
					}
				} catch (Exception err) {
					throw new RuntimeException(err);
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

		// Initiate camera
		int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
				.getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
		WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "webcam");

		// Initiate camera pipeline
		cameraPipeline = new CameraPipeline(opMode);

		// Assign camera variable
		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
		camera.setPipeline(cameraPipeline);

		// Starts the camera
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			/**
			 * Handle camera open success.
			 * Open the FTC dashboard camera stream and start streaming.
			 */
			@Override
			public void onOpened() {
				FtcDashboard dashboard = FtcDashboard.getInstance();
				dashboard.startCameraStream(camera, 24);
				camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
			}

			/**
			 * Handle camera open errors.
			 *
			 * @param errorCode reason for failure.
			 */
			@Override
			public void onError(int errorCode) {
				opMode.telemetry.addLine("CAMERA ERROR!!! code: " + errorCode);
			}
		});

		opMode.sleep(4000);
	}

	/**
	 * Captures an image.
	 */
	public void captureImage() {
		cameraPipeline.captureImage();
	}

	/**
	 * Detects an AprilTag from the camera.
	 *
	 * @return The type of the AprilTag detected.
	 */
	public AprilTagType detectAprilTag() {
		// Reset previous april tag id
		cameraPipeline.aprilTagID = AprilTagType.DETECTION_IN_PROGRESS;

		// Make the processFrame loop try to detect AprilTags
		cameraPipeline.detectAprilTag();

		ElapsedTime elapsedTime = new ElapsedTime();
		// Block the main loop until an AprilTag is detected
		while (opMode.opModeIsActive() &&
				cameraPipeline.aprilTagID == AprilTagType.DETECTION_IN_PROGRESS &&
				elapsedTime.milliseconds() < 8 * 1000) {}

		// Return the id
		return cameraPipeline.aprilTagID;
	}
}
