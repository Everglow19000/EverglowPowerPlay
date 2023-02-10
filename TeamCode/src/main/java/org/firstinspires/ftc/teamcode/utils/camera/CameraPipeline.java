package org.firstinspires.ftc.teamcode.utils.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.utils.AndroidUtils;
import org.firstinspires.ftc.teamcode.utils.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.Point2D;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.sql.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The Camera Pipeline
 */
public class CameraPipeline extends OpenCvPipeline {
	private final LinearOpMode opMode; // The OpMode that's currently running
	private boolean isCapturingImage = false;
	private boolean isDetectingAprilTag = false;

	//// AprilTag ////
	private long nativeAprilTagPtr; // A pointer to the JNI AprilTag detector
	private final Mat grey = new Mat(); // A blank canvas to store the b&w image

	// AprilTag variables
	static final double TAG_SIZE = 0.166; // UNITS ARE METERS
	public CameraSystem.AprilTagType aprilTagID = CameraSystem.AprilTagType.UNIDENTIFIED; // Last identified AprilTag


	//// Image Detection ////
	public final static double CAM_HEIGHT = 42; // cm
	public final static double CAM_ANGLE = Math.toRadians(57); // Radians - 0 is down
	public final static double F1 = 1385.92, F2 = 1385.92;
	public final static double CX = 951.982, CY = 534.084;

	public final static double CENTER_MASS_X_OFFSET = -13.5, CENTER_MASS_Y_OFFSET = -11;

	private final ConeDetector coneDetector;
//	private final PoleDetector poleDetector;


	// mats used for image detection
	private static final Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
	public static final Mat coeffs = new Mat(8, 1, CvType.CV_64F); // distortion coefficients
	public static final Mat rotateMatrix = new Mat(3, 3, CvType.CV_64F);

	static {
		double[] mtx = {F1, 0, CX, 0, F2, CY, 0, 0, 1};
		cameraMatrix.put(0, 0, mtx);

		double[] coe = {0.117627, -0.248549, 0, 0, 0.107441, 0, 0, 0}; // short for coeffs
		coeffs.put(0,0, coe);

		double[] rMtx = {Math.cos(CAM_ANGLE), 0, -Math.sin(CAM_ANGLE),
				0, 1, 0, Math.sin(CAM_ANGLE), 0, Math.cos(CAM_ANGLE)};
		rotateMatrix.put(0, 0, rMtx);
	}

	public CameraPipeline(LinearOpMode opMode) {
		this.opMode = opMode;

		// initialize camera matrix
		double[] mtx = {F1, 0, CX, 0, F2, CY, 0, 0, 1};
		cameraMatrix.put(0, 0, mtx);

		double[] coe = {0.117627, -0.248549, 0, 0, 0.107441, 0, 0, 0}; // short for coeffs
		coeffs.put(0,0, coe);

		double[] rmtx = {1,0,0,
			0, Math.cos(CAM_ANGLE), -Math.sin(CAM_ANGLE),
			0, Math.sin(CAM_ANGLE), 0, Math.cos(CAM_ANGLE)};
//		double[] rmtx = {Math.cos(CAM_ANGLE), 0, Math.sin(CAM_ANGLE), // Y Axis
//				0, 1, 0,
//				-Math.sin(CAM_ANGLE), 0, Math.cos(CAM_ANGLE)};
		rotateMatrix.put(0, 0, rmtx);

		coneDetector = new ConeDetector(opMode);
//		poleDetector = new PoleDetector(opMode);

		// initiate AprilTag detector
		nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
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
							aprilTagID = CameraSystem.AprilTagType.TAG_1;
							break;
						case 1:
							aprilTagID = CameraSystem.AprilTagType.TAG_2;
							break;
						case 2:
							aprilTagID = CameraSystem.AprilTagType.TAG_3;
							break;
						default:
							aprilTagID = CameraSystem.AprilTagType.INVALID;
					}
				} else { // no AprilTag detected
					aprilTagID = CameraSystem.AprilTagType.DETECTION_ERROR;
				}
			} catch (Exception e) {
				throw new RuntimeException(e);
			}
		}


		coneDetector.detect(input);
//		poleDetector.detect(input);

		return input;
	}

	@Override
	public void init(Mat mat) {
		coneDetector.init(mat);
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

	public static Point2D pointToPosition(Point p) {
		MatOfPoint2f pointMat = new MatOfPoint2f(p); // Convert point to a mat
		MatOfPoint2f result = new MatOfPoint2f(); // holds the result

		// undistort point
		Calib3d.undistortPoints(pointMat, result, cameraMatrix, coeffs, rotateMatrix);

		// extract point from mat
		Point point = result.toList().get(0);

		// negate camera height
		point.y *= -CAM_HEIGHT;
//		point.x *= -CAM_HEIGHT;

		// translate to the center of mass of the robot
//		point.x += CENTER_MASS_X_OFFSET;
//		point.y += CENTER_MASS_Y_OFFSET;

		return new Point2D(point.x, point.y);
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
}
