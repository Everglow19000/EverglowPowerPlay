package org.firstinspires.ftc.teamcode.utils.camera;

import static org.firstinspires.ftc.teamcode.utils.MathUtils.floatingPrecision;
import static org.firstinspires.ftc.teamcode.utils.camera.CvTools.getLowestPointOfContour;
import static org.firstinspires.ftc.teamcode.utils.camera.CvTools.largestContour;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.Point2D;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ConeDetector {
	private OpMode opmode;
	private Mat hsv;
	private Mat yellowMask1;
	private Mat yellowMask2;
	private Mat yellowMask;

	// thresholds for detection by color
	private static final Scalar YELLOW_THRESHOLD_1_LOWER = new Scalar(0, 100, 20);
	private static final Scalar YELLOW_THRESHOLD_1_UPPER = new Scalar(10, 255, 255);
	private static final Scalar YELLOW_THRESHOLD_2_LOWER = new Scalar(160, 100, 20);
	private static final Scalar YELLOW_THRESHOLD_2_UPPER = new Scalar(180, 255, 255);

	public ConeDetector(OpMode opmode) {
		this.opmode = opmode;
	}

	public void init(Mat mat) {
		hsv = Mat.zeros(mat.size(), mat.type());
		yellowMask1 = Mat.zeros(mat.size(), CvType.CV_8U);
		yellowMask2 = Mat.zeros(mat.size(), CvType.CV_8U);
		yellowMask = Mat.zeros(mat.size(), CvType.CV_8U);
	}

	public void detect(Mat mat) {
		generateConeMask(mat);

		List<MatOfPoint> contours = getContours(yellowMask);
		MatOfPoint largestContour = largestContour(contours);
		if (largestContour == null) {
			opmode.telemetry.addLine("No largest contour");
			opmode.telemetry.update();
			return;
		}
		Point lowestPoint = getLowestPointOfContour(largestContour);

		opmode.telemetry.addData("Num of contours:", contours.size());
		opmode.telemetry.addData("Contour Area:", Imgproc.contourArea(largestContour));
		Imgproc.drawContours(mat, Collections.singletonList(largestContour), 0, new Scalar(30, 255, 30), 20);
		Imgproc.ellipse(mat, lowestPoint, new Size(10f, 10f), 0, 0, 0, new Scalar(255,0,0));

		Point2D pos = CameraPipeline.pointToPosition(lowestPoint);
		opmode.telemetry.addData("Cone Position:", "(" + floatingPrecision(pos.x, 2) + ", " + floatingPrecision(pos.y, 2) + ")");
		opmode.telemetry.update();
		CameraPipeline.pointToPosition(lowestPoint);
	}

	private void generateConeMask(Mat mat) {
		Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_RGB2HSV);
		Core.inRange(hsv, YELLOW_THRESHOLD_1_LOWER, YELLOW_THRESHOLD_1_UPPER, yellowMask1);
		Core.inRange(hsv, YELLOW_THRESHOLD_2_LOWER, YELLOW_THRESHOLD_2_UPPER, yellowMask2);
		Core.bitwise_or(yellowMask1, yellowMask2, yellowMask);
	}

	private List<MatOfPoint> getContours(Mat mat) {
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		return contours;
	}
}
