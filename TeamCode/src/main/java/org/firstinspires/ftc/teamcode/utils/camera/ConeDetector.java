package org.firstinspires.ftc.teamcode.utils.camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Point2D;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ConeDetector {
	private OpMode opmode;
	private Mat hsv;
	private Mat redMask1;
	private Mat redMask2;
	private Mat redMask;

	// thresholds for detection by color
	private static final Scalar RED_THRESHOLD_1_LOWER = new Scalar(0, 100, 20);
	private static final Scalar RED_THRESHOLD_1_UPPER = new Scalar(10, 255, 255);
	private static final Scalar RED_THRESHOLD_2_LOWER = new Scalar(160, 100, 20);
	private static final Scalar RED_THRESHOLD_2_UPPER = new Scalar(180, 255, 255);

	public ConeDetector(OpMode opmode) {
		this.opmode = opmode;
	}

	public void init(Mat mat) {
		hsv = Mat.zeros(mat.size(), mat.type());
		redMask1 = Mat.zeros(mat.size(), CvType.CV_8U);
		redMask2 = Mat.zeros(mat.size(), CvType.CV_8U);
		redMask = Mat.zeros(mat.size(), CvType.CV_8U);
	}

	public Point2D detect(Mat mat) {
		generateConeMask(mat);

		List<MatOfPoint> contours = getContours(redMask);
		MatOfPoint largestContour = largestContour(contours);
		Point lowestPoint = getLowestPointOfContour(largestContour);

		opmode.telemetry.addData("Num of contours:", contours.size());
		opmode.telemetry.addData("Contour Area:", Imgproc.contourArea(largestContour));
		Imgproc.drawContours(mat, Collections.singletonList(largestContour), 0, new Scalar(30, 255, 30), 20);

		Point2D pos = CameraPipeline.pointToPosition(lowestPoint);
		opmode.telemetry.addData("Cone Position:", "(" + pos.x+ ", " +pos.y + ")");
		opmode.telemetry.update();
		return CameraPipeline.pointToPosition(lowestPoint);
	}

	private void generateConeMask(Mat mat) {
		Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_RGB2HSV);
		Core.inRange(hsv, RED_THRESHOLD_1_LOWER, RED_THRESHOLD_1_UPPER, redMask1);
		Core.inRange(hsv, RED_THRESHOLD_2_LOWER, RED_THRESHOLD_2_UPPER, redMask2);
		Core.bitwise_or(redMask1, redMask2, redMask);
	}

	private List<MatOfPoint> getContours(Mat mat) {
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		return contours;
	}

	private Point getLowestPointOfContour(MatOfPoint cnt) {
		List<Point> cntList = cnt.toList();
		Point lowestPoint = cntList.get(0);
		double lowestY = lowestPoint.y;

		for (Point p:cntList) {
			if (p.y > lowestY) {
				lowestPoint = p;
				lowestY = p.y;
			}
		}

		return lowestPoint.clone();
	}

	private static MatOfPoint largestContour(List<MatOfPoint> contours) {
		double largestContourArea = 0;
		MatOfPoint largestContour = null;
		for (MatOfPoint contour: contours){
			double area = Imgproc.contourArea(contour);
			if (area > largestContourArea){
				largestContourArea = area;
				largestContour = contour;
			}
		}
		return largestContour;
	}
}
