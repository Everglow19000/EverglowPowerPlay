package org.firstinspires.ftc.teamcode.utils.camera;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class CvTools {
	public static MatOfPoint largestContour(List<MatOfPoint> contours) {
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

	public static Point getLowestPointOfContour(MatOfPoint cnt) {
		if (cnt == null) return null;
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
}
