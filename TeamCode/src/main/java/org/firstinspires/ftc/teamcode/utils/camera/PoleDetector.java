package org.firstinspires.ftc.teamcode.utils.camera;

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
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class PoleDetector {
    private OpMode opmode;
    private Mat hsv;
    private Mat yellowMask1;
    private Mat yellowMask2;
    private Mat yellowMask;

    // thresholds for detection by color
    private static final Scalar YELLOW_THRESHOLD_1_LOWER = new Scalar(0, 100, 20); //need to change this numbers to fit the color of the pole
    private static final Scalar YELLOW_THRESHOLD_1_UPPER = new Scalar(10, 255, 255); //need to change this numbers to fit the color of the pole
    private static final Scalar YELLOW_THRESHOLD_2_LOWER = new Scalar(160, 100, 20); //need to change this numbers to fit the color of the pole
    private static final Scalar YELLOW_THRESHOLD_2_UPPER = new Scalar(180, 255, 255); //need to change this numbers to fit the color of the pole

    public PoleDetector(OpMode opmode) {
        this.opmode = opmode;
    }

    public void init(Mat mat) {
        hsv = Mat.zeros(mat.size(), mat.type());
        yellowMask1 = Mat.zeros(mat.size(), CvType.CV_8U);
        yellowMask2 = Mat.zeros(mat.size(), CvType.CV_8U);
        yellowMask = Mat.zeros(mat.size(), CvType.CV_8U);
    }

    public Point2D detect(Mat mat) {
        generatePoleMask(mat);

        List<MatOfPoint> contours = getContours(yellowMask);
        MatOfPoint largestContour = largestContour(contours);
        Point lowestPoint = getLowestPointOfContour(largestContour);

        opmode.telemetry.addData("Num of contours:", contours.size());
        opmode.telemetry.addData("Contour Area:", Imgproc.contourArea(largestContour));
        Imgproc.drawContours(mat, Collections.singletonList(largestContour), 0, new Scalar(30, 255, 30), 20);

        Point2D pos = CameraPipeline.pointToPosition(lowestPoint);
        opmode.telemetry.addData("Pole Position:", "(" + pos.x+ ", " +pos.y + ")");
        opmode.telemetry.update();
        return CameraPipeline.pointToPosition(lowestPoint);
    }

    private void generatePoleMask(Mat mat) {
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

