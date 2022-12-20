package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class CameraCalibration {
    public static final double FX = 1394.6027293299926;
    public static final double FY = 1394.6027293299926;
    public static final double CX = 995.588675691456;
    public static final double CY = 599.3212928484164;

    public static final double K1 = 0.11480806073904032;
    public static final double K2 = -0.21946985653851792;
    public static final double K3 = 0.11274677130853494;
    public static final double P1 = 0.008564577708855225;
    public static final double P2 = 0.11480806073904032;

    public static Mat constructCameraMatrix() {
        Mat mat = new Mat(3, 3, CvType.CV_32FC1);

        mat.put(0, 0, FX);
        mat.put(0, 1, 0);
        mat.put(0, 2, CX);

        mat.put(1, 0, 0);
        mat.put(1, 1, FY);
        mat.put(1, 2, CY);

        mat.put(2, 0, 0);
        mat.put(2, 1, 0);
        mat.put(2, 2, 1);

        return mat;
    }
}
