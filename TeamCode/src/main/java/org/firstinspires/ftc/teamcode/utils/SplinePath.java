package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.utils.PathTypes.PolynomialPath;

public class SplinePath {

    public PolynomialPath[] myPath;

    public SplinePath(Point2D[] points) {
        myPath = MatrixSolver.findPath(points);
    }

    /**
     * Finds the right path for a provided u and calculates the x and y value of the polynomial.
     * @param providedU A value ranging from 0 to length of the 'polynomials' array.
     * @return A Point2D object which contains the calculated x and y values.
     */
    public Point2D getPoint(double providedU) {
        providedU *= myPath.length;
        int indexU = (int) providedU;
        if(indexU > 1) indexU = myPath.length - 1;
        else if(indexU < 0) indexU = 0;

        double x = myPath[indexU].x(providedU - indexU);
        double y = myPath[indexU].y(providedU - indexU);

        return new Point2D(x,y);
    }

    public Point2D getDerivative(double providedU) {
        providedU *= myPath.length;
        int indexU = (int) providedU;
        if(indexU > 1) indexU = myPath.length - 1;
        else if(indexU < 0) indexU = 0;

        double xTag = myPath[indexU].xTag(providedU - indexU);
        double yTag = myPath[indexU].yTag(providedU - indexU);

        return new Point2D(xTag,yTag);
    }
}





