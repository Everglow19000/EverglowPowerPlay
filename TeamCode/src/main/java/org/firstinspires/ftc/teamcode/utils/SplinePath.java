package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.utils.PathTypes.PolynomialPath;
import java.util.List;

public class SplinePath {

    PolynomialPath[] myPath;

    SplinePath(PointD[] points) {

        myPath = findPath(points);
    }

    /**
     * Finds the polynomial between each point.
     *
     * @param points The points through which the robot needs to pass through.
     * @return An array of polynomial paths.
     */
    public PolynomialPath[] findPath(PointD[] points) {

        PolynomialPath[] polynomials = new PolynomialPath[points.length - 1];

        final double C = 2;

        double m0X = (points[0].x - points[1].x) / -2;
        double m0Y = (points[0].y - points[1].y) / -2;
        double mfX = (points[points.length - 2].x - points[points.length - 1].x) / -2;
        double mfY = (points[points.length - 2].y - points[points.length - 1].y) / -2;

        double nextM0X = C * m0X;
        double nextM0Y = C * m0Y;

        double maxM = Math.max(Math.abs(nextM0X), Math.abs(nextM0Y));
        nextM0X /= maxM * C;
        nextM0Y /= maxM * C;

        double m1X, m1Y;
        for(int i=0; i<polynomials.length; i++) {

            if (i == polynomials.length - 1){
                m1X = C*mfX;
                m1Y = C*mfY;
            }
            else {
                m1Y = ((points[i].y - points[i + 2].y) / -2)*C;
                m1X = ((points[i].x - points[i + 2].x) / -2)*C;
            }

            polynomials[i] = findPolynomial(points[i], nextM0X, nextM0Y, points[i+1], m1X, m1Y);

            nextM0Y = m1Y;
            nextM0X = m1X;
        }

        return polynomials;
    }

    /**
     * Finds the polynomial function between two points, which is a segment of the entire movement.
     *
     * @param point0 The initial point of the segment.
     * @param m0X    The slope of the X parameter equation in the first point.
     * @param m1X    The slope of the X parameter equation in the first point.
     * @param point1 The final point of the segment.
     * @param m0Y    The slope of the Y parameter equation in the first point.
     * @param m1Y    The slope of the Y parameter equation in the second point.
     */
    public PolynomialPath findPolynomial(PointD point0, double m0X, double m0Y,
                                                PointD point1, double m1X, double m1Y) {

        //Declaring the x and y matrices of parameters a and b.
        double[][] yMatrix = new double[2][3];
        double[][] xMatrix = new double[2][3];

        //Assign values to the matrix according to the polynomial ax^3+bx^2+cx+d
        //and the derivative 3ax^2+2bx+C; y(0)=y0; y(1)=y1; y'(0)=m0; y'(1)=m1, and the same for x.
        yMatrix[0][0] = 1; yMatrix[0][1] = 1; yMatrix[0][2] = point1.y - point0.y - m0Y;
        yMatrix[1][0] = 3; yMatrix[1][1] = 2; yMatrix[1][2] = m1Y - m0Y;

        xMatrix[0][0] = 1; xMatrix[0][1] = 1; xMatrix[0][2] = point1.x - point0.x - m0X;
        xMatrix[1][0] = 3; xMatrix[1][1] = 2; xMatrix[1][2] = m1X - m0X;

        double[] solutionY = solveMatrix(yMatrix);
        double[] solutionX = solveMatrix(xMatrix);

        return new PolynomialPath(
                solutionX[0], solutionX[1], m0X, point0.x, solutionY[0], solutionY[1], m0Y, point0.y
        ); //parameters a, b, c=m0, and d=y0.
    }

    /**
     * Finds the parameters of a given 2x2 matrix.
     *
     * @param matrix A matrix.
     * @return The parameters of the matrix.
     */
    public double[] solveMatrix(double[][] matrix) {

        double[][] d = {
                {matrix[0][0], matrix[0][1]},
                {matrix[1][0], matrix[1][1]},
        };

        double[][] da = {
                {matrix[0][2], matrix[0][1]},
                {matrix[1][2], matrix[1][1]}
        };

        double[][] db = {
                {matrix[0][0], matrix[0][2]},
                {matrix[1][0], matrix[1][2]}
        };

        double D = findDeterminant(d);
        double Da = findDeterminant(da);
        double Db = findDeterminant(db);

        double ap = Da / D;
        double bp = Db / D;

        return new double[]{ap, bp};
    }

    /**
     * Finds the determinant of a given 2x2 matrix in the form of Array_name[y][x] ([rows][columns]).
     *
     * @param matrix A 2x2 matrix.
     * @return The determinant of the given matrix.
     */
    public double findDeterminant(double[][] matrix) {
        return matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];
    }

    /**
     * Finds the right path for a provided u and calculates the x and y value of the polynomial.
     * @param providedU A value ranging from 0 to length of the 'polynomials' array.
     * @return A pointD object which contains the calculated x and y values.
     */
    public PointD getPoint(double providedU) {

        providedU *= myPath.length;
        int intU = (int) providedU;

        double x = myPath[intU].x(providedU - intU);
        double y = myPath[intU].y(providedU - intU);

        return new PointD(x,y);
    }

    public PointD getDerivative(double providedU) {

        providedU *= myPath.length;
        int intU = (int) providedU;

        double xTag = myPath[intU].xTag(providedU - intU);
        double yTag = myPath[intU].yTag(providedU - intU);

        return new PointD(xTag,yTag);
    }
}





