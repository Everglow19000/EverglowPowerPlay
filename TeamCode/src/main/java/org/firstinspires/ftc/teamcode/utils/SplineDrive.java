package org.firstinspires.ftc.teamcode.utils;

public abstract class SplineDrive {

//  NOTE: ARRAY_NAME[Y][X] ([ROWS][COLUMNS])

    /**
     * Finds the determinant of a given 3x3 matrix.
     * @param mat A 3x3 matrix.
     * @return The determinant of the given matrix.
     */
    public static double findDeterminant3(double[][] mat){

        double answer;
        answer = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
                - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
                + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

        return answer;
    }

    /**
     * Finds the determinant of a given 2x2 matrix.
     * @param mat A 2x2 matrix.
     * @return The determinant of the given matrix.
     */
    public static double findDeterminant2(double[][] mat){

        double answer;
        answer = mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1];

        return answer;
    }

    /**
     * Finds the polynomial function that contain the three given points and point (0,0).
     * @param point1 A point where the robot should pass.
     * @param point2 A point where the robot should pass.
     * @param point3 A point where the robot should pass.
     */
    public static void findPolynomial(PointD point1, PointD point2, PointD point3){

        double[][] coeffX = new double[3][4];
        double[][] coeffY = new double[3][4];

        PointD x1 = new PointD(0.3,point1.x);
        PointD x2 = new PointD(0.6,point2.x);
        PointD x3 = new PointD(0.9,point3.x);
        PointD y1 = new PointD(0.3,point1.y);
        PointD y2 = new PointD(0.6,point2.y);
        PointD y3 = new PointD(0.9,point3.y);

        int pow = 3;
        for(int i=0; i<3; i++){
            coeffX[0][i] = Math.pow(x1.x,pow);
            coeffX[1][i] = Math.pow(x2.x,pow);
            coeffX[2][i] = Math.pow(x3.x,pow);
            coeffY[0][i] = Math.pow(y1.x,pow);
            coeffY[1][i] = Math.pow(y2.x,pow);
            coeffY[2][i] = Math.pow(y3.x,pow);
            pow--;
        }

        coeffX[0][3] = x1.y;
        coeffX[1][3] = x2.y;
        coeffX[2][3] = x3.y;
        coeffY[0][3] = y1.y;
        coeffY[1][3] = y2.y;
        coeffY[2][3] = y3.y;

        double[] paramX = findParameters(coeffX);
        double[] paramY = findParameters(coeffY);
    }

    /**
     * Finds the polynomial function between two points.
     * @param y0 The y value of the first point.
     * @param m0 The slope in the first point.
     * @param y1 The y value of the second point.
     * @param m1 The slope in the second point.
     */
    public static void findParameters(double y0, double m0, double y1, double m1){

        PointD point0 = new PointD(0,y0);
        PointD point1 = new PointD(1,y1);

        //The matrix of the parameters a, b, and c
        double[][] matrix = new double[2][3];

        //Assign values to the matrix according to the polynomial ax^3+bx^2+cx+d
        //and the derivative 3ax^2+2bx+C; y(0)=y0; y(1)=y1; y'(0)=m0; y'(1)=m1.

        matrix[0][0] = 1; matrix[0][1] = 1; matrix[0][2] = y1 - y0 - m0;
        matrix[1][0] = 3; matrix[1][1] = 2; matrix[1][2] = m1 - m0;

        double[][] D = {
                {matrix[0][0], matrix[0][1]},
                {matrix[1][0], matrix[1][1]},
        };

        double[][] Da = {
                {matrix[0][2], matrix[0][1]},
                {matrix[1][2], matrix[1][1]}
        };

        double[][] Db = {
                {matrix[0][0], matrix[0][2]},
                {matrix[1][0], matrix[1][2]}
        };

        double d = findDeterminant2(D);
        double da = findDeterminant2(Da);
        double db = findDeterminant2(Db);

        double ap = da/d;
        double bp = db/d;
        double cp = m0;
        double dp = y0;
    }

    /**
     * Finds the parameters of a given parametric equation matrix.
     * Should only be called from the method 'findPolynomial'.
     * @param matrix The matrix of the X or Y parametric equation.
     * @return An array of the a, b, c parameters.
     */
    public static double[] findParameters(double[][] matrix){

        double[][] d = {
                {matrix[0][0], matrix[0][1], matrix[0][2]},
                {matrix[1][0], matrix[1][1], matrix[1][2]},
                {matrix[2][0], matrix[2][1], matrix[2][2]}
        };

        double[][] dx = {
                {matrix[0][3], matrix[0][1], matrix[0][2]},
                {matrix[1][3], matrix[1][1], matrix[1][2]},
                {matrix[2][3], matrix[2][1], matrix[2][2]}
        };

        double[][] dy = {
                {matrix[0][0], matrix[0][3], matrix[0][2]},
                {matrix[1][0], matrix[1][3], matrix[1][2]},
                {matrix[2][0], matrix[2][3], matrix[2][2]},
        };

        double[][] dz = {
                {matrix[0][0], matrix[0][1], matrix[0][3] },
                {matrix[1][0], matrix[1][1], matrix[1][3] },
                {matrix[2][0], matrix[2][1], matrix[2][3] },
        };

        double D = findDeterminant3(d);
        double Dx = findDeterminant3(dx);
        double Dy = findDeterminant3(dy);
        double Dz = findDeterminant3(dz);

        if(D!=0){
            double a = Dx / D;
            double b = Dy / D;
            double c = Dz / D;

            double[] param = new double[3];
            param[0] = a; param[1] = b; param[2] = c;

            return param;
        }

        else{
            return null;
        }
    }
}





