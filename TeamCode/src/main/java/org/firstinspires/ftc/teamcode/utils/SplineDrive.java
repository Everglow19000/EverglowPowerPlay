package org.firstinspires.ftc.teamcode.utils;

public abstract class SplineDrive {

    //ARRAY_NAME[Y][X] ([ROWS][COLUMNS])
    public static double findDeterminant(double[][] mat){

        double answer;

        answer = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
                - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
                + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

        return answer;
    }

    /**
     * Finds the polynomial function that contain the three given points and point (0,0).
     * @param point1 A point where the robot should pass.
     * @param point2 A point where the robot should pass.
     * @param point3 A point where the robot should pass.
     */
    public static void findSolutions(PointD point1, PointD point2, PointD point3){

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

    public static double[] findParameters(double[][] coeff){

        double[][] d = {
                {coeff[0][0], coeff[0][1], coeff[0][2]},
                {coeff[1][0], coeff[1][1], coeff[1][2]},
                {coeff[2][0], coeff[2][1], coeff[2][2]}
        };

        double[][] dx = {
                {coeff[0][3], coeff[0][1], coeff[0][2]},
                {coeff[1][3], coeff[1][1], coeff[1][2]},
                {coeff[2][3], coeff[2][1], coeff[2][2]}
        };

        double[][] dy = {
                {coeff[0][0], coeff[0][3], coeff[0][2]},
                {coeff[1][0], coeff[1][3], coeff[1][2]},
                {coeff[2][0], coeff[2][3], coeff[2][2]},
        };

        double[][] dz = {
                {coeff[0][0], coeff[0][1], coeff[0][3] },
                {coeff[1][0], coeff[1][1], coeff[1][3] },
                {coeff[2][0], coeff[2][1], coeff[2][3] },
        };

        double D = findDeterminant(d);
        double Dx = findDeterminant(dx);
        double Dy = findDeterminant(dy);
        double Dz = findDeterminant(dz);

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





