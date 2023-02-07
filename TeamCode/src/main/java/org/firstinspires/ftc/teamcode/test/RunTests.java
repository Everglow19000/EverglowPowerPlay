package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.SplinePath;
import org.junit.Test;

public class RunTests {
    @Test
    public void test(){

        PointD[] pts = {
                new PointD(0,0),
                new PointD(0,65),
                new PointD(110,65),
        };

        SplinePath path = new SplinePath(pts);

        for(int i=0; i<path.myPath.length; i++) {
            System.out.println("a_" + i + "(t)= " + path.myPath[i].aX + "(t" + "-" + i + ")^3+" +
                    path.myPath[i].bX + "(t" + "-" + i + ")^2+" +
                    path.myPath[i].cX + "(t" + "-" + i + ")+" +
                    path.myPath[i].dX);

            System.out.println("b_" + i + "(t)= " + path.myPath[i].aY + "(t" + "-" + i + ")^3+" +
                    path.myPath[i].bY + "(t" + "-" + i + ")^2+" +
                    path.myPath[i].cY + "(t" + "-" + i + ")+" +
                    path.myPath[i].dY);

            System.out.println("(a_" + i + "(t),b_" + i + "(t))");

        }
    }
}