package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.junit.Test;

public class RunTests {
    @Test
    public void test() {

        AccelerationProfile profile = new AccelerationProfile(10,50,100);
        double velocity = profile.getVelocity(4);
        System.out.println(velocity);

        /*PointD p1 = new PointD(0,0);
        PointD p2 = new PointD(0,240);
        PointD p3 = new PointD(240,240);
        PointD p4 = new PointD(360, 360);
        PointD p5 = new PointD(400, 460);


        PointD[] points = {p1,p2,p3,p4,p5};
        SplinePath path = new SplinePath(points);
        Trajectory traj = new Trajectory(path);

        for(int i=0; i<path.myPath.length; i++) {
            System.out.println("a_" + i + "(t)= " + path.myPath[i].aX + "(t" + "-" + i + ")^3+" +
                    path.myPath[i].bX + "(t" + "-" + i + ")^2+" +
                    path.myPath[i].cX + "(t" + "-" + i + ")+" +
                    path.myPath[i].dX);

            System.out.println("b_" + i + "(t)= " + path.myPath[i].aY + "(t" + "-" + i + ")^3+" +
                    path.myPath[i].bY + "(t" + "-" + i + ")^2+" +
                    path.myPath[i].cY + "(t" + "-" + i + ")+" +
                    path.myPath[i].dY + "\n");

            System.out.println("(a_" + i + "(t),b_" + i + "(t))\n");
        }*/
    }
}