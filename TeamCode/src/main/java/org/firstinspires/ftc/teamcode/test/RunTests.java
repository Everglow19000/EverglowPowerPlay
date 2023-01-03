package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.SplinePath;
import org.firstinspires.ftc.teamcode.utils.Trajectory;
import org.junit.Test;

public class RunTests {
    @Test
    public void test() {

        PointD p1 = new PointD(0,0);
        PointD p2 = new PointD(0,240);
        PointD p3 = new PointD(240,240);

        PointD[] points = {p1,p2,p3};

        SplinePath path = new SplinePath(points);
        //AccelerationProfile profile = new AccelerationProfile(3,140,);
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
        }

        for(int i =1 ; i < 30 ; i++){
            PointD p = traj.getPoint(i*0.2);
            System.out.println("(" + p.x + "," + p.y + ")");
        }



    }
}