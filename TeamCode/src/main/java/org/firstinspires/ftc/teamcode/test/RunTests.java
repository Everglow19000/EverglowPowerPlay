package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.SplinePath;
import org.firstinspires.ftc.teamcode.utils.Trajectory;
import org.junit.Test;

public class RunTests {
    @Test
    public void test(){

    }

    @Test
    public void testOdeSolver(){

//        List<Double> result = OdeSolver.rungeKutta45(
//                (x)->x,
//                0.01,
//                1,
//                100
//        );

//        for(double d: result){
//            System.out.println(d);
//        }
        double d = 150;
        AccelerationProfile a = new AccelerationProfile(RobotParameters.MAX_A_X, RobotParameters.MAX_V_X / 2, d);
        System.out.println(a.finalTime());
        System.out.println();

        for (double t = 0; t < 10; t += 0.2)
        {
            System.out.print(t);
            System.out.print(" ");
            System.out.println(a.getPosition(t));
        }
    }

    @org.junit.Test
    public void testSpline(){
        PointD[] pts = {
                new PointD(0,0),
                new PointD(0,100),
                new PointD(0,200)
        };
        SplinePath spline = new SplinePath(pts);
        Trajectory trajectory = new Trajectory(spline, 0, 0.0001);
        System.out.println(trajectory.pathLength);

    }

    @Test
    public void test3(){
        PointD[] pts = {
                new PointD(0,0),
                new PointD(0,75),
                new PointD(0,150),
        };
        SplinePath spline = new SplinePath(pts);
        Trajectory trajectory = new Trajectory(spline, 0, 0.0001);
        for (double t = 0; t < 10; t += 0.01)
        {
            Pose pose = trajectory.getPose(t);
            if (pose != null) {
                System.out.print(t);
                System.out.print(" ");
                System.out.println(pose.y);
            }
        }
    }
}