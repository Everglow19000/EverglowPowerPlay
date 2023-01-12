package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.OdeSolver;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;
import org.junit.Test;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

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
        double d = 20;
        AccelerationProfile a = new AccelerationProfile(RobotParameters.MAX_A_Y, RobotParameters.MAX_V_Y, d);

        for (double t = 0; t < 10; t += 0.2)
        {
            System.out.print(t);
            System.out.println(a.position(t));
        }
    }
}