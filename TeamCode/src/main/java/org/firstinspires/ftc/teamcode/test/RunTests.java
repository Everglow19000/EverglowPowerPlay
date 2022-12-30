package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.utils.OdeSolver;
import org.junit.Test;

import java.util.List;

public class RunTests {
    @Test
    public void test(){
        System.out.println("Test running");
    }

    @Test
    public void testOdeSolver(){

        List<Double> result = OdeSolver.rungeKutta45(
                (x)->x,
                0.01,
                1,
                100
        );

        for(double d: result){
            System.out.println(d);
        }

    }
}