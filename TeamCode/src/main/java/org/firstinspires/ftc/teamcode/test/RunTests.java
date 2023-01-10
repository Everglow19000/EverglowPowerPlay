package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.OdeSolver;
import org.junit.Test;

import java.util.List;
import java.util.Locale;

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

    @Test
    public void testAccelerationProfile(){
        AccelerationProfile accelerationProfile = new AccelerationProfile(1, 1, 10);

        for (int i=0;i<10*accelerationProfile.finalTime();i++){
            double t = i*0.1;
             double x =accelerationProfile.position(t);
             double v =accelerationProfile.velocity(t);
             double a =accelerationProfile.acceleration(t);

             System.out.printf(Locale.US, "%s, %s, %s, %s%n", t, a, v, x);
        }
    }
}