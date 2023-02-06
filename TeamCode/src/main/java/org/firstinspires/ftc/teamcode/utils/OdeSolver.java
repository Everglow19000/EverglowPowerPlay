package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;

public class OdeSolver {

    public interface Function{
        public double of(double x);
    }

    public static List<Double> rungeKutta45(Function f, double step, double xStart, double x_end){
        List<Double> xValues = new ArrayList<>();
        double x = xStart;

        while (x < x_end){
            xValues.add(x);
            double k1 = f.of(x);
            double k2 = f.of(x + step * k1 / 2.);
            double k3 = f.of(x + step * k2 / 2.);
            double k4 = f.of(x + step * k3);
            x += 1./6. * (k1 + 2 * k2 + 2 * k3 + k4) * step;
        }
        return xValues;
    }
}
