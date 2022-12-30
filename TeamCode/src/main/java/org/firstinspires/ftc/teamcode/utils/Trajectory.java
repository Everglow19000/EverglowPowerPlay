package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.utils.PathTypes.PolynomialPath;
import java.util.List;

public class Trajectory {

    SplinePath path;
    List<Double> uList;
    final double trajectoryLength;
    final double vMax = 140; // cm/sec
    final double step = 0.01;

    Trajectory(SplinePath path) {

        this.path = path;
        trajectoryLength = step * uList.size();

        uList = derivePath(path);
    }

    public List<Double> derivePath(SplinePath spline) {

        int xStart = 0;
        int xEnd = 1;

        OdeSolver.Function f;

        f = o ->{
            PointD p = path.getDerivative(o);
            return 1. / Math.sqrt(Math.pow(p.x, 2) + Math.pow(p.y, 2));
        };

        return OdeSolver.rungeKutta45(f, step, xStart, xEnd);
    }

    public PointD getNextPoint(double time){

        final double distance = time*vMax;
        final int index = (int) (distance/step);

        if(index >= uList.size()) return null;
        double nextU = uList.get(index);

        return path.getPoint(nextU);
    }
}
