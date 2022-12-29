package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.utils.PathTypes.PolynomialPath;

import java.util.List;

public class Trajectory {

    SplinePath path;
    AccelerationProfile profile;
    List<Double> uList;
    final double trajectoryLength;
    final double vMax = 140; // cm/sec

    Trajectory(SplinePath path, AccelerationProfile profile) {

        this.path = path;
        this.profile = profile;
        trajectoryLength = 0.01 * uList.size();

        uList = derivePath(path);
    }

    public List<Double> derivePath(SplinePath spline) {

        double step = 0.01;
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
        final int index = (int) (distance/0.01);

        if(index >= uList.size()) return null;
        double nextU = uList.get(index);

        return path.getPoint(nextU);
    }
}
