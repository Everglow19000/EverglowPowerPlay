package org.firstinspires.ftc.teamcode.utils;

import java.util.List;

public class Trajectory {

    SplinePath path;
    List<Double> uList;

    final double maxVelocity = 40; //Temporary value for tests (vMax = 130-140 cm/sec)
    final double step = 0.01;
    final double totalLength;
    final double totalTime; //In seconds

    public Trajectory(SplinePath path) {

        this.path = path;
        uList = derivePath(path);
        totalLength = step * uList.size();
        totalTime = totalLength / maxVelocity;
    }

    /**
     * Uses the Runge-Kutta Methods to create a list of u values on the spline path
     * @param spline A spline path
     * @return A list of u values ranging from 0 to 1
     */
    public List<Double> derivePath(SplinePath spline) {

        int xStart = 0;
        int xEnd = 1;

        OdeSolver.Function f = (o) ->{
            PointD p = path.getDerivative(o);
            return 1. / Math.sqrt(Math.pow(p.x, 2) + Math.pow(p.y, 2));
        };

        return OdeSolver.rungeKutta45(f, step, xStart, xEnd);
    }

    /**
     * Finds the point of the robot's position using a given time.
     * @param time The real time of the robot.
     * @return A pointD object.
     */
    public PointD getVelocity(double time){

        final double distance = time* maxVelocity;
        final int ptIndex = (int) (distance/step);

        if(ptIndex >= uList.size()) return null;
        double nextU = uList.get(ptIndex);

        return path.getDerivative(nextU);
    }

    public PointD getPoint(double time){

        final double distance = time* maxVelocity;
        final int ptIndex = (int) (distance/step);

        if(ptIndex >= uList.size()) return null;
        double nextU = uList.get(ptIndex);

        return path.getPoint(nextU);
    }

    public double getTotalTime() {
        return totalTime;
    }

    public double getTotalLength() {
        return totalLength;
    }

    public double getMaxVelocity(){
        return maxVelocity;
    }
}
