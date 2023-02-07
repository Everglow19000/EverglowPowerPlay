package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.RobotParameters;
import java.util.List;

public class Trajectory {
    SplinePath path;
    AccelerationProfile accelerationProfile;
    AccelerationProfile rotationProfile;
    List<Double> uList;

    final double maxVelocity = RobotParameters.MAX_V_X; //Temporary value for tests (vMax = 130-140 cm/sec)
    final double step = 0.01;
    final double pathLength;
    final double totalTime; //In seconds
    final double startAngle;

    public Trajectory(SplinePath path, double startAngle, double endAngle) {
        this.path = path;
        this.startAngle = startAngle;
        uList = getUList(path);
        pathLength = step * uList.size() * path.myPath.length;

        accelerationProfile = new AccelerationProfile(RobotParameters.MAX_A_X, maxVelocity, pathLength);
        rotationProfile = new AccelerationProfile(RobotParameters.MAX_A_ROT, RobotParameters.MAX_V_ROT, endAngle-startAngle);
        totalTime = accelerationProfile.finalTime();
    }

    /**
     * Uses the Runge-Kutta Methods to create a list of u values on the spline path
     * @return A list of u values ranging from 0 to 1
     */
    public List<Double> getUList(SplinePath spline) {
        int xStart = 0;
        int xEnd = 1;

        OdeSolver.Function f = (o) ->{
            PointD p = spline.getDerivative(o);
            return 1. / Math.sqrt(Math.pow(p.x, 2) + Math.pow(p.y, 2));
        };

        return OdeSolver.rungeKutta45(f, step, xStart, xEnd);
    }

    /**
     * Finds the Pose of the robot's powers in a given time.
     * @param time The real time of the robot.
     * @return A Pose object.
     */
    public Pose getVelocity(double time){
//      Finds the u value for a a given time:
        final double distance = accelerationProfile.getPosition(time);
        final int ptIndex = (int) (distance/pathLength * uList.size());
        if(ptIndex >= uList.size()) return null;
        final double nextU = uList.get(ptIndex);

//      Finds the slope of x(u) and y(u):
        PointD vector = path.getDerivative(nextU);
        double xVelocity = vector.x;
        double yVelocity = vector.y;
        double maxPower = Math.max(Math.abs(xVelocity), Math.abs(yVelocity));
        xVelocity /= maxPower;
        yVelocity /= maxPower;

//      Finds the size of the vector:
        final double velocity = accelerationProfile.getVelocity(time);
        xVelocity *= velocity;
        yVelocity *= velocity;

//      Finds the rotation power.
        final double angularVelocity = rotationProfile.getVelocity(time);;

        return new Pose(xVelocity, yVelocity, angularVelocity);
    }

    public Pose getPose(double time){
        final double distance = accelerationProfile.getPosition(time);
        final int ptIndex = (int) (distance/pathLength * uList.size());
        if(ptIndex >= uList.size()) return null;
        double nextU = uList.get(ptIndex);

        PointD position = path.getPoint(nextU);
        double angle = rotationProfile.getPosition(time);

//        return new Pose(position.x,position.y,angle + startAngle);
        return new Pose(position.x,position.y,0);
    }

    public double getTotalTime() {
        return totalTime;
    }
}
