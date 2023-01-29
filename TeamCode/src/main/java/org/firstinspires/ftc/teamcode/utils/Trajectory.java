package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.RobotParameters;
import java.util.List;

public class Trajectory {
    SplinePath path;
    AccelerationProfile profile;
    AccelerationProfile rotationProfile;
    List<Double> uList;

    final double maxVelocity = RobotParameters.MAX_V_X*0.5; //Temporary value for tests (vMax = 130-140 cm/sec)
    final double step = 0.01;
    final double pathLength;
    final double totalTime; //In seconds
    final double startAngle;

    public Trajectory(SplinePath path, double startAngle, double endAngle) {
        this.path = path;
        this.startAngle = startAngle;
        uList = getUList(path);
        pathLength = step * uList.size();

        profile = new AccelerationProfile(RobotParameters.MAX_A_X, maxVelocity, pathLength);
        rotationProfile = new AccelerationProfile(RobotParameters.MAX_A_ROT,
                RobotParameters.MAX_V_ROT, endAngle-startAngle);

        totalTime = profile.finalTime();
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
    public Pose getPowers(double time){
//      Finds the u value for a a given time     :
        final double distance = profile.getPosition(time);
        final int ptIndex = (int) (distance/step);
        if(ptIndex >= uList.size()) return null;
        final double nextU = uList.get(ptIndex);

//      Finds the slope of x(u) and y(u) (the direction of the vector):
        PointD vector = path.getDerivative(nextU);
        double xPower = vector.x;
        double yPower = vector.y;
        double maxPower = Math.max(Math.abs(xPower), Math.abs(yPower));
        xPower /= maxPower;
        yPower /= maxPower;

//      Finds the size of the vector:
        final double velocityPower = profile.getVelocity(time) / RobotParameters.MAX_V_X;
        xPower *= velocityPower;
        yPower *= velocityPower;

//      Finds the rotation power.
        final double angularVelocity = rotationProfile.getVelocity(time);
        final double rotationPower = angularVelocity/RobotParameters.MAX_V_ROT;

        return new Pose(xPower, yPower, 0);
    }

    public Pose getPose(double time){
        final double distance = profile.getPosition(time);
        final int ptIndex = (int) (distance/step);
        if(ptIndex >= uList.size()) return null;
        double nextU = uList.get(ptIndex);

        PointD position = path.getPoint(nextU);
        double angle = rotationProfile.getPosition(time);

//      return new Pose(position.x,position.y,angle + startAngle);
        return new Pose(position.x,position.y,0);
    }

//  Getters & Setters:

    public double getTotalTime() {
        return totalTime;
    }

}
