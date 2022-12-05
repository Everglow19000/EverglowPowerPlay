package org.firstinspires.ftc.teamcode.utils;

import android.util.Pair;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public abstract class Path {

    public abstract double x(double u);

    public abstract double y(double u);

    public abstract double xTag(double u);

    public abstract double yTag(double u);

    /**
     * Calculates the velocities of the robot for the x, y directions for a specific point on the path.
     *
     * @param u The "tick" value of the path (0 <= u <= 1).
     * @return A PointD object with the x and y velocities.
     */
    private PointD VelocityAtPoint(double u) {
        double yVelocity = yTag(u);
        double xVelocity = xTag(u);
        double max = max(abs(yVelocity), abs(xVelocity));
        yVelocity /= max;
        xVelocity /= max;

        return new PointD(xVelocity, yVelocity);
    }

    /**
     * Calculates the u value of the next point on the path.
     *
     * @param location  The current location of the robot.
     * @param uPrevious The previous u value.
     * @return The next u value.
     */
    private double NearestPointU(PointD location, double uPrevious) {
        //A small value to add to the u value to try to get as close as possible to the path
        double alpha = 0.01, u = uPrevious;

        //Check when errorTag approaches 0 (minima point) to find the nearest point on the path
        //or when u is out of bounds, which means we reached the end of the path
        while (errorTag(u, location) < -1E-7 && u < 1) {
            u -= alpha * errorTag(u, location);
        }

        return u;
    }

    /**
     * Calculates the error between the robot's location and the nearest calculatable point on the path.
     *
     * @param u        The "tick" value of the path (0 <= u <= 1).
     * @param location The current location of the robot in a PointD obj.
     * @return The derivative of the distance squared between the previous point and the path.
     */
    private double errorTag(double u, PointD location) {
        return 2 * (x(u) - location.x) * xTag(u) + 2 * (y(u) - location.y) * yTag(u);
    }

    /**
     * Calculates the velocities of the robot for the x, y directions for the nearest point on the path,
     * tries to fix any errors with an epsilon value, and returns them and the new u value as a Pair<> object.
     *
     * @param location  The current location of the robot.
     * @param uPrevious The previous u value.
     * @return A Pair<PointD, Double> object with the x and y velocities as a PointD and the new u value.
     */
    public Pair<PointD, Double> VelocityForPoint(PointD location, double uPrevious) {
        double u = NearestPointU(location, uPrevious);
        PointD velocity = VelocityAtPoint(u);
        PointD error = new PointD(x(u) - location.x, y(u) - location.y);
        double alpha = 0.01;

        velocity.x += alpha * error.x;
        velocity.y += alpha * error.y;

        return new Pair<>(velocity, u);
    }

    // Used in the research of making this class:
    // https://www.desmos.com/calculator/h3ppebwfmf
}
