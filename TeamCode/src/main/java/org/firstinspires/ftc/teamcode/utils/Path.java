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
     * @return A Point2D object with the x and y velocities.
     */
    private Point2D VelocityAtPoint(double u) {
        double yVelocity = yTag(u);
        double xVelocity = xTag(u);
        double max = max(abs(yVelocity), abs(xVelocity));
        yVelocity /= max;
        xVelocity /= max;

        return new Point2D(xVelocity, yVelocity);
    }

    /**
     * Calculates the u value of the next point on the path.
     *
     * @param location  The current location of the robot.
     * @param uPrevious The previous u value.
     * @return The next u value.
     */
    private double NearestPointU(Point2D location, double uPrevious) {
        //A small value to add to the u value to try to get as close as possible to the path
        double alpha = 1E-4, u = uPrevious, epsilon = -1E-7;

        //Check when errorTag approaches 0 (minima point) to find the nearest point on the path
        //or when u is out of bounds, which means we reached the end of the path
        while (errorTag(u, location) < epsilon && u < 1) {
            u -= alpha * errorTag(u, location);
        }

        return u;
    }

    /**
     * Calculates the error between the robot's location and the nearest calculatable point on the path.
     *
     * @param u        The "tick" value of the path (0 <= u <= 1).
     * @param location The current location of the robot in a Point2D obj.
     * @return The derivative of the distance squared between the previous point and the path.
     */
    private double errorTag(double u, Point2D location) {
        return 2 * (x(u) - location.x) * xTag(u) + 2 * (y(u) - location.y) * yTag(u);
    }

    /**
     * Calculates the velocities of the robot for the x, y directions for the nearest point on the path,
     * tries to fix any errors with an epsilon value, and returns them and the new u value as a Pair<> object.
     *
     * @param location  The current location of the robot.
     * @param uPrevious The previous u value.
     * @return A Pair<Point2D, Double> object with the x and y velocities as a Point2D and the new u value.
     */
    public Pair<Point2D, Double> VelocityForPoint(Point2D location, double uPrevious) {
        double u = NearestPointU(location, uPrevious);
        Point2D velocity = VelocityAtPoint(u);
        Point2D error = new Point2D(x(u) - location.x, y(u) - location.y);
        double alpha = 0.4;

        velocity.x += alpha * error.x;
        velocity.y += alpha * error.y;

        return new Pair<>(velocity, u);
    }

    // Used in the research of making this class:
    // https://www.desmos.com/calculator/h3ppebwfmf
}
