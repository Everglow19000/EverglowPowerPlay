package org.firstinspires.ftc.teamcode.utils;

import android.util.Pair;
import static java.lang.Math.abs;
import static java.lang.Math.max;

public class Path {

    public Function x;
    public Function y;
    Function xTag;
    Function yTag;

    public interface Function {
        double function(double u);
    }

    /**
     * Creates a path with the given functions
     *
     * @param x A function that returns the x value of the path at a given u value
     * @param y A function that returns the y value of the path at a given u value
     * @param xTag xTag is the derivative of x with respect to u
     * @param yTag yTag is the derivative of y with respect to u
     */
    public Path(Function x, Function y, Function xTag, Function yTag) {
        this.x = x;
        this.y = y;
        this.xTag = xTag;
        this.yTag = yTag;
    }

    /**
     * Calculates the velocities of the robot for the x, y directions for a specific point on the path.
     *
     * @param u The "tick" value of the path (0 <= u <= 1).
     * @return A PointD object with the x and y velocities.
     */
    private PointD VelocityAtPoint(double u) {
        double yVelocity = yTag.function(u);
        double xVelocity = xTag.function(u);
        double max = max(abs(yVelocity), abs(xVelocity));
        yVelocity /= max;
        xVelocity /= max;

        return new PointD(xVelocity, yVelocity);
    }

    /**
     * Calculates the u value of the next point on the path.
     *
     * @param location The current location of the robot.
     * @param uPrevious The previous u value.
     * @return The next u value.
     */
    private double NearestPointU(PointD location, double uPrevious) {
        //The derivative of the distance squared between the previous point and the path
        Function errorTag = (u) -> 2 * (x.function(u) - location.x) * xTag.function(u)
                + 2 * (y.function(u) - location.y) * yTag.function(u);
        //A small value to add to the u value to try to get as close as possible to the path
        double alpha = 0.01, u = uPrevious;

        //Check when errorTag approaches 0 (minima point) to find the nearest point on the path
        //or when u is out of bounds, which means we reached the end of the path
        while (errorTag.function(u) < -1E-7 && u < 1) {
            u -= alpha * errorTag.function(u);
        }

        return u;
    }

    /**
     * Calculates the velocities of the robot for the x, y directions for the nearest point on the path,
     * tries to fix any errors with an epsilon value, and returns them and the new u value as a Pair<> object.
     *
     * @param location The current location of the robot.
     * @param uPrevious The previous u value.
     * @return A Pair<PointD, Double> object with the x and y velocities as a PointD and the new u value.
     */
    public Pair<PointD, Double> VelocityForPoint(PointD location, double uPrevious) {
        double u = NearestPointU(location, uPrevious);
        PointD velocity = VelocityAtPoint(u);
        PointD error = new PointD(x.function(u) - location.x, y.function(u) - location.y);
        double alpha = 0.01;

        velocity.x += alpha * error.x;
        velocity.y += alpha * error.y;

        return new Pair<>(velocity, u);
    }
}
