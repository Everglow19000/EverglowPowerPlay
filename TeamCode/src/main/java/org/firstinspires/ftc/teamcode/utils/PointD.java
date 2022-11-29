package org.firstinspires.ftc.teamcode.utils;

public class PointD {
    public double x;
    public double y;

    /**
     * Given no values, sets values to 0 and creates a new PointD object.
     */
    public PointD() {
        this.x = 0;
        this.y = 0;
    }

    /**
     * Given a set of values, sets the values to the given values and creates a new PointD object.
     *
     * @param x position/velocity on the sideways-axis.
     * @param y position/velocity on the forward-axis.
     */
    public PointD(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Calculates the hypotenuse of the triangle created by x, y as its sides.
     *
     * @return sqrt(x^2 + y^2).
     */
    public double hyp() {
        return Math.hypot(x, y);
    }

    /**
     * Converts a Pose object to a PointD object.
     *
     * @param angle the angle to add to the Pose object, as PointD does not have an angle.
     * @return A new Pose object containing the same x, y values as the PointD.
     */
    public Pose toPose(double angle) {
        return new Pose(x, y, angle);
    }
}
