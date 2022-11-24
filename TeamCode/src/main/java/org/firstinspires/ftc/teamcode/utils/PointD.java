package org.firstinspires.ftc.teamcode.utils;

public class PointD {
    public double x;
    public double y;

    /**
     * Given no values, sets values to 0.
     */
    public PointD() {
        this.x = 0;
        this.y = 0;
    }

    public PointD(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return sqrt(x^2 + y^2).
     */
    public double hyp() {
        return Math.hypot(x, y);
    }
}
