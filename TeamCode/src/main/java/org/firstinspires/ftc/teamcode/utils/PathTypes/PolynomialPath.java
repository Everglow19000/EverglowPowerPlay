package org.firstinspires.ftc.teamcode.utils.PathTypes;

import org.firstinspires.ftc.teamcode.utils.Path;

public class PolynomialPath extends Path {

    public final double aX, bX, cX, dX;
    public final double aY, bY, cY, dY;

    /**
     * Creates a polynomial path.
     * @param aX The coefficient of the x^3 term.
     * @param bX The coefficient of the x^2 term.
     * @param cX The coefficient of the x^1 term.
     * @param dX The coefficient of the x^0 term.
     * @param aY The coefficient of the y^3 term.
     * @param bY The coefficient of the y^2 term.
     * @param cY The coefficient of the y^1 term.
     * @param dY The coefficient of the y^0 term.
     */
    public PolynomialPath(double aX, double bX, double cX, double dX, double aY, double bY, double cY, double dY) {
        this.aX = aX;
        this.bX = bX;
        this.cX = cX;
        this.dX = dX;
        this.aY = aY;
        this.bY = bY;
        this.cY = cY;
        this.dY = dY;
    }

    @Override
    public double x(double u) {
        return aX * Math.pow(u, 3) + bX * Math.pow(u, 2) + cX * u + dX;
    }

    @Override
    public double y(double u) {
        return aY * Math.pow(u, 3) + bY * Math.pow(u, 2) + cY * u + dY;
    }

    @Override
    public double xTag(double u) {
        return 3 * aX * Math.pow(u, 2) + 2 * bX * u + cX;
    }

    @Override
    public double yTag(double u) {
        return 3 * aY * Math.pow(u, 2) + 2 * bY * u + cY;
    }
}
