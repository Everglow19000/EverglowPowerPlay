package org.firstinspires.ftc.teamcode.utils.PathTypes;

import org.firstinspires.ftc.teamcode.utils.Path;
import org.firstinspires.ftc.teamcode.utils.Point2D;

public class CirclePath extends Path {

    private final double radius;
    private final Point2D startPoint;

    /**
     * Creates a circle path.
     *
     * @param radius The radius of the circle the robot should travel.
     * @param center The center of the circle the robot should travel.
     */
    public CirclePath(double radius, Point2D center) {
        this.radius = radius;
        this.startPoint = Point2D.difference(center, new Point2D(radius, 0));
    }

    @Override
    public double x(double u) {
        return radius * Math.cos(u * 2 * Math.PI) + startPoint.x;
    }

    @Override
    public double y(double u) {
        return radius * Math.sin(u * 2 * Math.PI) + startPoint.y;
    }

    @Override
    public double xTag(double u) {
        return -2 * Math.PI * radius * Math.sin(u * 2 * Math.PI);
    }

    @Override
    public double yTag(double u) {
        return 2 * Math.PI * radius * Math.cos(u * 2 * Math.PI);
    }
}
