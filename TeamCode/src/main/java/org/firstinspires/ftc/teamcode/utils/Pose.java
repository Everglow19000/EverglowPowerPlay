package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;

/**
 * An object encompassing a set of two dimensional position and orientation.
 */
public class Pose {
    public double x;
    public double y;
    public double angle;

    public Pose() {
        this.x = 0;
        this.y = 0;
        this.angle = 0;
    }

    public Pose(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double hyp() {
        return Math.hypot(x, y);
    }

    public void normalizeAngle() {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
    }

    /**
     * Given any angle, normalizes it such that it is between -PI and PI RADIANS,
     * increasing or decreasing by 2 * PI RADIANS to make it so.
     *
     * @param angle Random angle.
     * @return The angle normalized (-PI < angle < PI).
     */
    public static double normalizeAngle(double angle) {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }

    /**
     * Sets the pose the values of a given pose.
     *
     * @param pose A given pose.
     */
    public void setValue(Pose pose) {
        this.x = pose.x;
        this.y = pose.y;
        this.angle = pose.angle;
    }

    /**
     * Add a given pose's values to the pose's values.
     *
     * @param pose A given pose.
     */
    public void addPose(Pose pose) {
        this.x += pose.x;
        this.y += pose.y;
        this.angle += pose.angle;
    }

    /**
     * Calculate the difference between the values of two poses.
     *
     * @param pose1 A given pose.
     * @param pose2 Another given pose.
     * @return A new pose object containing the differences in values.
     */
    public static Pose difference(Pose pose1, Pose pose2) {
        return new Pose(pose1.x - pose2.x, pose1.y - pose2.y,
                pose1.angle - pose2.angle);
    }
}
