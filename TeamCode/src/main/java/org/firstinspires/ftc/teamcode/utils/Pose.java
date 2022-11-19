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

    public void normalizeAngle() {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
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
     * @param targetPose A given pose.
     * @return A new pose object with the new values.
     */
    public Pose difference(Pose targetPose) {
        return new Pose(targetPose.x - x, targetPose.y - y,
                targetPose.angle - angle);
    }
}
