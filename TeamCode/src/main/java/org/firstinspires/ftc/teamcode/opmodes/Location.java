package org.firstinspires.ftc.teamcode.opmodes;

public class Location {
    public double x;
    public double y;
    public double angle;

    public Location(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public void setValue(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public void add(Location plus) {
        this.x += plus.x;
        this.y += plus.y;
        this.angle += plus.angle;
    }

    public Location difference(Location targetLocation) {
        return new Location(targetLocation.x - x, targetLocation.y - y,
                targetLocation.angle - angle);
    }



}
