package org.firstinspires.ftc.teamcode.utils;

public class accelerationProfile {
    private double a;
    private double vMax;
    private double d;
    private double t1;// the end time of the first section
    private double t2;// the end time of the second section
    private double tEnd;// the end time
    private double x1;// the length of the first section
    private double x2;// the length of the second section
    private boolean isVersion1;// true --> version1 (the robot gets to max speed), false--> version2 (the robot
    // dosn't get to max speed)

    public accelerationProfile(double a, double vMax, double d){
        this.a = a;
        this.vMax = vMax;
        this.d = d;

        this.isVersion1 = (d/2 > Math.pow(vMax,2)/a);
        if(isVersion1) {
            this.t1 = d / vMax;
            this.t2 = (d / vMax) - vMax / (2 * a);
            this.tEnd = 2*t1 + t2;
            this.x1 = Math.pow(vMax, 2) / a;
            this.x2 = vMax * (t2 - vMax / a);
        }
        else {
           this.t1 = Math.sqrt(d/a);
           this.t2 = t1;
           this.tEnd = t1 + t2;
           this.x1 = d/2;
           this.x2 = x1;
        }

    }

    public double acceleration(double t){
        if(isVersion1) {
            if (t < t1)
                return a;
            else if (t > t1 && t < t2)
                return 0;
            return -a;
        }
        if(t < t1)
            return a;
        return -a;
    }

    public double velocity(double t){
        if(isVersion1) {
            if (t < t1)
                return a * t;
            else if (t > t1 && t < t2)
                return vMax;
            return vMax - a * (t - t2);
        }
        if(t < t1)
            return a*t;
        return a*t1 - a*(t - t1);
    }

    public double position(double t){
        if(isVersion1) {
            if (t < t1)
                return 0.5 * a * Math.pow(t, 2);
            else if (t > t1 && t < t2)
                return vMax * (t - t1) + x1;
            return -0.5 * a * Math.pow(t - t1, 1) + x2 + x1;
        }
        if(t < t1)
            return 0.5*a*Math.pow(t,2);
        return 0.5*a*Math.pow(t1,2) - 0.5*a*Math.pow(t-t1,2);
    }

    public double finalTime(){
        return tEnd;
    }
}
