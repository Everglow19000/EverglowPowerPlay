package org.firstinspires.ftc.teamcode.utils;

public class PosePIDControler {
    private PIDControler x, y, angle;
    Pose powers = new Pose();

    public PosePIDControler(Pose Kp, Pose Ki, Pose Kd) {
        x = new PIDControler(Kp.x, Ki.x, Kd.x);
        y = new PIDControler(Kp.y, Ki.y, Kd.y);
        angle = new PIDControler(Kp.angle, Ki.angle, Kd.angle);
    }

    public Pose powerByDeviation(Pose deviation) {
        powers.x = x.powerByDeviation(deviation.x);
        powers.y = y.powerByDeviation(deviation.y);
        powers.angle = angle.powerByDeviation(deviation.angle);
        return powers;
    }
}
