package org.firstinspires.ftc.teamcode.utils;

public class PosePIDController {
    private final PIDController x, y, angle;
    Pose powers = new Pose();

    public PosePIDController(Pose Kp, Pose Ki, Pose Kd) {
        x = new PIDController(Kp.x, Ki.x, Kd.x);
        y = new PIDController(Kp.y, Ki.y, Kd.y);
        angle = new PIDController(Kp.angle, Ki.angle, Kd.angle);
    }

    public Pose powerByDeviation(Pose deviation) {
        powers.x = x.powerByDeviation(deviation.x);
        powers.y = y.powerByDeviation(deviation.y);
        powers.angle = angle.powerByDeviation(deviation.angle);
        return powers;
    }
}
