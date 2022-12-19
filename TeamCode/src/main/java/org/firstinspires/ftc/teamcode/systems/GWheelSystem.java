package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class for handling the grabbing wheel.
 */
public class GWheelSystem {
    private final DcMotor motor;
    private final static double POWER = 0.5;

    private boolean isCollecting = false;
    private boolean isSpitting = false;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public GWheelSystem(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotor.class, "gWheel");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void collect() {
        motor.setPower(POWER);
        isCollecting = true;
    }

    public void spit() {
        motor.setPower(-POWER);
        isSpitting = true;
    }

    public void stop() {
        motor.setPower(0);
        isSpitting = false;
        isCollecting = false;
    }

    public void toggleCollect() {
        if (isCollecting) {
            stop();
        } else {
            collect();
        }
    }

    public void toggleSpit() {
        if (isSpitting) {
            stop();
        } else {
            spit();
        }
    }
}
