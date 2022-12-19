package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class for handling the grabbing wheel.
 */
public class GWheelSystem {
    private final DcMotor motor;
    private final static double POWER = 0.5;

    //These variables control whether the grabbing wheel is running or not.
    private boolean isCollecting = false;
    private boolean isSpitting = false;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public GWheelSystem(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotor.class, "gWheel");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * A function which spins te grabbing wheel to inward to collect the cones.
     * Used in toggleCollecting().
     */
    private void collect() {
        motor.setPower(POWER);
        isCollecting = true;
    }

    /**
     * A function which spins the grabbing wheel outward to spit the cones.
     * Used in toggleSpitting().
     */
    private void spit() {
        motor.setPower(-POWER);
        isSpitting = true;
    }

    /**
     * A function which stops the grabbing wheel and resets the isCollecting and isSpitting variables.
     * Used in toggleCollecting() and toggleSpitting().
     */
    private void stop() {
        motor.setPower(0);
        isSpitting = false;
        isCollecting = false;
    }

    /**
     * A function which collects cones using the grabbing wheel.
     */
    public void toggleCollect() {
        if (isCollecting) {
            stop();
        } else {
            collect();
        }
    }

    /**
     * A function which ejects cones using the grabbing wheel.
     */
    public void toggleSpit() {
        if (isSpitting) {
            stop();
        } else {
            spit();
        }
    }
}
