package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FourBar {
    public enum Level {
        PICKUP(-250), DROPOFF(-500), NEUTRAL(-1000);

        Level(int position) {
            this.position = position;
        }
        private int position;
    }

    private final DcMotor motor;

    public FourBar(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotor.class, "4bar");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.7);
        goTo(Level.NEUTRAL);
    }

    public void goTo(Level level) {
        motor.setTargetPosition(level.position);
    }
}
