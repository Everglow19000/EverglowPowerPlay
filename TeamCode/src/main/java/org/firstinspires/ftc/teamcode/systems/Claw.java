package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the claw.
 */
public class Claw {
    public enum ServoPosition {
        OPEN(0.67),CLOSED(0.168);

        ServoPosition(double position) {
            this.position = position;
        }

        public double position;
    }
    private final Servo servo1;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public Claw(LinearOpMode opMode) {
        servo1 = opMode.hardwareMap.get(Servo.class, "claw");
        setPosition(ServoPosition.OPEN);
    }

    public void setPosition(double position){
        servo1.setPosition(position);
    }

    public void setPosition(ServoPosition location){
        setPosition(location.position);
    }

}
