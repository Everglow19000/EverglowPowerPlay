package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the claw.
 */



public class ClawSystem {
    public enum ServoPosition {
        OPEN(0.36, 0.3), CLOSED(0.29, 0.23);

        ServoPosition(double claw1pos, double claw2pos) {
            this.claw1pos = claw1pos;
            this.claw2pos = claw2pos;
        }

        public final double claw1pos;
        public final double claw2pos;
    }

    private final Servo servo1;
    private final Servo servo2;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public ClawSystem(LinearOpMode opMode) {
        servo1 = opMode.hardwareMap.get(Servo.class, "claw1");
        servo2 = opMode.hardwareMap.get(Servo.class, "claw2");
        servo2.setDirection(Servo.Direction.REVERSE);
        setPosition(ServoPosition.OPEN);
    }

    public void setPosition(ServoPosition location) {
        servo1.setPosition(location.claw1pos);
        servo2.setPosition(location.claw2pos);
    }

}
