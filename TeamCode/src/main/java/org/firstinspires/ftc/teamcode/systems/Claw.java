package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the claw.
 */
public class Claw {
    private final Servo servo1;
    private final Servo servo2;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public Claw(LinearOpMode opMode) {
        servo1 = opMode.hardwareMap.get(Servo.class, "servo1");
        servo2 = opMode.hardwareMap.get(Servo.class, "servo2");
        servo2.setDirection(Servo.Direction.REVERSE);
        open();
    }

    /**
     * Closes the claw.
     */
    public void close() {
        final double CLOSE_POSITION = 0.1;
        servo1.setPosition(CLOSE_POSITION);
        servo2.setPosition(CLOSE_POSITION);
    }

    /**
     * Opens the claw.
     */
    public void open() {
        servo1.setPosition(0);
        servo2.setPosition(0);
    }
}
