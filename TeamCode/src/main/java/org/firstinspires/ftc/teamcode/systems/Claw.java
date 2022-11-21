package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo servo1;
    private final Servo servo2;
    private final double CLOSE_POSITION = 0.1;

    public Claw(LinearOpMode opMode) {
        servo1 = opMode.hardwareMap.get(Servo.class, "servo1");
        servo2 = opMode.hardwareMap.get(Servo.class, "servo2");
        servo2.setDirection(Servo.Direction.REVERSE);
    }

    public void close(){
        servo1.setPosition(CLOSE_POSITION);
        servo2.setPosition(CLOSE_POSITION);
    }

    public void open(){
        servo1.setPosition(0);
        servo2.setPosition(0);
    }
}
