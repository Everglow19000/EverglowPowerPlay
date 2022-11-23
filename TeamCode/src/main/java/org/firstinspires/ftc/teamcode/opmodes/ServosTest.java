package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "ServosTest")
public class ServosTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo left = hardwareMap.get(Servo.class, "left_servo");
        Servo right = hardwareMap.get(Servo.class, "right_servo");
        waitForStart();
        left.setPosition(0.25);
        right.setPosition(0.25);
        while (opModeIsActive()){

        }
    }
}
