package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "ClawTest")
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        claw2.setDirection(Servo.Direction.REVERSE);
        double position = 0.5;
        waitForStart();
        claw1.setPosition(position);
        claw2.setPosition(position);
        while (opModeIsActive()) {
            position += gamepad1.left_stick_y * 0.01;
            claw1.setPosition(position);
            claw2.setPosition(position);
            telemetry.addData("position", position);
            telemetry.update();
            sleep(10);
        }
    }
}