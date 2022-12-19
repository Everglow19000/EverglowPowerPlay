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

        Servo claw = hardwareMap.get(Servo.class, "claw");
        double position = 0.5;
        waitForStart();
        claw.setPosition(position);
        while (opModeIsActive()) {
            position += gamepad1.left_stick_y * 0.01;
            claw.setPosition(position);
            telemetry.addData("position", position);
            telemetry.update();
            sleep(10);
        }
    }
}