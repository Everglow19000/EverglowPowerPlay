package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;

@TeleOp(name = "One Driver Teleop")
public class OneDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();
        while (opModeIsActive()) {
            drivingSystem.driveMecanum(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }
    }
}
