package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "DriveByAxisTeleOp")
public class DriveByAxisTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();
        Pose actPowers = new Pose(0, 0, 0);
        while (opModeIsActive()) {
            actPowers.x = -gamepad1.left_stick_x;
            actPowers.y = -gamepad1.left_stick_y;
            actPowers.angle = -gamepad1.right_stick_x;
            drivingSystem.driveMecanum(actPowers);

            //drivingSystem.printPosition();
            //telemetry.update();
        }
    }
}
