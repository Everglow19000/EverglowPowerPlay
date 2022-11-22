package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "MTF2")
public class MTF2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        getRuntime();
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();

        drivingSystem.move2(new Pose(0, 10, Math.toRadians(0)));

        Pose actPowers = new Pose(0, 0, 0);
        while (opModeIsActive()) {
            actPowers.x = -gamepad1.left_stick_x;
            actPowers.y = -gamepad1.left_stick_y;
            actPowers.angle = -gamepad1.right_stick_x;
            drivingSystem.driveByAxis(actPowers);

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
