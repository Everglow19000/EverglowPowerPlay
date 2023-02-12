package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "testDeadWeels", group = "Test")
public class testDeadWeels extends LinearOpMode {
    @Override
    public void runOpMode(){
        SystemCoordinator systems = SystemCoordinator.init(this, true);
        Pose powers = new Pose();
        while (opModeIsActive()) {
            powers.x = -gamepad1.left_stick_x * 0.75;
            powers.y = -gamepad1.left_stick_y * 0.75;
            powers.angle = -gamepad1.right_stick_x * 0.5;

            systems.drivingSystem.driveByAxis(powers);
            telemetry.addData("x:", systems.trackingSystem.getPosition().x);
            telemetry.addData("y:", systems.trackingSystem.getPosition().y);
            telemetry.addData("angle:", systems.trackingSystem.getPosition().angle);
            telemetry.update();
        }
    }
}
