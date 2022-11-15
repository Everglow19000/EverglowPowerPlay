package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;

@TeleOp(name = "TestMotorsTeleOp")
public class TestMotorsTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();
        while (opModeIsActive()) {
            drivingSystem.frontRight.setPower(gamepad1.square ? 1 : 0);
            drivingSystem.backRight.setPower(gamepad1.circle ? 1 : 0);
            drivingSystem.frontLeft.setPower(gamepad1.cross ? 1 : 0);
            drivingSystem.backLeft.setPower(gamepad1.triangle ? 1 : 0);
            telemetry.addLine(String.valueOf(getRuntime()));
            telemetry.update();
        }
    }
}
