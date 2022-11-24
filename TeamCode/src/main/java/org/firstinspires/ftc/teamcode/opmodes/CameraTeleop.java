package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "Camera Teleop")
public class CameraTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        CameraSystem cameraSystem = new CameraSystem(this);
        Pose actPowers = new Pose();

        waitForStart();

        cameraSystem.captureImage();

        while (opModeIsActive()) {
            gamepad.update();

            // assign image capture to the circle button
            if (gamepad.circle()) {
                cameraSystem.captureImage();
            }

            // assign AprilTag detection to cross button
            if (gamepad.cross()) {
                telemetry.addLine("Detecting AprilTag...");

                CameraSystem.AprilTagType aprilTagType = cameraSystem.detectAprilTag();
                telemetry.addData("AprilTag ID:", aprilTagType);
                telemetry.update();
            }

            actPowers.x = -gamepad1.left_stick_x;
            actPowers.y = -gamepad1.left_stick_y;
            actPowers.angle = -gamepad1.right_stick_x;
            drivingSystem.driveMecanum(actPowers);
        }
    }
}
