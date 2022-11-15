package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Camera Teleop")
public class CameraTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        EverglowGamepad ourGamepad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        CameraSystem cameraSystem = new CameraSystem(this);
        waitForStart();
        cameraSystem.captureImage();
        while (opModeIsActive()){
            if (ourGamepad.circle()){
                cameraSystem.captureImage();
            }
        }
    }
}
