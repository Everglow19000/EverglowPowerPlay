package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "ElevatorTestTeleop")
public class ElevatorTestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        DcMotor leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        DcMotor rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            gamepad.update();

            leftElevator.setPower(gamepad1.left_stick_y);
            telemetry.addData("left position", leftElevator.getCurrentPosition());
            telemetry.addData("right position", rightElevator.getCurrentPosition());
            telemetry.update();
        }
    }
}
