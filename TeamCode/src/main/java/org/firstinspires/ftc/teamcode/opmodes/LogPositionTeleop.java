package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

import java.io.File;
import java.io.IOException;

@TeleOp(name = "LogPositionTeleop")
public class LogPositionTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        PositionLogger positionLogger = new PositionLogger(drivingSystem);
        waitForStart();
        while (opModeIsActive()) {
            gamepad.update();
            Pose actPowers = new Pose();
            actPowers.x = -gamepad1.left_stick_x;
            actPowers.y = -gamepad1.left_stick_y;
            actPowers.angle = -gamepad1.right_stick_x;
            drivingSystem.driveMecanum(actPowers);
            positionLogger.update();
            drivingSystem.printPosition();
            telemetry.update();
            if (gamepad.circle()){
                drivingSystem.driveMecanum(new Pose(0,0,0)); // stop the robot to prevent it from moving while saving, in case the save takes unreasonably long.
                File fileToCreate = PositionLogger.generateLogFileName();
                telemetry.addData("Saving File to: ", fileToCreate.getAbsolutePath());
                telemetry.update();
                try {
                    positionLogger.saveTo(fileToCreate);
                    positionLogger.clear();
                    telemetry.addData("Successfully saved file to: ", fileToCreate.getAbsolutePath());
                    telemetry.update();
                    sleep(1000);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
