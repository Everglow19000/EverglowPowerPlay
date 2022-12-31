package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

import java.io.File;

@TeleOp(name = "PositionLoggerTeleop", group = "Test")
@Disabled
public class PositionLoggerTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        //Pose actPowers = new Pose(0, 0, 0);

        waitForStart();

        if (opModeIsActive()) {
            gamepad.update();

/*
            if (gamepad1.left_stick_button) {
                actPowers.x = -gamepad1.left_stick_x;
                actPowers.y = -gamepad1.left_stick_y;
                actPowers.angle = -gamepad1.right_stick_x;
            }
            drivingSystem.driveMecanum(actPowers);
*/

            drivingSystem.driveStraight(250, 1);
            File fileToCreate = PositionLogger.generateLogFileName("positionLog");
            drivingSystem.positionLogger.saveTo(fileToCreate);

        }
    }
}
