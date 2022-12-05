package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Path;
import org.firstinspires.ftc.teamcode.utils.PathTypes.CirclePath;
import org.firstinspires.ftc.teamcode.utils.PointD;

@TeleOp(name = "CircleTeleop")
public class CircleTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {

        EverglowGamepad gamePad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);

        waitForStart();

        if (opModeIsActive()) {
            gamePad.update();

            Path path = new CirclePath(30, new PointD(0, 0));
            drivingSystem.driveByPath(path);
        }
    }
}
