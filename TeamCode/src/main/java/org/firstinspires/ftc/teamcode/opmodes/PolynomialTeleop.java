package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Path;
import org.firstinspires.ftc.teamcode.utils.PathTypes.PolynomialPath;

@TeleOp(name = "PolynomialTeleop")
public class PolynomialTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {

        EverglowGamepad gamePad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);

        waitForStart();

        if (opModeIsActive()) {
            gamePad.update();

            Path path2 = new PolynomialPath(3, 7, 8, 9,6, 2, 2, 4);
            drivingSystem.driveByPath(path2);
        }
    }
}
