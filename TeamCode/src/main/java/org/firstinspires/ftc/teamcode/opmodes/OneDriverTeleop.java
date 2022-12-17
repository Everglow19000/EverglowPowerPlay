package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.systems.GWheel;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "OneDriverTeleop")
public class OneDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);

        Pose actPowers = new Pose(0, 0, 0);

        waitForStart();

        while (opModeIsActive()) {
            gamepad.update();

            actPowers.x = -gamepad1.left_stick_x;
            actPowers.y = -gamepad1.left_stick_y;
            actPowers.angle = -gamepad1.right_stick_x;

            drivingSystem.driveMecanum(actPowers);
            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
