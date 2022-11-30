package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "SimpleDrive10")
public class SimpleDrive10 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        Claw claw = new Claw(this);
        FourBar fourBar = new FourBar(this);

        drivingSystem.maxDrivePower = 0.1;

        waitForStart();
        Pose actPowers = new Pose(0, 0, 0);
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
