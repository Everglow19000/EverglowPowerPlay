package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "NewTwoDriverTeleop")
public class NewTwoDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        ClawSystem claw = new ClawSystem(this);
        FourBarSystem fourBar = new FourBarSystem(this);
        GWheelSystem gWheel = new GWheelSystem(this);
        ElevatorSystem elevator = new ElevatorSystem(this);

        Pose actPowers = new Pose(0, 0, 0);
        final int divisorSpeed = 10;

        waitForStart();

        while (opModeIsActive()) {
            gamepadA.update();
            gamepadB.update();

            // If we want to drive and turn slower, for finer adjustment
            if (gamepad1.left_bumper) {
                actPowers.x = -gamepad1.left_stick_x / divisorSpeed;
                actPowers.y = -gamepad1.left_stick_y / divisorSpeed;
                actPowers.angle = -gamepad1.right_stick_x / divisorSpeed;
            } else {
                actPowers.x = -gamepad1.left_stick_x;
                actPowers.y = -gamepad1.left_stick_y;
                actPowers.angle = -gamepad1.right_stick_x;
            }

            drivingSystem.driveMecanum(actPowers);

            if (gamepadB.rt()) {
                claw.setPosition(ClawSystem.ServoPosition.CLOSED);
            }
            if (gamepadB.lt()) {
                claw.setPosition(ClawSystem.ServoPosition.OPEN);
            }

            if (gamepadB.triangle()) {
                fourBar.goTo(FourBarSystem.Level.PICKUP);
            }
            if (gamepadB.circle()) {
                fourBar.goTo(FourBarSystem.Level.DROPOFF);
            }

            if (gamepadB.rb()) {
                gWheel.toggleCollect();
            }
            if (gamepadB.lb()) {
                gWheel.toggleSpit();
            }

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
