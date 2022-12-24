package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "OneDriverTeleop")
public class OneDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        ClawSystem claw = new ClawSystem(this);
        ElevatorSystem elevator = new ElevatorSystem(this);

        Pose actPowers = new Pose(0, 0, 0);
        final int divisorSpeed = 10;

        waitForStart();

        while (opModeIsActive()) {
            gamepad.update();

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

            if (gamepad.rt()) {
                claw.setPosition(ClawSystem.ServoPosition.CLOSED);
            }
            if (gamepad.lt()) {
                claw.setPosition(ClawSystem.ServoPosition.OPEN);
            }

            if (gamepad.dpad_down()) {
                elevator.goTo(ElevatorSystem.Level.PICKUP);
            }

            if (gamepad.circle()) {
                elevator.goTo(ElevatorSystem.Level.LOW);
            }

            if (gamepad.triangle()) {
                elevator.goTo(ElevatorSystem.Level.MID);
            }

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
