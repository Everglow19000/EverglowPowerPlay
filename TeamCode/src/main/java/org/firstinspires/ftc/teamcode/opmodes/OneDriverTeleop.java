package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "OneDriverTeleop")
public class OneDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        ClawSystem claw = new ClawSystem(this);
        FourBarSystem fourBar = new FourBarSystem(this);
        GWheelSystem gWheel = new GWheelSystem(this);
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
                claw.goTo(ClawSystem.Level.CLOSED);
            }
            if (gamepad.lt()) {
                claw.goTo(ClawSystem.Level.OPEN);
            }

            if (gamepad.dpad_down()){
                elevator.goTo(ElevatorSystem.Level.PICKUP);
                fourBar.goTo(FourBarSystem.Level.PICKUP);
            }

            if (gamepad.cross()){
                new Thread(()->{
                    sleep(500);
                    elevator.goTo(ElevatorSystem.Level.PRE_PICKUP);
                }).start();
                fourBar.goTo(FourBarSystem.Level.PICKUP);
            }

            if (gamepad.circle()){
                elevator.goTo(ElevatorSystem.Level.LOW);
                new Thread(()->{
                    sleep(500);
                    fourBar.goTo(FourBarSystem.Level.DROPOFF);
                }).start();
            }

            if (gamepad.triangle()){
                elevator.goTo(ElevatorSystem.Level.MID);
                new Thread(()->{
                    sleep(500);
                    fourBar.goTo(FourBarSystem.Level.DROPOFF);
                }).start();
            }

//            if (gamepad.square()){
//                elevator.goTo(ElevatorSystem.Level.HIGH);
//                new Thread(()->{
//                    sleep(1000);
//                    fourBar.goTo(FourBarSystem.Level.DROPOFF);
//                });
//            }

            if (gamepad.rb()) {
                gWheel.toggleCollect();
            }
            if (gamepad.lb()) {
                gWheel.toggleSpit();
            }

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
