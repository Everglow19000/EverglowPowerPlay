package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.systems.GWheel;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "NewTwoDriverTeleOp")
public class NewTwoDriverTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        Claw claw = new Claw(this);
        FourBar fourBar = new FourBar(this);
        GWheel gWheel = new GWheel(this);
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
                claw.setPosition(Claw.ServoPosition.CLOSED);
            }
            if (gamepadB.lt()) {
                claw.setPosition(Claw.ServoPosition.OPEN);
            }

            if (gamepadB.triangle()) {
                fourBar.goTo(FourBar.Level.PICKUP);
            }
            if (gamepadB.circle()) {
                fourBar.goTo(FourBar.Level.DROPOFF);
            }

//            if(gamepad.rb()){
//                gWheel.toggleCollect();
//            }
//            if(gamepad.lb()){
//                gWheel.toggleSpit();
//            }

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
