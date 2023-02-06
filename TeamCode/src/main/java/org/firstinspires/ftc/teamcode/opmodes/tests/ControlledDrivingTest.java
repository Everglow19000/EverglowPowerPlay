package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Point2D;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.Sequence;

@TeleOp(name = "ControlledDrivingTest", group = ".Main")
public class ControlledDrivingTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SystemCoordinator systems = new SystemCoordinator(this);
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        Pose actPowers = new Pose();

        int typeCount = 4, driveType = 0;

        waitForStart();



        while (opModeIsActive()) {
            gamepadA.update();

            if(gamepadA.square()) {
                systems.drivingSystem.driveToClosestPole();
            }

            // Calculate desired robot velocity
            actPowers.x = -gamepad1.left_stick_x * 0.4;
            actPowers.y = -gamepad1.left_stick_y * 0.4;
            actPowers.angle = -gamepad1.right_stick_x * 0.3;

            if(gamepadA.triangle()) {
                driveType = (driveType + 1) % typeCount;
            }

            switch (driveType) {
                case 0:
                    systems.drivingSystem.driveByAxis(actPowers);
                case 1:
                    systems.drivingSystem.controlledDriveByAxis(actPowers);
                case 2:
                    systems.drivingSystem.controlledDriveByAxis2(actPowers);
                case 3:
                    systems.drivingSystem.controlledDriveByAxis3(actPowers);
            }

            systems.trackingSystem.printPosition();

            Point2D tileLocation = systems.trackingSystem.getTileLocation();
            telemetry.addData("tileLocation.x:", tileLocation.x);
            telemetry.addData("tileLocation.y:", tileLocation.y);
            Point2D tileDeviation = systems.trackingSystem.getTileDeviation();
            telemetry.addData("tileDeviation.x:", tileDeviation.x);
            telemetry.addData("tileDeviation.y:", tileDeviation.y);
            Point2D tileLCenter = systems.trackingSystem.getTileCenter();
            telemetry.addData("tileLCenter.x:", tileLCenter.x);
            telemetry.addData("tileLCenter.y:", tileLCenter.y);

            Point2D closePole = systems.trackingSystem.closestPoleLocation();
            telemetry.addData("closePole.x:", closePole.x);
            telemetry.addData("closePole.y:", closePole.y);

            telemetry.update();

            systems.tick();

        }
    }
}