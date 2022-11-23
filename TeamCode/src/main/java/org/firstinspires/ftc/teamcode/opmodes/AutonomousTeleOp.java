package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "AutonomousTeleOp")
public class AutonomousTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();
        if (opModeIsActive()) {
            drivingSystem.driveStraight(15, 0.5);
            drivingSystem.driveSideways(15, 0.5);
            drivingSystem.rotate(Math.PI / 2);
            //drivingSystem.moveTo(new Pose(-15, -15, -Math.PI / 2));

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}