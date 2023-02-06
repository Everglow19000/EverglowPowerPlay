package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;

@TeleOp(name = "testDrives", group = "Test")
public class testDrives extends LinearOpMode {
    @Override
    public void runOpMode() {
        SystemCoordinator systemCoordinator = new SystemCoordinator(this);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.cross){
                systemCoordinator.drivingSystem.driveStraightSequenceItem(10,50);
            }
            if (gamepad1.circle){
                systemCoordinator.drivingSystem.driveStraightSequenceItem(10,0);
            }
            if (gamepad1.triangle){
                systemCoordinator.drivingSystem.driveSidewaysSequenceItem(10,0);
            }
            if (gamepad1.square){
                systemCoordinator.drivingSystem.driveStraightSequenceItem(10,50);
            }
            telemetry.addData("pos:", systemCoordinator.trackingSystem.getPosition().toString());
        }
    }
}



