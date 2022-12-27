package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TestDriveSideways")
@Disabled
public class TestDriveSideways extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutonomousRoutes autonomous = new AutonomousRoutes(this);
        waitForStart();
        if (opModeIsActive()){
            autonomous.testDriveSideways();
        }
    }
}