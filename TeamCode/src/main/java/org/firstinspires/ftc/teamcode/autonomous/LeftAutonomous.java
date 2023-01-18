package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LeftAutonomous")
public class LeftAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutonomousRoutes autonomous = new AutonomousRoutes(this);
        waitForStart();
        if (opModeIsActive()) {
            autonomous.run(-1);
        }
    }
}
