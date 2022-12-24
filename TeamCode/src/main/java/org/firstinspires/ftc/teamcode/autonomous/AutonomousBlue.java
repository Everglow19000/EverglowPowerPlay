package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousBlue")
public class AutonomousBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Autonomous autonomous = new Autonomous(this);
        waitForStart();
        if (opModeIsActive()){
            autonomous.run(-1);
        }
    }
}