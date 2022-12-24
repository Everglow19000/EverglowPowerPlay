package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousRed")
public class AutonomousRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Autonomous autonomous = new Autonomous(this);
        waitForStart();
        if (opModeIsActive()){
            autonomous.run(1);
        }
    }
}