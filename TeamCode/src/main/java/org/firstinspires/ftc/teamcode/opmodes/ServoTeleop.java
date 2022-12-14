package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;

@TeleOp(name = "ServoTeleop")
public class ServoTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        FourBar fourBar = new FourBar(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad.triangle()) {
                fourBar.goTo(FourBar.Level.PICKUP);
            }
            if (gamepad.circle()) {
                fourBar.goTo(FourBar.Level.DROPOFF);
            }
            if (gamepad.cross()) {
                fourBar.goTo(FourBar.Level.NEUTRAL);
            }
        }
    }
}
