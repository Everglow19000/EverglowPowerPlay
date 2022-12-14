package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.systems.GWheel;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "GWheelTeleop")
public class GWheelTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);

        GWheel gWheel = new GWheel(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad.triangle()) {
                telemetry.addData("hello", "triangle");
                telemetry.update();
                gWheel.toggleCollect();
            }
            if (gamepad.circle()) {
                telemetry.addData("hello", "circle");
                telemetry.update();
                gWheel.toggleSpit();
            }
        }
    }
}
