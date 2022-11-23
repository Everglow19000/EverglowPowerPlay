package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "OneDriverTeleop")
public class OneDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        Claw claw = new Claw(this);
        FourBar fourBar = new FourBar(this);
        //GWheel gWheel = new GWheel(this);

        waitForStart();
        Pose actPowers = new Pose(0, 0, 0);
        while (opModeIsActive()) {
            gamepad.update();

            actPowers.x = -gamepad1.left_stick_x;
            actPowers.y = -gamepad1.left_stick_y;
            actPowers.angle = -gamepad1.right_stick_x;
            drivingSystem.driveMecanum(actPowers);

            if (gamepad.rt()) {
                claw.close();
            }
            if (gamepad.lt()) {
                claw.open();
            }

            if (gamepad.triangle()) {
                fourBar.goTo(FourBar.Level.PICKUP);
            }
            if (gamepad.circle()) {
                fourBar.goTo(FourBar.Level.DROPOFF);
            }
            if (gamepad.cross()) {
                fourBar.goTo(FourBar.Level.NEUTRAL);
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
