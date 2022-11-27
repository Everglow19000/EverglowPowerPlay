package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.systems.GWheel;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "TwoDriverTeleOp")
public class TwoDriverTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        Claw claw = new Claw(this);
        FourBar fourBar = new FourBar(this);
//        GWheel gWheel = new GWheel(this);

        Pose actPowers = new Pose(0, 0, 0);
        final int divisorSpeed = 10;

        waitForStart();

        while (opModeIsActive()) {
            gamepadA.update();
            gamepadB.update();

            // If we want to drive and turn slower, for finer adjustment
            if (gamepad1.left_bumper) {
                actPowers.x = -gamepad1.left_stick_x / divisorSpeed;
                actPowers.y = -gamepad1.left_stick_y / divisorSpeed;
                actPowers.angle = -gamepad1.right_stick_x / divisorSpeed;
            } else {
                actPowers.x = -gamepad1.left_stick_x;
                actPowers.y = -gamepad1.left_stick_y;
                actPowers.angle = -gamepad1.right_stick_x;
            }

            drivingSystem.driveMecanum(actPowers);

            if (gamepadB.rt()) {
                claw.close();
            }
            if (gamepadB.lt()) {
                claw.open();
            }

            if (gamepadB.triangle()) {
                fourBar.goTo(FourBar.Level.PICKUP);
            }
            if (gamepadB.circle()) {
                fourBar.goTo(FourBar.Level.DROPOFF);
            }
            if (gamepadB.cross()) {
                fourBar.goTo(FourBar.Level.NEUTRAL);
            }

//            if(gamepad.rb()){
//                gWheel.toggleCollect();
//            }
//            if(gamepad.lb()){
//                gWheel.toggleSpit();
//            }

            drivingSystem.printPosition();
            telemetry.addData("Angle 1", (drivingSystem.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle));
            telemetry.addData("Angle 2", (drivingSystem.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).secondAngle));
            telemetry.addData("Angle 3", (drivingSystem.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle));
            telemetry.update();
        }
    }
}
