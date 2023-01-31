package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test270", group = "Template")
public class Test270 extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        final DcMotor fourBarMotor;
        fourBarMotor = hardwareMap.get(DcMotor.class, "4bar");
        double samPower = 0;
        while (opModeIsActive()) {
            if(gamepad1.triangle) {
                fourBarMotor.setPower(0.25);
                samPower+=0.25;
            }

            if(gamepad1.circle) {
                fourBarMotor.setPower(-0.25);
                samPower-=0.25;
            }
            fourBarMotor.setPower(0);
            telemetry.addData("sum power:", samPower);
            telemetry.update();
        }
    }
}