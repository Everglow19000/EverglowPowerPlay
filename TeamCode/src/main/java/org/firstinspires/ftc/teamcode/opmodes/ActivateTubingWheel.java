package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "ActivateTubingWheel")
public class ActivateTubingWheel extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        DcMotor le, re;
        //le = this.hardwareMap.get(DcMotor.class, "left_elevator");
        re = this.hardwareMap.get(DcMotor.class, "gWheel");
        //le.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        re.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(opModeIsActive()){
            //le.setPower(gamepad1.left_stick_y /2);
            re.setPower(gamepad1.right_stick_y / 2);
        }
    }
}



