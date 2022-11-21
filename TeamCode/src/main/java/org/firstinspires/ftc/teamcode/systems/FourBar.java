package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FourBar {
    private final DcMotor motor;
    private final int ticNum = 100;
    public FourBar(LinearOpMode opMode){
        motor = opMode.hardwareMap.get(DcMotor.class, "4bar");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extend(){
        motor.setTargetPosition(ticNum);
    }

    public void retract(){
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
