package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ElevatorSystem {
    private final OpMode opMode;
    private final DcMotor left;
    private final DcMotor right;

    public ElevatorSystem(OpMode opMode) {
        this.opMode = opMode;
        left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
        right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(0);
        right.setTargetPosition(0);
        left.setPower(1);
        right.setPower(1);
    }

    public void goTo(int position){
        left.setTargetPosition(position);
        right.setTargetPosition(position);
    }



}
