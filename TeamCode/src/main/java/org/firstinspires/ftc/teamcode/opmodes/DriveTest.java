package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "DriveTest")
public class DriveTest extends LinearOpMode {
	private DcMotor frontRight;
	private DcMotor frontLeft;
	private DcMotor backRight;
	private DcMotor backLeft;

	@Override
	public void runOpMode() throws InterruptedException {
		frontRight = this.hardwareMap.get(DcMotor.class, "front_right");
		frontLeft = this.hardwareMap.get(DcMotor.class, "front_left");
		backLeft = this.hardwareMap.get(DcMotor.class, "back_left");
		backRight = this.hardwareMap.get(DcMotor.class, "back_right");

		while(opModeIsActive()){
			if(gamepad1.triangle){
				frontLeft.setPower(0.5);
			} else {
				frontLeft.setPower(0);
			}

			if(gamepad1.circle){
				frontRight.setPower(0.5);
			} else {
				frontRight.setPower(0);
			}

			if(gamepad1.cross){
				backLeft.setPower(0.5);
			} else {
				backLeft.setPower(0);
			}

			if(gamepad1.square){
				backRight.setPower(0.5);
			} else {
				backRight.setPower(0);
			}
		}
	}
}