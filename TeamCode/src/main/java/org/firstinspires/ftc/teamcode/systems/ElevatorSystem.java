package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ElevatorSystem {
	public enum Level {
		PICKUP(0), PRE_PICKUP(-1833) , LOW(-1833), MID(-2914), HIGH(-2914);

		Level(int position) {
			this.position = position;
		}

		public final int position;
	}

	private final OpMode opMode;
	private final DcMotor left;
	private final DcMotor right;

	public ElevatorSystem(OpMode opMode) {
		this.opMode = opMode;
		left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
		right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

		left.setDirection(DcMotorSimple.Direction.REVERSE);
		left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		left.setTargetPosition(0);
		right.setTargetPosition(0);

		left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		left.setPower(0.7);
		right.setPower(0.7);
	}

	public void goTo(Level level){
		left.setTargetPosition(level.position);
		right.setTargetPosition(level.position);
	}


}
