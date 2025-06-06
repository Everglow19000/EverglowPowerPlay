package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class for handling the elevator system.
 */
public class ElevatorSystem {
	/**
	 * Enum encapsulating all the positions the elevator should reach.
	 */
	public enum Level {
		PICKUP(0), PRE_PICKUP(-1025), LOW(-1025), MID(-2914), HIGH(-3300);

		public final int state;

		Level(int state) {
			this.state = state;
		}
	}

	/**
	 * The motor which controls the left side of the elevator.
	 */
	private final DcMotor left;
	/**
	 * The motor which controls the right side of the elevator.
	 */
	private final DcMotor right;

	public ElevatorSystem(OpMode opMode) {
		left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
		right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

		left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		left.setDirection(DcMotor.Direction.REVERSE);
		left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//		left.setTargetPosition(0);
//		right.setTargetPosition(0);
//
//		left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		left.setPower(0.7);
//		right.setPower(0.7);
		setPower(0);
	}

	/**
	 * Moves the elevator to the specified state.
	 *
	 * @param level The level to move the elevator to.
	 */
	public void goTo(Level level) {
		left.setTargetPosition(level.state);
		right.setTargetPosition(level.state);
	}

	public void setPower(double power){
		left.setPower(power);
		right.setPower(power);
	}
}
